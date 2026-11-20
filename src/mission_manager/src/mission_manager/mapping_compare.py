"""Replay the exact same filtered 2D input on separate private ROS masters."""
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

from mission_manager.mapping_engine import Process
from mission_manager.mapping_store import atomic_json


def prepare_input(source, destination, seconds=None):
    import rosbag
    count=0;tf_count=0;first=None;last=None
    with rosbag.Bag(str(source)) as original,rosbag.Bag(str(destination),'w') as output:
        static=[]
        for topic,msg,t in original.read_messages(topics=['/scan','/tron/wheel_odom_raw','/tf','/tf_static']):
            if topic=='/scan':
                if first is None:first=t.to_sec()
                if seconds is not None and t.to_sec()>first+seconds:break
                last=t.to_sec();count+=1
            if topic in ['/tf','/tf_static']:
                msg.transforms=[tr for tr in msg.transforms if tr.header.frame_id.lstrip('/')!='map' and tr.child_frame_id.lstrip('/')!='map']
                if not msg.transforms:continue
                tf_count+=len(msg.transforms)
            output.write(topic,msg,t,connection_header=dict(type=msg._type,md5sum=msg._md5sum,message_definition=msg._full_text,latching='1') if topic=='/tf_static' else None)
    if count<2:raise ValueError('비교에 필요한 /scan 데이터가 부족합니다.')
    hasher=hashlib.sha256()
    with destination.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            hasher.update(block)
    digest=hasher.hexdigest()
    return dict(scan_count=count,input_tf_count=tf_count,duration_sec=last-first,sha256=digest)


def live_trajectory(source, period=0.05):
    """map->base_Link (t, x, y, yaw) from the live SLAM TF recorded in the
    session BAG, at the scan times. Empty when the live map->odom is missing."""
    import math
    import rosbag
    import rospy
    import tf2_py
    import warnings
    warnings.filterwarnings('ignore',message='(translation|rotation) should be of type')
    core=tf2_py.BufferCore(rospy.Duration(36000))
    stamps=[]
    with rosbag.Bag(str(source)) as bag:
        for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static','/scan']):
            if topic=='/scan':stamps.append(msg.header.stamp.to_sec());continue
            for transform in msg.transforms:
                if topic=='/tf_static':core.set_transform_static(transform,'bag')
                else:core.set_transform(transform,'bag')
    rows=[];last=None
    for t in sorted(stamps):
        if last is not None and t-last<period:continue
        try:transform=core.lookup_transform_core('map','base_Link',rospy.Time.from_sec(t))
        except Exception:continue
        q=transform.transform.rotation;p=transform.transform.translation
        rows.append([t,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]);last=t
    return rows,stamps


def run_live(source, root, saved, stop=None, progress=None, cloud=None):
    """No 2D replay: the live SLAM map saved during the drive is the 2D result
    and its recorded trajectory places the raw LiDAR for the 3D map."""
    root=Path(root);directory=root/('compare_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    report=dict(state='RUNNING',mode='live',input=dict(source=Path(source).name),results=[dict(saved)],methods=['live','3d'],
                accuracy='외부 실측 없음: 이 결과만으로 정확도를 판정할 수 없습니다.')
    atomic_json(root/'comparison.json',report)
    def tell(index,label,fraction,eta=None):
        if progress:progress(dict(step=index,steps=2,label=label,fraction=max(0.,min(1.,fraction)),eta_sec=eta,detail=''))
    try:
        tell(1,'실시간 경로 읽기',0.)
        rows,_=live_trajectory(source)
        atomic_json(directory/'trajectory.json',rows);report['input']['trajectory_poses']=len(rows)
        tell(1,'실시간 경로 읽기',1.,0)
        key3=directory.name+'_3d';started=time.monotonic()
        def cloud_progress(fraction,detail=''):
            if stop and stop.is_set():raise RuntimeError('지도 생성 중단됨')
            elapsed=time.monotonic()-started
            tell(2,'3D 지도 (원본 LiDAR)',fraction,elapsed*(1-fraction)/fraction if fraction>.05 else None)
        try:
            result=cloud(directory/'trajectory.json',root/key3,cloud_progress)
            result['key']=key3;report['results'].append(result)
        except Exception as error:
            report['cloud_error']=str(error)
        report['state']='COMPLETE'
    except Exception as error:
        report.update(state='FAILED',error=str(error))
        raise
    finally:
        atomic_json(root/'comparison.json',report)
    return report


def run_comparison(source, root, stop=None, progress=None, cloud=None):
    """2D SLAM Toolbox replay of the recorded scan/odom/TF, then (when a raw
    LiDAR BAG of the same run exists) a 3D point cloud along its trajectory.

    progress(dict) receives step/label/fraction/eta_sec updates.
    cloud(trajectory_path, output_dir, progress_fn) builds the 3D result, or
    is None when no raw LiDAR recording is available."""
    root=Path(root);directory=root/('compare_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    prepared=directory/'replay.bag'
    evidence=prepare_input(source,prepared)
    steps=[('toolbox','2D 지도 (SLAM Toolbox 재생)')]+([('3d','3D 지도 (원본 LiDAR)')] if cloud else [])
    report=dict(state='RUNNING',input=evidence,results=[],methods=[m for m,_ in steps],
                accuracy='외부 실측 없음: 이 결과만으로 정확도를 판정할 수 없습니다.')
    if not cloud:report['cloud_note']='원본 LiDAR 기록이 없어 3D 지도는 만들지 않았습니다.'
    atomic_json(root/'comparison.json',report)
    def tell(index,fraction,eta=None,detail=''):
        if progress:
            progress(dict(step=index+1,steps=len(steps),label=steps[index][1],fraction=max(0.,min(1.,fraction)),
                          eta_sec=None if eta is None else max(0.,eta),detail=detail))
    try:
        # Step 1: 2D replay at recorded speed (SLAM timing depends on it).
        method='toolbox'
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1')
        env.pop('ROS_HOSTNAME',None)
        core=Process(['roscore','-p',str(port)],directory/(method+'_master.log'),env)
        worker=None
        try:
            worker=Process([sys.executable,__file__,'worker',method,str(prepared),str(directory/method)],directory/(method+'_worker.log'),env)
            started=time.monotonic();expected=evidence['duration_sec']+12.
            deadline=started+max(120,evidence['duration_sec']*2+90)
            while worker.alive:
                if (stop and stop.is_set()) or time.monotonic()>deadline:raise RuntimeError('지도 생성 중단 또는 처리 시간 초과')
                elapsed=time.monotonic()-started
                tell(0,elapsed/expected*.97,expected-elapsed)
                time.sleep(.5)
            if worker.child.returncode:raise RuntimeError(method+' 처리 실패: '+str(directory/(method+'_worker.log')))
            result=json.loads((directory/method/'result.json').read_text())
            key=directory.name+'_'+method
            (directory/method).rename(root/key)
            result['key']=key;report['results'].append(result)
            atomic_json(root/'comparison.json',report)
            tell(0,1.,0)
        finally:
            if worker:worker.stop()
            core.stop()
        # Step 2: 3D cloud along the replayed 2D trajectory.
        if cloud:
            if stop and stop.is_set():raise RuntimeError('지도 생성 중단됨')
            trajectory=root/key/'trajectory.json'
            if not trajectory.is_file():raise RuntimeError('2D 재생 궤적이 저장되지 않았습니다.')
            key3=directory.name+'_3d';started=time.monotonic()
            def cloud_progress(fraction,detail=''):
                if stop and stop.is_set():raise RuntimeError('지도 생성 중단됨')
                elapsed=time.monotonic()-started
                eta=elapsed*(1-fraction)/fraction if fraction>.05 else None
                tell(1,fraction,eta,detail)
            try:
                result=cloud(trajectory,root/key3,cloud_progress)
                result['key']=key3;report['results'].append(result)
            except Exception as error:
                # 3D failure never discards the finished 2D map.
                report['cloud_error']=str(error)
            atomic_json(root/'comparison.json',report)
        report['state']='COMPLETE'
    except Exception as error:
        report.update(state='FAILED',error=str(error))
        raise
    finally:
        atomic_json(root/'comparison.json',report)
        if prepared.exists():prepared.unlink()
    return report


def worker(method, bag_path, output):
    import xmlrpc.client
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from mission_manager.mapping_engine import mapper_command,save_grid,save_graph
    for _ in range(100):
        try:
            xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/comparison');break
        except OSError:time.sleep(.1)
    rospy.init_node('mapping_comparison',anonymous=True,disable_signals=True)
    rospy.set_param('/use_sim_time',True)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    latest=[None];received=[0]
    def on_map(m):latest[0]=m
    stamps=[]
    def on_scan(m):received[0]+=1;stamps.append(m.header.stamp.to_sec())
    import tf2_ros
    buffer=tf2_ros.Buffer(rospy.Duration(36000));listener=tf2_ros.TransformListener(buffer)
    sub=rospy.Subscriber('/comparison/map',OccupancyGrid,on_map,queue_size=1)
    scan_sub=rospy.Subscriber('/scan',LaserScan,on_scan,queue_size=1000)
    command=mapper_command(method,'/comparison',output.parent)
    mapper=Process(command,output.parent/(method+'_mapper.log'))
    start=time.monotonic();player=None
    try:
        # Wait for the actual algorithm's scan subscriber, not an arbitrary sleep.
        import rosgraph
        master=rosgraph.Master(rospy.get_name());ready=False
        for _ in range(200):
            pubs,subs,services=master.getSystemState()
            if any(t=='/scan' and '/comparison/mapper' in names for t,names in subs):ready=True;break
            if not mapper.alive:raise RuntimeError('SLAM 시작 실패')
            time.sleep(.1)
        if not ready:raise RuntimeError('SLAM /scan 구독 준비 시간 초과')
        player=Process(['rosbag','play','--clock','--delay=1','--rate=1',str(bag_path)],output.parent/(method+'_replay.log'))
        while player.alive:
            if not mapper.alive:raise RuntimeError('SLAM이 재생 도중 종료됨')
            time.sleep(.2)
        if player.child.returncode:raise RuntimeError('BAG 재생 실패')
        time.sleep(3)
        if latest[0] is None:raise RuntimeError('SLAM 지도 출력이 없습니다.')
        result=save_grid(latest[0],output)
        result.update(method=method,wall_seconds=time.monotonic()-start,delivered_scans=received[0],graph_saved=False)
        if method=='toolbox':
            try:result['graph_saved']=save_graph('/comparison',output)
            except Exception as error:result['graph_error']=str(error)
        trajectory=sample_trajectory(buffer,stamps)
        atomic_json(output/'trajectory.json',trajectory)
        result['trajectory_poses']=len(trajectory)
        atomic_json(output/'result.json',result)
    finally:
        listener.unregister()
        if player:player.stop()
        mapper.stop();sub.unregister();scan_sub.unregister();rospy.signal_shutdown('comparison finished')

def sample_trajectory(buffer, stamps, period=0.05):
    """map->base_Link (t, x, y, yaw) at the replayed scan times, thinned to
    `period`. Times before SLAM published map->odom are skipped."""
    import math
    import rospy
    rows=[];last=None
    for t in sorted(stamps):
        if last is not None and t-last<period:continue
        try:transform=buffer.lookup_transform('map','base_Link',rospy.Time.from_sec(t))
        except Exception:continue
        q=transform.transform.rotation;p=transform.transform.translation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rows.append([t,p.x,p.y,yaw]);last=t
    return rows

if __name__=='__main__':
    worker(sys.argv[2],sys.argv[3],sys.argv[4])
