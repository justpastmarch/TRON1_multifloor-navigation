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


def run_comparison(source, root, stop=None):
    root=Path(root);directory=root/('compare_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    prepared=directory/'replay.bag'
    evidence=prepare_input(source,prepared)
    report=dict(state='RUNNING',input=evidence,results=[],accuracy='외부 실측 없음: 이 결과만으로 정확도 우열을 판정할 수 없습니다.')
    atomic_json(root/'comparison.json',report)
    try:
        for method in ['toolbox','gmapping']:
            if stop and stop.is_set():raise RuntimeError('비교 중단됨')
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
            env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1')
            env.pop('ROS_HOSTNAME',None)
            core=Process(['roscore','-p',str(port)],directory/(method+'_master.log'),env)
            worker=None
            try:
                worker=Process([sys.executable,__file__,'worker',method,str(prepared),str(directory/method)],directory/(method+'_worker.log'),env)
                deadline=time.monotonic()+max(120,evidence['duration_sec']*2+90)
                while worker.alive:
                    if (stop and stop.is_set()) or time.monotonic()>deadline:raise RuntimeError('비교 중단 또는 처리 시간 초과')
                    time.sleep(.2)
                if worker.child.returncode:raise RuntimeError(method+' 처리 실패: '+str(directory/(method+'_worker.log')))
                result=json.loads((directory/method/'result.json').read_text())
                # Library results are direct children; move only completed artifacts.
                key=directory.name+'_'+method
                (directory/method).rename(root/key)
                result['key']=key;report['results'].append(result)
                atomic_json(root/'comparison.json',report)
            finally:
                if worker:worker.stop()
                core.stop()
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
    def on_scan(m):received[0]+=1
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
        atomic_json(output/'result.json',result)
    finally:
        if player:player.stop()
        mapper.stop();sub.unregister();scan_sub.unregister();rospy.signal_shutdown('comparison finished')

if __name__=='__main__':
    worker(sys.argv[2],sys.argv[3],sys.argv[4])
