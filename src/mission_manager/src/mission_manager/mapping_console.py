"""Mapping-only backend in the existing mission_manager process."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import urlparse,parse_qs

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

from mission_manager.mapping_engine import Process,mapper_command,save_grid,save_graph
from mission_manager.mapping_store import MapLibrary,atomic_json

class MappingConsole:
    def __init__(self, root, env=None):
        self.library=MapLibrary(root);self.env=env or os.environ
        self.lock=threading.RLock();self.worker=ThreadPoolExecutor(max_workers=1)
        self.cancel_event=threading.Event();self.jobs={};self.requests={}
        self.session=None;self.mapper=None;self.recorder=None;self.grid=None
        self.scan_received=0.;self.scan_stamp=0.;self.map_received=0.
        self.buffer=tf2_ros.Buffer(rospy.Duration(15));self.listener=tf2_ros.TransformListener(self.buffer)
        self.subs=[rospy.Subscriber('/mapping/map',OccupancyGrid,self.on_map,queue_size=1),
                   rospy.Subscriber('/scan',LaserScan,self.on_scan,queue_size=1)]
        self.server=None;self.closed=False

    def on_map(self,msg):
        with self.lock:self.grid=msg;self.map_received=time.monotonic()
    def on_scan(self,msg):
        with self.lock:self.scan_received=time.monotonic();self.scan_stamp=msg.header.stamp.to_sec()

    def snapshot(self):
        with self.lock:
            return dict(mode='mapping',active=False,locations=[],session=self.session,
                        scan_age_sec=time.monotonic()-self.scan_received if self.scan_received else None,
                        map_age_sec=time.monotonic()-self.map_received if self.map_received else None,
                        mapper_alive=bool(self.mapper and self.mapper.alive),
                        recording=bool(self.recorder and self.recorder.alive),
                        jobs=list(self.jobs.values())[-20:],maps=self.library.list(),
                        root=str(self.library.root),motion_commands_sent=0)

    def map_snapshot(self):
        from mission_manager.console_map import encode_map
        with self.lock:message=self.grid
        return encode_map(message) if message else dict(error='아직 작성된 지도가 없습니다.')

    def mapping_get(self, raw_path):
        url=urlparse(raw_path);parts=url.path.strip('/').split('/');q=parse_qs(url.query)
        if url.path=='/api/mapping/bags':
            roots=[Path(self.env.get('MAPPING_BAG_ROOT','/mnt/ethan/tron-bags/live/ui-tests'))]
            rows=[]
            for root in roots:
                if root.is_dir():
                    for p in sorted(root.glob('*.bag'),reverse=True):
                        if '_raw' not in p.name:rows.append(dict(id=p.name,name=p.name,bytes=p.stat().st_size))
            return rows[:100]
        if len(parts)==5 and parts[:2]==['api','maps']:
            _,_,identifier,key,action=parts
            if action=='preview':return self.library.preview(identifier,key)
            if action=='file':return self.library.file(identifier,key,q.get('name',[''])[0]),'application/octet-stream'
        raise ValueError('지원하지 않는 지도 조회입니다.')

    def mapping_request(self,path,data):
        operations={'/api/mapping/start':self.start,'/api/mapping/save':self.save,
                    '/api/mapping/stop':self.stop,'/api/mapping/compare':self.compare,
                    '/api/mapping/import_compare':self.import_compare}
        if path not in operations:raise ValueError('지도 작성 모드에서는 이 명령을 지원하지 않습니다.')
        request_id=data.get('request_id')
        if not isinstance(request_id,str) or not request_id or len(request_id)>100:raise ValueError('요청 ID가 필요합니다.')
        with self.lock:
            if request_id in self.requests:
                prior=self.requests[request_id]
                if prior[0]!=path or prior[1]!=data:raise ValueError('같은 요청 ID로 다른 작업을 요청할 수 없습니다.')
                return dict(accepted=True,job_id=prior[2])
            busy=any(j['state'] in ['RUNNING','QUEUED'] for j in self.jobs.values())
            if busy and path=='/api/mapping/stop':self.cancel_event.set()
            if busy and path!='/api/mapping/stop':
                raise ValueError('지도 작업을 처리 중입니다. 완료 상태를 확인하세요.')
            identifier=uuid.uuid4().hex
            self.jobs[identifier]=dict(id=identifier,operation=path,state='QUEUED',detail='대기 중')
            self.requests[request_id]=(path,dict(data),identifier)
            if not busy:self.cancel_event.clear()
            self.worker.submit(self._execute,identifier,operations[path],data)
        return dict(accepted=True,job_id=identifier)

    def _execute(self,identifier,operation,data):
        with self.lock:self.jobs[identifier].update(state='RUNNING',detail='처리 중')
        try:
            result=operation(data)
            with self.lock:self.jobs[identifier].update(state='COMPLETE',detail='완료',result=result)
        except Exception as error:
            with self.lock:self.jobs[identifier].update(state='FAILED',detail=str(error))
            rospy.logwarn('Mapping task failed: %s',error)

    def start(self,data):
        import rosnode
        if self.mapper and self.mapper.alive:raise ValueError('이미 지도 작성 중입니다.')
        conflict={'/amcl','/move_base','/map_server','/slam_toolbox','/slam_gmapping'} & set(rosnode.get_node_names())
        if conflict:raise ValueError('지도 작성과 충돌하는 기존 실행을 종료하세요: '+', '.join(sorted(conflict)))
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if self.cancel_event.is_set():raise RuntimeError('지도 시작 중단됨')
            if time.monotonic()-self.scan_received<2:
                try:self.buffer.lookup_transform('odom','base_Link',rospy.Time(0),rospy.Duration(.1));break
                except Exception:pass
            time.sleep(.2)
        else:raise ValueError('/scan 또는 odom→base_Link TF가 없습니다. 센서 연결 상태를 확인하세요.')
        if self.recorder:self.stop({})
        resume=None
        if data.get('resume_id'):
            folder=self.library.result_path(data['resume_id'],data.get('resume_key'))
            if not all((folder/f).is_file() for f in ['graph.posegraph','graph.data']):raise ValueError('이어서 작성할 그래프 파일이 없습니다.')
            resume=folder/'graph'
        identifier,directory=self.library.create(data.get('name',''))
        self.session=dict(id=identifier,name=data['name'],state='STARTING',resume_id=data.get('resume_id'),resume_key=data.get('resume_key'))
        self.grid=None;self.map_received=0
        try:
            self.recorder=Process(['rosbag','record','-O',str(directory/'input.bag'),'/scan','/tron/wheel_odom_raw','/tf','/tf_static'],directory/'record.log')
            # Record subscriptions must exist before SLAM starts.
            time.sleep(.5)
            if not self.recorder.alive:raise RuntimeError('비교용 BAG 기록을 시작하지 못했습니다.')
            self.mapper=Process(mapper_command('toolbox','/mapping',directory,resume),directory/'toolbox.log')
            deadline=time.monotonic()+30
            while self.map_received==0 and time.monotonic()<deadline:
                if not self.mapper.alive:raise RuntimeError('SLAM Toolbox 시작 실패: '+str(directory/'toolbox.log'))
                if self.cancel_event.is_set():raise RuntimeError('지도 시작 중단됨')
                time.sleep(.2)
            if self.map_received==0:raise RuntimeError('SLAM 지도 출력 대기 시간이 지났습니다. 센서 시각과 TF를 확인하세요.')
            self.session['state']='MAPPING'
            atomic_json(directory/'capture.json',dict(topics=['/scan','/tron/wheel_odom_raw','/tf','/tf_static'],source='existing MID360 scan pipeline',motion_commands_sent=0))
            return dict(session_id=identifier)
        except Exception:
            self.stop({})
            raise

    def save(self,data):
        if not self.session or not self.mapper or not self.mapper.alive:raise ValueError('작성 중인 지도가 없습니다.')
        message=self.grid
        if message is None:raise ValueError('저장할 지도 데이터가 없습니다.')
        directory=self.library.path(self.session['id'])
        key='saved_'+time.strftime('%H%M%S_')+uuid.uuid4().hex[:6]
        target=directory/key
        result=save_grid(message,target)
        result.update(method='toolbox',graph_saved=False)
        try:result['graph_saved']=save_graph('/mapping',target)
        except Exception as error:result['graph_error']=str(error)
        atomic_json(target/'result.json',result)
        return dict(session_id=self.session['id'],key=key,**result)

    def stop(self,data):
        if self.recorder:self.recorder.stop();self.recorder=None
        if self.mapper:self.mapper.stop();self.mapper=None
        if self.session:self.session['state']='STOPPED'
        return dict(stopped=True,bag_saved=bool(self.session and (self.library.path(self.session['id'])/'input.bag').is_file()))

    def compare(self,data):
        from mission_manager.mapping_compare import run_comparison
        if self.mapper and self.mapper.alive:raise ValueError('작성 종료 후 비교하세요. 저장과 BAG 마감이 먼저입니다.')
        directory=self.library.path(data.get('session_id'))
        source=directory/'input.bag'
        if not source.is_file():raise ValueError('저장이 완료된 비교용 BAG이 없습니다.')
        return run_comparison(source,directory,self.cancel_event)

    def import_compare(self,data):
        from mission_manager.mapping_compare import run_comparison
        name=data.get('bag_id','')
        if not isinstance(name,str) or Path(name).name!=name or not name.endswith('.bag'):raise ValueError('잘못된 BAG 이름입니다.')
        source=Path(self.env.get('MAPPING_BAG_ROOT','/mnt/ethan/tron-bags/live/ui-tests'))/name
        if not source.is_file():raise ValueError('BAG을 찾을 수 없습니다.')
        if self.mapper and self.mapper.alive:raise ValueError('지도 작성 종료 후 비교하세요.')
        identifier,directory=self.library.create(data.get('name') or name)
        # Make a minimal immutable 2D capture, leaving the original bag unchanged.
        from mission_manager.mapping_compare import prepare_input
        prepare_input(source,directory/'input.bag')
        atomic_json(directory/'source.json',dict(source=str(source)))
        return run_comparison(directory/'input.bag',directory,self.cancel_event)

    def close(self):
        if self.closed:return
        self.closed=True;self.cancel_event.set()
        self.worker.shutdown(wait=True)
        self.stop({})
        for sub in self.subs:sub.unregister()
        self.listener.unregister()
        if self.server:self.server.close()


def main():
    from mission_manager.mission_console import ConsoleServer
    import rospkg
    default='/mnt/ethan/tron-bags/mapping' if os.path.ismount('/mnt/ethan') else str(Path.home()/'TRON1_maps/mapping')
    backend=MappingConsole(rospy.get_param('~mapping_output_root',os.environ.get('MAPPING_OUTPUT_ROOT',default)))
    html=Path(rospkg.RosPack().get_path('mission_manager'))/'config/mapping_console.html'
    class NoDoor:
        def __init__(self,*args):pass
        def snapshot(self):return dict(reason='지도 작성 모드')
        def close(self):pass
    backend.server=ConsoleServer(backend,html,int(rospy.get_param('~console_port',8766)),door_factory=NoDoor)
    rospy.on_shutdown(backend.close)
    rospy.loginfo('Mapping GUI ready: http://127.0.0.1:%s',backend.server.http.server_port)
    rospy.spin()
    backend.close()
    return 0
