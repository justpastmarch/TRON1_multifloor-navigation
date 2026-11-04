"""Owned mapping processes; no velocity commands or robot transport."""
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

class Process:
    def __init__(self, command, log, env=None):
        self.log=open(log,'a')
        self.child=subprocess.Popen(command,env=env,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
    def stop(self):
        for sig,timeout in [(signal.SIGINT,12),(signal.SIGTERM,4),(signal.SIGKILL,2)]:
            if self.child.poll() is not None:break
            try:os.killpg(self.child.pid,sig)
            except ProcessLookupError:break
            try:self.child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:continue
        self.log.close()
    @property
    def alive(self):return self.child.poll() is None


def mapper_command(method, namespace, directory, resume=None):
    import rosparam
    namespace=namespace.rstrip('/')
    node=namespace+'/mapper'
    if method=='toolbox':
        params=yaml.safe_load(Path('/opt/ros/noetic/share/slam_toolbox/config/mapper_params_online_async.yaml').read_text())
        params.update(base_frame='base_Link',odom_frame='odom',map_frame='map',scan_topic='/scan',
                      max_laser_range=30.,minimum_laser_range=.35,minimum_travel_distance=.05,
                      minimum_travel_heading=.05,map_update_interval=1.,resolution=.05,
                      minimum_time_interval=0.,throttle_scans=1,do_loop_closing=True)
        if resume:
            params.update(map_file_name=str(resume),map_start_at_dock=True)
        package,executable='slam_toolbox','async_slam_toolbox_node'
    elif method=='gmapping':
        if resume:raise ValueError('Gmapping은 이 화면에서 그래프 이어쓰기를 지원하지 않습니다.')
        params=dict(base_frame='base_Link',odom_frame='odom',map_frame='map',maxRange=30.,maxUrange=29.5,
                    minimumScore=0.,linearUpdate=.05,angularUpdate=.05,temporalUpdate=1.,
                    map_update_interval=1.,delta=.05,particles=30)
        package,executable='gmapping','slam_gmapping'
    else:raise ValueError('지원하지 않는 매핑 방식입니다.')
    rosparam.upload_params(node,params)
    Path(directory).mkdir(parents=True,exist_ok=True)
    (Path(directory)/(method+'_params.yaml')).write_text(yaml.safe_dump(params,sort_keys=True))
    return ['rosrun',package,executable,'__name:=mapper','__ns:='+namespace,
            'scan:=/scan','map:='+namespace+'/map','/map:='+namespace+'/map',
            'map_metadata:='+namespace+'/map_metadata','/map_metadata:='+namespace+'/map_metadata']


def save_grid(message, directory):
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from mission_manager.mapping_store import map_metadata
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    topic='/mapping_save_'+str(os.getpid())
    publisher=rospy.Publisher(topic,OccupancyGrid,queue_size=1,latch=True)
    try:
        publisher.publish(message)
        result=subprocess.run(['rosrun','map_server','map_saver','-f',str(directory/'map'),'map:='+topic],
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=20)
        if result.returncode:raise RuntimeError(result.stdout.decode(errors='replace')[-1200:])
        metadata=yaml.safe_load((directory/'map.yaml').read_text())
        metadata['image']='map.pgm'
        (directory/'map.yaml').write_text(yaml.safe_dump(metadata,sort_keys=False))
        return map_metadata(directory)
    finally:publisher.unregister()


def save_graph(namespace, directory):
    import rospy
    from slam_toolbox_msgs.srv import SerializePoseGraph
    from multifloor_manager.ros_services import BoundedServiceCaller
    service=namespace.rstrip('/')+'/mapper/serialize_map'
    rospy.wait_for_service(service,timeout=5)
    BoundedServiceCaller(15).call(service,rospy.ServiceProxy(service,SerializePoseGraph),str(Path(directory)/'graph'))
    for name in ['graph.posegraph','graph.data']:
        if not (Path(directory)/name).is_file():raise RuntimeError('그래프 저장 응답 후 파일이 없습니다: '+name)
    return True
