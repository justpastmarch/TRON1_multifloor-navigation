"""Read-only map and NAV overlays, hosted by the existing mission node."""
import base64
import hashlib
import math
import struct
import threading
import time
import zlib

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from multifloor_manager.msg import FloorState


def heading(q):
    if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)):
        raise ValueError('invalid orientation')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def encode_map(message):
    info=message.info
    w,h=info.width,info.height
    if not 0<w*h<=16000000 or len(message.data)!=w*h or not math.isfinite(info.resolution) or info.resolution<=0:
        raise ValueError('invalid occupancy grid')
    origin=[info.origin.position.x,info.origin.position.y,heading(info.origin.orientation)]
    if not all(math.isfinite(x) for x in origin):raise ValueError('invalid map origin')
    pixels=bytes(150 if v<0 else round(245*(1-min(100,v)/100)) for v in message.data)
    raw=b''.join(b'\x00'+pixels[y*w:(y+1)*w] for y in range(h-1,-1,-1))
    def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')
    meta=dict(width=w,height=h,resolution=info.resolution,origin=origin,frame=message.header.frame_id.lstrip('/'))
    mask=bytes(v>=0 for v in message.data)
    xmin,ymin,xmax,ymax=w,h,-1,-1
    for y in range(h):
        row=mask[y*w:(y+1)*w];first=row.find(b'\x01')
        if first>=0:
            xmin=min(xmin,first);xmax=max(xmax,row.rfind(b'\x01'));ymin=min(ymin,y);ymax=y
    meta['bounds']=[xmin,h-1-ymax,xmax-xmin+1,ymax-ymin+1] if xmax>=0 else [0,0,w,h]
    meta['revision']=hashlib.sha256(png+str(meta).encode()).hexdigest()[:20]
    meta['image']='data:image/png;base64,'+base64.b64encode(png).decode()
    return meta


class ConsoleMap:
    def __init__(self):
        self.lock=threading.RLock()
        self.grid=None
        self.paths={}
        self.goal=None
        self.goal_id=None
        self.floor_key=None
        self.floor_ready=False
        self.floor_received=0.
        self.ended=False
        self.error='지도 수신 대기'
        self.buffer=tf2_ros.Buffer(rospy.Duration(10))
        self.listener=tf2_ros.TransformListener(self.buffer)
        self.subscribers=[
            rospy.Subscriber('/map',OccupancyGrid,self.on_map,queue_size=1),
            rospy.Subscriber('/move_base/NavfnROS/plan',Path,lambda m:self.on_path('global',m),queue_size=1),
            rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan',Path,lambda m:self.on_path('local',m),queue_size=1),
            rospy.Subscriber('/move_base/current_goal',PoseStamped,self.on_goal_pose,queue_size=1),
            rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,self.on_goal,queue_size=1),
            rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.on_result,queue_size=1),
            rospy.Subscriber('/multifloor/floor_state',FloorState,self.on_floor,queue_size=1)]

    def close(self):
        for sub in self.subscribers:sub.unregister()
        self.listener.unregister()

    def on_map(self,message):
        try:
            payload=encode_map(message)
            if payload['frame']!='map':raise ValueError('지도 frame이 map이 아닙니다')
            with self.lock:
                if self.grid is None or self.grid['revision']!=payload['revision']:
                    self.paths.clear();self.goal=None
                self.grid=payload;self.error=''
        except ValueError as error:
            with self.lock:self.error=str(error);self.grid=None;self.paths.clear();self.goal=None

    def on_floor(self,message):
        with self.lock:
            key=(message.floor_id,message.map_generation)
            if key!=self.floor_key or message.state!=FloorState.READY:
                self.paths.clear();self.goal=None
            self.floor_key=key
            self.floor_ready=message.state==FloorState.READY
            self.floor_received=time.monotonic()

    def on_goal(self,message):
        with self.lock:
            self.paths.clear();self.goal_id=message.goal_id.id
            self.ended=False
            self.goal=message.goal.target_pose

    def on_goal_pose(self,message):
        with self.lock:
            if not self.ended:self.goal=message

    def on_result(self,message):
        with self.lock:
            if self.goal_id is None or message.status.goal_id.id==self.goal_id:
                self.paths.clear();self.goal=None;self.ended=True

    def on_path(self,key,message):
        with self.lock:
            if not self.ended:self.paths[key]=(time.monotonic(),message)

    def image(self):
        with self.lock:return self.grid or dict(error=self.error)

    def transform(self,frame,stamp):
        if frame.lstrip('/')=='map':return (0.,0.,0.)
        if not frame:raise ValueError('빈 경로 frame')
        t=self.buffer.lookup_transform('map',frame,stamp,rospy.Duration(0)).transform
        return (t.translation.x,t.translation.y,heading(t.rotation))

    @staticmethod
    def point(p,transform):
        x,y,a=transform;c,s=math.cos(a),math.sin(a)
        result=[x+c*p.x-s*p.y,y+s*p.x+c*p.y]
        if not all(math.isfinite(v) for v in result):raise ValueError('invalid point')
        return result

    def snapshot(self):
        with self.lock:
            grid=self.grid;paths=dict(self.paths);goal=self.goal;ready=self.floor_ready and time.monotonic()-self.floor_received<=2
        data=dict(map={k:v for k,v in grid.items() if k!='image'} if grid else None,
                  global_path=[],local_path=[],robot=None,goal=None,notes=[],floor_ready=ready)
        if not grid:
            data['notes'].append(self.error);return data
        if not ready:
            data['notes'].append('층 위치 확인 중 · 배경은 마지막 수신 지도 · 경로 표시 대기');return data
        errors=(tf2_ros.TransformException,ValueError)
        for key,(received,path) in paths.items():
            if key=='local' and time.monotonic()-received>3:continue
            try:
                frame=path.header.frame_id.lstrip('/')
                transform=self.transform(frame,path.header.stamp)
                # Preserve endpoints when reducing transport/render cost.
                step=max(1,math.ceil(len(path.poses)/1500))
                poses=path.poses[::step]
                if path.poses and (not poses or poses[-1] is not path.poses[-1]):poses=poses+[path.poses[-1]]
                if any(p.header.frame_id and p.header.frame_id.lstrip('/')!=frame for p in poses):raise ValueError('mixed path frames')
                data[key+'_path']=[self.point(p.pose.position,transform) for p in poses]
            except errors:data['notes'].append(key+' 경로 TF 확인 불가')
        if goal:
            try:data['goal']=self.point(goal.pose.position,self.transform(goal.header.frame_id,goal.header.stamp))
            except errors:data['notes'].append('목표 TF 확인 불가')
        try:
            t=self.buffer.lookup_transform('map','base_Link',rospy.Time(0),rospy.Duration(0))
            age=(rospy.Time.now()-t.header.stamp).to_sec()
            if not -.5<=age<=2:raise ValueError('stale robot transform')
            p=t.transform.translation
            if not all(math.isfinite(v) for v in (p.x,p.y)):raise ValueError('invalid robot position')
            data['robot']=[p.x,p.y,heading(t.transform.rotation)]
        except errors:data['notes'].append('로봇 위치 TF 수신 대기/지연')
        return data
