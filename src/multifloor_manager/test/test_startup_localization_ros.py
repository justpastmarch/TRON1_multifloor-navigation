#!/usr/bin/env python3
"""Real AMCL + floor manager; synthetic stationary scan, no actuator nodes."""
import json
import math
from pathlib import Path
import threading
import time
import unittest

import numpy as np
from PIL import Image
import yaml
import rospy
import rospkg
import rostest
import tf2_ros
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, Trigger
from multifloor_manager.msg import FloorState
from test_startup_matching import raycast


class StartupTest(unittest.TestCase):
    def wait_for(self, predicate, timeout=20.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(.05)
        self.fail('timeout; localization status={!r}, floor={!r}'.format(self.status,self.floor))

    def setUp(self):
        root=Path(rospkg.RosPack().get_path('multifloor_manager'))/'config/maps'
        config=yaml.safe_load((root/'floor_3F.yaml').read_text())
        pixels=np.asarray(Image.open(root/config['image']))
        occupancy=(255-pixels)/255.
        self.grid=np.full(pixels.shape,-1)
        self.grid[occupancy<config['free_thresh']]=0
        self.grid[occupancy>config['occupied_thresh']]=100
        self.grid=np.flipud(self.grid)
        self.origin=config['origin']
        self.status={};self.floor=None;self.poses=[];self.amcl=None
        self.action_active=False;self.publish_scan=True
        self.scan_pub=rospy.Publisher('/scan',LaserScan,queue_size=2)
        self.odom_pub=rospy.Publisher('/tron/wheel_odom_raw',Odometry,queue_size=2)
        self.status_pub=rospy.Publisher('/mission/status',GoalStatusArray,queue_size=1)
        self.pose_pub=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        self.tf=tf2_ros.TransformBroadcaster()
        self.subs=[rospy.Subscriber('/multifloor/localization_status',String,self.on_status),
                   rospy.Subscriber('/multifloor/floor_state',FloorState,lambda m:setattr(self,'floor',m)),
                   rospy.Subscriber('/multifloor/amcl_initialpose',PoseWithCovarianceStamped,self.poses.append),
                   rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,lambda m:setattr(self,'amcl',m))]
        self.clear=rospy.Service('/move_base/clear_costmaps',Empty,lambda r:EmptyResponse())
        self.set_scan((4.,-4.,.5))
        self.timer=rospy.Timer(rospy.Duration(.1),self.sensors)

    def on_status(self,message):
        self.status=json.loads(message.data)

    def set_scan(self,pose):
        local=(pose[0]-self.origin[0],pose[1]-self.origin[1],pose[2])
        points=raycast(self.grid,local)
        self.assertEqual(len(points),360)
        scan=LaserScan();scan.header.frame_id='base_Link'
        scan.angle_min=-math.pi;scan.angle_increment=math.pi/180
        scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.range_min=.1;scan.range_max=15.
        scan.ranges=np.linalg.norm(points,axis=1).tolist()
        self.scan=scan

    def sensors(self,_event):
        now=rospy.Time.now()
        transform=TransformStamped();transform.header.stamp=now
        transform.header.frame_id='odom';transform.child_frame_id='base_Link'
        transform.transform.rotation.w=1.;self.tf.sendTransform(transform)
        odom=Odometry();odom.header.stamp=now;odom.header.frame_id='odom'
        odom.child_frame_id='base_Link';odom.pose.pose.orientation.w=1.
        self.odom_pub.publish(odom)
        status=GoalStatusArray();status.header.stamp=now
        if self.action_active:
            status.status_list=[GoalStatus(status=GoalStatus.ACTIVE)]
        self.status_pub.publish(status)
        if self.publish_scan:
            self.scan.header.stamp=now;self.scan_pub.publish(self.scan)

    def manual(self,pose):
        message=PoseWithCovarianceStamped();message.header.stamp=rospy.Time.now()
        message.header.frame_id='map';message.pose.pose.position.x=pose[0];message.pose.pose.position.y=pose[1]
        message.pose.pose.orientation.z=math.sin(pose[2]/2);message.pose.pose.orientation.w=math.cos(pose[2]/2)
        message.pose.covariance[0]=message.pose.covariance[7]=.04;message.pose.covariance[35]=.03
        self.pose_pub.publish(message)

    def test_auto_manual_busy_and_retry(self):
        self.wait_for(lambda:self.status.get('state')=='READY')
        self.assertEqual(self.status['source'],'auto')
        self.assertLess(math.hypot(self.status['x']-4.,self.status['y']+4.),.20)
        self.assertEqual(self.floor.state,FloorState.READY)
        # User input replaces the pose while idle, at a new arbitrary position.
        prior=len(self.poses);self.set_scan((2.599130,-1.983942,1.455898))
        self.manual((2.599130,-1.983942,1.455898))
        self.wait_for(lambda:len(self.poses)>prior and self.status.get('state')=='READY' and self.status.get('source')=='manual')
        self.assertLess(math.hypot(self.status['x']-2.599130,self.status['y']+1.983942),.2)
        # A late automatic result must not overwrite the user; active mission rejects reset.
        self.action_active=True;time.sleep(.3);prior=len(self.poses)
        self.manual((4.,-4.,.5))
        self.wait_for(lambda:self.status.get('state')=='INPUT_REJECTED')
        self.assertEqual(len(self.poses),prior)
        self.action_active=False;time.sleep(.3)
        # Retrying auto then immediately setting a manual pose supersedes the search.
        rospy.wait_for_service('/multifloor/localize_auto',timeout=3)
        self.assertTrue(rospy.ServiceProxy('/multifloor/localize_auto',Trigger)().success)
        self.manual((2.599130,-1.983942,1.455898))
        self.wait_for(lambda:len(self.poses)>prior and self.status.get('state')=='READY' and self.status.get('source')=='manual')
        time.sleep(1.5)
        self.assertEqual(self.status['source'],'manual')
        # Stale scans cannot inherit READY; fresh data lets the same queued input recover.
        self.publish_scan=False;time.sleep(.7)
        prior=len(self.poses);self.manual((2.599130,-1.983942,1.455898))
        self.wait_for(lambda:self.floor.state==FloorState.UNKNOWN)
        time.sleep(.7);self.assertEqual(len(self.poses),prior)
        self.publish_scan=True
        self.wait_for(lambda:len(self.poses)>prior and self.status.get('state')=='READY')
        # Malformed manual input must not reach AMCL or revoke a good pose.
        prior=len(self.poses);bad=PoseWithCovarianceStamped();bad.header.frame_id='map'
        self.pose_pub.publish(bad)
        self.wait_for(lambda:self.status.get('state')=='INPUT_REJECTED')
        self.assertEqual(len(self.poses),prior)
        self.assertEqual(self.floor.state,FloorState.READY)
        # No reset path is allowed to publish velocity or a motion goal.
        import rosgraph
        publishers=rosgraph.Master(rospy.get_name()).getSystemState()[0]
        for topic,nodes in publishers:
            if topic in ('/cmd_vel','/navigation/cmd_vel','/move_base/goal','/stair_traversal/goal'):
                self.assertNotIn('/multifloor_manager',nodes)

    def tearDown(self):
        self.timer.shutdown();self.clear.shutdown()
        for subscriber in self.subs:subscriber.unregister()


if __name__=='__main__':
    rospy.init_node('startup_localization_test')
    rostest.rosrun('multifloor_manager','startup_localization',StartupTest)
