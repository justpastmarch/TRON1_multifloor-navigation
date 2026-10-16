"""Replay unmodified BAG odometry into the actual admission callback; no ROS I/O."""
import json
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from mission_manager.ros_state import RosStateMonitor
from mission_manager.stair_entry_gate import entry_motion_window, LIDAR_ENTRY_POLICY


class RecordedEntryClockTest(unittest.TestCase):
    def test_stationary_recording_survives_measured_cross_pc_clock_skew(self):
        data=json.loads((Path(__file__).parent/'entry_odom_clock_skew.json').read_text())
        for rows in data['windows']:
            state=RosStateMonitor.__new__(RosStateMonitor)
            state._condition=threading.Condition()
            state._lidar_handoff=True
            state._entry_motion=deque(maxlen=256)
            state._entry_motion_stamp=None
            self.assertTrue(any(row[1]<0 for row in rows))
            for now,age,v,w in rows:
                message=NS(header=NS(stamp=NS(to_sec=lambda:now-age)),
                           twist=NS(twist=NS(linear=NS(x=v),angular=NS(z=w))))
                with patch('mission_manager.ros_state.time.monotonic',return_value=now+1000), \
                     patch('mission_manager.ros_state.rospy.Time.now',return_value=NS(to_sec=lambda:now)):
                    state._accept_odom(message)
            motion=entry_motion_window(state._entry_motion,rows[-1][0]+1000)
            self.assertIsNotNone(motion)
            self.assertLess(motion[0],LIDAR_ENTRY_POLICY.max_linear_speed)
            self.assertLess(motion[1],LIDAR_ENTRY_POLICY.max_angular_speed)

    def test_small_skew_is_not_extrapolated_into_extra_stationary_duration(self):
        rows=[(100+i*.01,.01,.02) for i in range(41)]
        self.assertIsNotNone(entry_motion_window(rows,100.395))
        self.assertIsNone(entry_motion_window(rows,100.37))
        self.assertIsNone(entry_motion_window(rows[-10:],100.395))
