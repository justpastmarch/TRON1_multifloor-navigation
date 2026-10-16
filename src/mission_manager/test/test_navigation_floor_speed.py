import unittest
from unittest.mock import patch
from mission_manager.ros_navigation_progress import configure_floor_speed

class FloorSpeedTest(unittest.TestCase):
    def test_rf_scaled_other_floors_restored_without_compounding(self):
        with patch('rospy.get_param',side_effect=lambda key,default:default), patch('rospy.ServiceProxy'), patch('multifloor_manager.ros_services.BoundedServiceCaller.call') as call:
            for floor in ['RF','RF','5F','3F']:
                configure_floor_speed(floor)
            requests=[c.args[2].config.doubles for c in call.call_args_list]
            self.assertEqual([len(x) for x in requests],[1]*4)
            self.assertTrue(all(x[0].name=='max_vel_x' for x in requests))
            for got,want in zip(requests,[.21,.21,.30,.30]):self.assertAlmostEqual(got[0].value,want)
