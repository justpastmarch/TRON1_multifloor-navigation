import unittest
from unittest.mock import patch
from mission_manager.ros_navigation_progress import configure_floor_speed

class FloorSpeedTest(unittest.TestCase):
    def test_rf_scaled_other_floors_restored_without_compounding(self):
        with patch('rospy.get_param',side_effect=lambda key,default:default), patch('rospy.ServiceProxy'), patch('multifloor_manager.ros_services.BoundedServiceCaller.call') as call:
            for floor in ['RF','RF','5F','3F']:
                configure_floor_speed(floor)
            requests=[c.args[2].config.doubles for c in call.call_args_list]
            self.assertEqual([len(x) for x in requests],[1]*8)
            self.assertTrue(all(x[0].name=='max_vel_x' for x in requests))
            for got,want in zip(requests,[.21,.21,.21,.21,.30,.30,.30,.30]):self.assertAlmostEqual(got[0].value,want)

    def test_receiver_and_planner_receive_same_floor_limit_before_goal(self):
        with patch('rospy.get_param',side_effect=lambda key,default:default), patch('rospy.ServiceProxy'), patch('multifloor_manager.ros_services.BoundedServiceCaller.call') as call:
            configure_floor_speed('RF')
            self.assertEqual([c.args[0] for c in call.call_args_list], [
                '/move_base/TrajectoryPlannerROS/set_parameters', '/stair_supervisor/set_navigation_limits'])
            planner,receiver = [c.args[2] for c in call.call_args_list]
            self.assertIsNot(planner,receiver)
            self.assertEqual(planner.config.strs,[])
            self.assertEqual(receiver.config.strs[0].name,'nav_limit_revision')
            self.assertGreater(int(receiver.config.strs[0].value),0)

    def test_receiver_failure_is_reported_before_navigation_can_be_started(self):
        from mission_manager.navigation_executor import NavigationLifecycleError
        with patch('rospy.get_param',side_effect=lambda key,default:default), patch('rospy.ServiceProxy'), patch('multifloor_manager.ros_services.BoundedServiceCaller.call',side_effect=[None,RuntimeError('receiver unavailable')]):
            with self.assertRaisesRegex(NavigationLifecycleError,'receiver unavailable'):
                configure_floor_speed('RF')

    def test_receiver_revisions_increase_without_leaking_custom_fields_to_planner(self):
        with patch('rospy.get_param',side_effect=lambda key,default:default), patch('rospy.ServiceProxy'), patch('multifloor_manager.ros_services.BoundedServiceCaller.call') as call, patch('mission_manager.ros_navigation_progress.time.time_ns',return_value=1):
            configure_floor_speed('5F');configure_floor_speed('RF')
            planner=[c.args[2] for c in call.call_args_list[::2]]
            receiver=[c.args[2] for c in call.call_args_list[1::2]]
            self.assertTrue(all(not request.config.strs for request in planner))
            revisions=[int(request.config.strs[0].value) for request in receiver]
            self.assertGreater(revisions[1],revisions[0])

    def test_delayed_earlier_planner_call_retains_its_earlier_revision(self):
        import threading
        entered,release=threading.Event(),threading.Event();requests=[];errors=[]
        def call(name,proxy,request):
            if name.startswith('/move_base') and request.config.doubles[0].value==.30:
                entered.set();release.wait(2)
            if name.startswith('/stair_supervisor'):
                requests.append((request.config.doubles[0].value,int(request.config.strs[0].value)))
        def old():
            try:configure_floor_speed('5F')
            except Exception as error:errors.append(error)
        with patch('rospy.get_param',side_effect=lambda key,default:default),patch('rospy.ServiceProxy'),patch('multifloor_manager.ros_services.BoundedServiceCaller.call',side_effect=call):
            worker=threading.Thread(target=old);worker.start()
            try:
                self.assertTrue(entered.wait(1));configure_floor_speed('RF')
            finally:release.set();worker.join(1)
        self.assertEqual(errors,[]);self.assertFalse(worker.is_alive())
        self.assertEqual([value for value,_ in requests],[.21,.30])
        self.assertLess(requests[1][1],requests[0][1])
