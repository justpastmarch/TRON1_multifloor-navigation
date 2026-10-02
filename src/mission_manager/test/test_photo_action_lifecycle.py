"""Existing action boundary with real photo orchestrator; child motion is mocked."""
import unittest,threading
from unittest.mock import Mock,patch
from types import SimpleNamespace as NS
from mission_manager.mission_action_server import MissionActionServer
from mission_manager.mission_types import MissionRunRequest,MissionType
from mission_manager.photo_mission import PhotoMission

class PhotoActionLifecycle(unittest.TestCase):
 def setUp(self):
  from test_photo_mission import PhotoTourTest
  PhotoTourTest.setUp(self)
  self.server=MissionActionServer.__new__(MissionActionServer)
  self.server._orchestrator=self.host;self.server._lock=threading.RLock()
  self.server._cancel_requested=threading.Event();self.server._arrival_hold=Mock()
 def execute(self,kind):
  handle=Mock();handle.get_goal_id.return_value=NS(id='fixture-goal')
  self.server._active=handle;self.server._active_goal_id='fixture-goal'
  request=MissionRunRequest('fixture_'+kind.value,'roof_loop_se',kind,False)
  with patch('rospy.logerr'):self.server._execute(handle,request)
  return handle
 def test_tour_hold_approval_descent_return_hold_complete(self):
  h=self.execute(MissionType.PHOTO_TOUR);h.set_succeeded.assert_called_once()
  self.assertEqual(self.photo.snapshot()['stage'],'WAITING_RETURN')
  self.server._arrival_hold.arm.assert_called_with('roof_loop_se')
  self.assertFalse(any(e[2]=='stair_5f_rf_down' for e in self.executor.events))
  h=self.execute(MissionType.RETURN_TO_START);h.set_succeeded.assert_called_once()
  self.assertEqual(self.photo.snapshot()['stage'],'COMPLETE')
  self.assertTrue(any(e[2]=='stair_5f_rf_down' for e in self.executor.events))
 def test_failed_hold_does_not_close_return_and_is_retryable(self):
  self.execute(MissionType.PHOTO_TOUR)
  self.server._arrival_hold.arm.side_effect=RuntimeError('no fresh pose for hold')
  h=self.execute(MissionType.RETURN_TO_START);h.set_aborted.assert_called_once();h.set_succeeded.assert_not_called()
  self.assertEqual(self.photo.snapshot()['stage'],'FAILED');self.photo.preflight(True)
  restored=PhotoMission(self.settings,self.temp.name,self.executor)
  self.assertEqual(restored.snapshot()['stage'],'FAILED')
  self.server._arrival_hold.arm.side_effect=None
  h=self.execute(MissionType.RETURN_TO_START);h.set_succeeded.assert_called_once()
  self.assertEqual(self.photo.snapshot()['stage'],'COMPLETE')
 def test_failed_photo_hold_keeps_origin_and_return_approval(self):
  self.server._arrival_hold.arm.side_effect=RuntimeError('hold failed')
  h=self.execute(MissionType.PHOTO_TOUR);h.set_aborted.assert_called_once()
  self.assertEqual(self.photo.snapshot()['stage'],'FAILED');self.photo.preflight(True)
  self.assertFalse(any(e[2]=='stair_5f_rf_down' for e in self.executor.events))
 def test_restart_between_nav_arrival_and_hold_is_interrupted(self):
  self.execute(MissionType.PHOTO_TOUR);self.photo.save(stage='RETURN_ARRIVED')
  restored=PhotoMission(self.settings,self.temp.name,self.executor)
  self.assertEqual(restored.snapshot()['stage'],'INTERRUPTED')
 def test_cancel_during_final_hold_does_not_mark_complete(self):
  self.execute(MissionType.PHOTO_TOUR)
  self.server._arrival_hold.arm.side_effect=lambda *a,**kw:self.server._cancel_requested.set()
  h=self.execute(MissionType.RETURN_TO_START);h.set_succeeded.assert_not_called()
  self.assertEqual(self.photo.snapshot()['stage'],'FAILED')
