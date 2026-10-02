import unittest,threading
from types import SimpleNamespace as N
from unittest.mock import Mock,patch
from pathlib import Path
from multifloor_manager.ros_floor_selection import FloorSelection,FloorState,MapGenerationState,MapGeneration
class TestFloorSelection(unittest.TestCase):
 def setUp(self):
  self.timer=patch("multifloor_manager.ros_floor_selection.rospy.Time.now",return_value=N(to_nsec=lambda:123456789));self.timer.start();self.addCleanup(self.timer.stop)
  self.params=patch("multifloor_manager.ros_floor_selection.rospy.get_param",side_effect=lambda _name,default:default);self.params.start();self.addCleanup(self.params.stop)
  self.proxy=patch("multifloor_manager.ros_floor_selection.rospy.ServiceProxy",return_value=Mock());self.proxy.start();self.addCleanup(self.proxy.stop)
 def fixture(self):
  runtime=N(condition=threading.Condition(),epoch=1,active_epoch=7,localization=object(),map_state=MapGenerationState(MapGeneration(4),None),publish_floor=Mock(),map_guard=1,tag_state=1,tag_context=1,tag_accepted=True,costmap_identity=1)
  startup=N(lock=threading.RLock(),busy=Mock(return_value=False),stationary=Mock(return_value=True),generation=3,request=None,pose=None,latest_map=None,publish=Mock(),wake=threading.Event())
  node=N(startup=startup,floors={'5F':N(map_yaml='5f.yaml')},identities={'5F':'identity5'},_active_lock=threading.RLock(),_active=False,runtime=runtime,config_root=Path('/maps'),services=N(call=Mock(return_value=N(result=0,map='grid5'))),change_map_name='/change_map',change_map=Mock())
  selection=FloorSelection.__new__(FloorSelection);selection.node=node
  return selection,node
 def test_selection_invalidates_previous_evidence_and_waits_for_localization(self):
  s,n=self.fixture()
  with patch('multifloor_manager.ros_floor_selection.fingerprint_occupancy_grid',return_value=N(identity=lambda:'identity5')):
   response=s.select(N(floor_id='5F'))
  self.assertTrue(response.accepted,response.reason);self.assertFalse(n._active)
  self.assertEqual(n.runtime.initial_floor,'5F');self.assertEqual(int(n.runtime.map_state.generation),5)
  self.assertIsNone(n.runtime.localization);self.assertIsNone(n.runtime.active_epoch)
  self.assertFalse(n.runtime.tag_accepted);self.assertIsNone(n.runtime.costmap_identity)
  self.assertEqual(n.startup.request,('auto',None));self.assertTrue(n.startup.wake.is_set())
  self.assertTrue(all(c.args[0]==FloorState.UNKNOWN for c in n.runtime.publish_floor.call_args_list))
 def test_busy_or_unknown_floor_never_changes_map(self):
  for reason in ('unknown','active','moving','busy'):
   s,n=self.fixture();target='5F'
   if reason=='unknown':target='2F'
   if reason=='active':n._active=True
   if reason=='moving':n.startup.stationary.return_value=False
   if reason=='busy':n.startup.busy.return_value=True
   self.assertFalse(s.select(N(floor_id=target)).accepted)
   n.services.call.assert_not_called()
 def test_prepare_exception_releases_busy_flag(self):
  s,n=self.fixture();n.identities={}
  self.assertFalse(s.select(N(floor_id='5F')).accepted)
  self.assertFalse(n._active)
 def test_superseded_load_cannot_publish_old_floor(self):
  s,n=self.fixture()
  def supersede(*args):
   n.startup.generation+=1
   raise RuntimeError('late service failure')
  n.services.call.side_effect=supersede
  self.assertFalse(s.select(N(floor_id='5F')).accepted)
  self.assertEqual(n.runtime.publish_floor.call_count,1)
 def test_wrong_map_or_failed_load_stays_unknown(self):
  for result in (0,1):
   s,n=self.fixture();n.services.call.return_value.result=result
   with patch('multifloor_manager.ros_floor_selection.fingerprint_occupancy_grid',return_value=N(identity=lambda:'WRONG')):
    self.assertFalse(s.select(N(floor_id='5F')).accepted)
   self.assertIsNone(n.startup.request);self.assertFalse(n._active)
   self.assertEqual(n.runtime.publish_floor.call_args.args[0],FloorState.UNKNOWN)
if __name__=='__main__':unittest.main()
