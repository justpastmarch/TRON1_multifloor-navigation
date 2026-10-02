import base64,math,struct,threading,time,unittest,zlib
from unittest.mock import Mock,patch
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from multifloor_manager.msg import FloorState
from move_base_msgs.msg import MoveBaseActionResult
from mission_manager.console_map import ConsoleMap,encode_map


def grid():
    m=OccupancyGrid();m.header.frame_id='map';m.info.width=2;m.info.height=2;m.info.resolution=.05
    m.info.origin.orientation.w=1;m.data=[0,100,-1,50];return m


class MapTest(unittest.TestCase):
    def view(self):
        v=ConsoleMap.__new__(ConsoleMap);v.lock=threading.RLock();v.grid=None;v.paths={};v.goal=None;v.goal_id=None
        v.floor_key=None;v.floor_ready=True;v.floor_received=time.monotonic();v.ended=False;v.error='';v.buffer=Mock()
        v.buffer.lookup_transform.side_effect=ValueError('no transform')
        return v
    def test_png_flips_ros_bottom_up_rows(self):
        p=encode_map(grid());raw=base64.b64decode(p['image'].split(',')[1]);pos=8;data=b''
        while pos<len(raw):
            n=struct.unpack('!I',raw[pos:pos+4])[0];kind=raw[pos+4:pos+8]
            if kind==b'IDAT':data+=raw[pos+8:pos+8+n]
            pos+=n+12
        self.assertEqual(zlib.decompress(data),bytes([0,150,122,0,245,0]))
    def test_invalid_grid_not_rendered(self):
        m=grid();m.data=[0]
        with self.assertRaises(ValueError):encode_map(m)
    def test_origin_change_changes_revision(self):
        m=grid();a=encode_map(m);m.info.origin.position.x=1
        self.assertNotEqual(a['revision'],encode_map(m)['revision'])
    def test_map_change_clears_old_paths(self):
        v=self.view();v.on_map(grid());v.paths['global']=('old',None)
        m=grid();m.info.origin.position.x=2;v.on_map(m);self.assertEqual(v.paths,{})
    def test_rotation_and_translation(self):
        p=PoseStamped().pose.position;p.x=1;p.y=0
        out=ConsoleMap.point(p,(2,3,math.pi/2))
        self.assertAlmostEqual(out[0],2);self.assertAlmostEqual(out[1],4)
    def test_missing_tf_hides_path_instead_of_treating_odom_as_map(self):
        v=self.view();v.on_map(grid());p=Path();p.header.frame_id='odom';p.poses=[PoseStamped()]
        v.on_path('local',p);s=v.snapshot()
        self.assertEqual(s['local_path'],[]);self.assertIn('local 경로 TF 확인 불가',s['notes'])
    def test_map_frame_path_and_no_tf_robot(self):
        v=self.view();v.on_map(grid());p=Path();p.header.frame_id='map';p.poses=[PoseStamped()]
        p.poses[0].pose.position.x=2;v.on_path('global',p);s=v.snapshot()
        self.assertEqual(s['global_path'],[[2,0]]);self.assertIsNone(s['robot'])
    def test_floor_transition_and_terminal_clear_overlays(self):
        v=self.view();v.on_map(grid());v.paths['global']=(0,None)
        v.on_floor(FloorState(floor_id='4F',map_generation=2,state=FloorState.TRANSITIONING))
        self.assertEqual(v.paths,{});self.assertFalse(v.snapshot()['floor_ready'])
        v.on_result(MoveBaseActionResult());v.on_path('global',Path());self.assertEqual(v.paths,{})

if __name__=='__main__':unittest.main()
