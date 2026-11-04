import json, tempfile, unittest
from pathlib import Path
import rospy,rosbag
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from mission_manager.mapping_store import MapLibrary,atomic_json,map_metadata
from mission_manager.mapping_compare import prepare_input
from PIL import Image
import yaml

class MappingTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name);self.lib=MapLibrary(self.root)
 def test_bad_name_and_path(self):
  for name in ['', ' '*4, 'x'*101]:
   with self.assertRaises(ValueError):self.lib.create(name)
  for identifier in ['.'*2+'/x','/tmp','a/b',None]:
   with self.assertRaises(ValueError):self.lib.path(identifier)
 def test_library_preserves_unicode_and_unique_ids(self):
  a,p=self.lib.create('새 장소 / 외부');b,q=self.lib.create('새 장소 / 외부')
  self.assertNotEqual(a,b);self.assertEqual(len(self.lib.list()),2);self.assertEqual(self.lib.list()[0]['name'],'새 장소 / 외부')
 def test_moved_result_preview_and_download(self):
  identifier,folder=self.lib.create('map');old=folder/'working';old.mkdir()
  Image.new('L',(4,3),205).save(old/'map.pgm')
  (old/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-2,3,0],negate=0,occupied_thresh=.65,free_thresh=.196)))
  atomic_json(old/'result.json',map_metadata(old));old.rename(folder/'saved')
  preview=self.lib.preview(identifier,'saved');self.assertEqual(preview['width'],4);self.assertEqual(preview['origin'],[-2,3,0])
  self.assertTrue(self.lib.file(identifier,'saved','map.pgm').startswith(b'P5'))
  with self.assertRaises(ValueError):self.lib.file(identifier,'saved','.'*2+'/session.json')
 def test_replay_filters_old_map_tf_and_preserves_latched_static(self):
  source=self.root/'source.bag';target=self.root/'filtered.bag'
  def transform(parent,child):
   tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child;tr.transform.rotation.w=1;return tr
  with rosbag.Bag(str(source),'w') as bag:
   bag.write('/tf_static',TFMessage([transform('base_Link','mid360_link')]),rospy.Time(10))
   bag.write('/tf',TFMessage([transform('map','odom'),transform('odom','base_Link')]),rospy.Time(10))
   for n in [10,11]:
    msg=LaserScan();msg.header.stamp=rospy.Time(n);bag.write('/scan',msg,rospy.Time(n))
   bag.write('/cmd_vel',TFMessage([]),rospy.Time(11))
  before=source.read_bytes();result=prepare_input(source,target)
  self.assertEqual(source.read_bytes(),before);self.assertEqual(result['scan_count'],2)
  with rosbag.Bag(str(target)) as bag:
   messages=list(bag.read_messages());self.assertNotIn('/cmd_vel',[t for t,m,ts in messages])
   for t,m,ts in messages:
    if t=='/tf':self.assertEqual([tr.header.frame_id for tr in m.transforms],['odom'])
   static=[c for c in bag._get_connections() if c.topic=='/tf_static'][0]
   self.assertEqual(static.header['latching'],b'1')
 def test_no_scan_cannot_compare(self):
  source=self.root/'empty.bag'
  with rosbag.Bag(str(source),'w'):pass
  with self.assertRaises(ValueError):prepare_input(source,self.root/'output.bag')

if __name__=='__main__':unittest.main()
