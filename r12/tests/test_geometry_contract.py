import ast,copy,importlib.util,json,os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
BASE=Path(__file__).resolve().parents[1]
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
g=module('geometry_candidate',BASE/'src/arm_practice/isaac_pixel_geometry.py')
shared=module('shared_reference',BASE/'src/arm_practice/shared_frame_prototype.py')
class Contract(unittest.TestCase):
 def setUp(self):self.env=patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'});self.env.start();self.addCleanup(self.env.stop)
 def test_modern_accept(self):self.assertEqual(g.require_archive_geometry(g.archive_geometry_fields()),g.metadata())
 def test_modern_missing_reject(self):
  with self.assertRaisesRegex(ValueError,'missing'):g.require_archive_geometry({})
 def test_legacy_missing_preserved(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):self.assertIsNone(g.require_archive_geometry({}))
 def test_modern_to_legacy_reject(self):
  d=g.archive_geometry_fields()
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):
   with self.assertRaisesRegex(ValueError,'mismatch'):g.require_archive_geometry(d)
 def test_legacy_to_modern_reject(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):d=g.archive_geometry_fields()
  with self.assertRaisesRegex(ValueError,'mismatch'):g.require_archive_geometry(d)
 def test_explicit_legacy_accept(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):self.assertEqual(g.require_archive_geometry(g.archive_geometry_fields())['intrinsics'],[400.,400.,320.,240.])
 def test_each_contract_field_fails_closed(self):
  for key in g.metadata():
   for mode in ('wrong','missing'):
    with self.subTest(key=key,mode=mode):
     d=copy.deepcopy(g.metadata())
     if mode=='wrong':d[key]='invalid'
     else:del d[key]
     with self.assertRaisesRegex(ValueError,'mismatch'):g.require_archive_geometry({'pixel_geometry_json':np.array(json.dumps(d))})
 def test_invalid_json_shape_and_dtype(self):
  for raw in [np.array('{}bad'),np.array('[]'),np.array(['{}']),np.array(3),np.array({'x':1},dtype=object)]:
   with self.subTest(raw=repr(raw)):
    with self.assertRaises(ValueError):g.require_archive_geometry({'pixel_geometry_json':raw})
 def test_nan_and_bool_intrinsics_reject(self):
  for bad in [float('nan'),True]:
   d=g.metadata();d['intrinsics'][0]=bad
   with self.assertRaises(ValueError):g.require_archive_geometry({'pixel_geometry_json':np.array(json.dumps(d))})
 def test_npz_roundtrip_no_pickle_and_pixels_unchanged(self):
  rgb=np.arange(72,dtype=np.uint8).reshape(4,6,3);depth=np.ones((4,6),np.float32);d=dict(rgb=rgb,depth=depth,**g.archive_geometry_fields())
  with tempfile.TemporaryDirectory() as t:
   path=Path(t)/'capture.npz';np.savez(path,**d)
   with np.load(path,allow_pickle=False) as loaded:
    g.require_archive_geometry(loaded);np.testing.assert_array_equal(loaded['rgb'],rgb);np.testing.assert_array_equal(loaded['depth'],depth)
 def test_shared_frame_readonly_copy_and_contract(self):
  d=dict(rgb=np.zeros((4,6,3),np.uint8),**g.archive_geometry_fields())
  with shared.SharedFrameSlot(capacity=16384) as slot:
   descriptor=slot.publish(d);copied,ack=shared.snapshot(descriptor);g.require_archive_geometry(copied);self.assertFalse(copied['pixel_geometry_json'].flags.writeable);slot.acknowledge(ack)
 def test_guard_clears_previous_points_on_contract_reject(self):
  source=ast.parse((BASE/'scripts/sensor_runtime/wrist_surface_guard.py').read_text());cls=next(n for n in source.body if isinstance(n,ast.ClassDef));method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update');scope={'np':np,'require_archive_geometry':g.require_archive_geometry};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'guard_update','exec'),scope)
  class Stub:pass
  x=Stub();x.points=np.zeros((10,3));x.capture_ns=123;scope['update'](x,{},{});self.assertIsNone(x.points);self.assertIsNone(x.capture_ns);self.assertNotEqual(x.audit['reason'],'depth_ready')
 def test_all_consumers_validate_before_geometry_work(self):
  worker=(BASE/'scripts/r12_perception_worker.py').read_text();self.assertLess(worker.index('    require_archive_geometry(data)'),worker.index('    rgb=data'))
  ros=(BASE/'scripts/r12_sensor_ros.py').read_text();self.assertLess(ros.index('  require_archive_geometry(data)'),ros.index('  for i,side in enumerate(images)'))
  sim=(BASE/'scripts/r12_sensor_sim.py').read_text();self.assertLess(sim.index(' frame_arrays.update(archive_geometry_fields())'),sim.index('shared_frame_slot.publish(frame_arrays)'))
if __name__=='__main__':unittest.main(verbosity=2)
