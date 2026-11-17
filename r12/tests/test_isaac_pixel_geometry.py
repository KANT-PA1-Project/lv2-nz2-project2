import ast,importlib.util,os,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np,cv2
BASE=Path(__file__).resolve().parents[1]
REPO=Path(os.environ.get('R12_PIXEL_TEST_REPO',str(BASE)))
sys.path[:0]=[str(REPO/'src'),str(REPO/'scripts'),str(REPO/'scripts/sensor_runtime')]
import arm_practice
arm_practice.__path__=[str(BASE/'src/arm_practice'),*list(arm_practice.__path__)]
from arm_practice.isaac_pixel_geometry import intrinsics,camera_matrix,projection_matrix,metadata
from camera_rig import transforms

def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
cloud=module('candidate_cloud',BASE/'scripts/sensor_runtime/colored_cloud.py')
guard=module('candidate_guard',BASE/'scripts/sensor_runtime/wrist_surface_guard.py')

class PixelGeometryTests(unittest.TestCase):
 def test_default_is_exact_legacy_cloud(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):
   rgb=np.zeros((480,640,3),np.uint8);rgb[:,:,0]=42;d=np.full((480,640),2.,dtype=np.float32);T=np.eye(4);p,col=cloud.project(rgb,d,T,step=13)
   v,u=np.mgrid[0:480:13,0:640:13];old=np.stack([(u-320)*2/400,(v-240)*2/400,np.full_like(u,2)],axis=-1).reshape(-1,3).astype('<f4')
   np.testing.assert_array_equal(p,old);self.assertTrue(np.all(col==42<<16));self.assertEqual(intrinsics(),(400.,400.,320.,240.))
 @unittest.skipUnless(os.getenv('R12_PIXEL_PLANE_NPZ'), 'Recorded Isaac plane fixture not configured')
 def test_recorded_independent_plane(self):
  d=np.load(Path(os.environ['R12_PIXEL_PLANE_NPZ']))['depth'];rgb=np.zeros((*d.shape,3),np.uint8)
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):p,_=cloud.project(rgb,d,np.eye(4),step=1)
  residual=p@np.array([.3,.2,1.])-2
  self.assertLess(float(abs(residual).max()),1.5e-6)
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'0'}):legacy,_=cloud.project(rgb,d,np.eye(4),step=1)
  self.assertGreater(float(np.median(abs(legacy@np.array([.3,.2,1.])-2))),.0005)
 def test_cv2_reprojection_and_actual_fk(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   rgb=np.zeros((480,640,3),np.uint8);d=np.full((480,640),np.nan);pixels=[(0,0),(319,239),(320,240),(639,479)]
   for u,v in pixels:d[v,u]=2.
   T=transforms(REPO,{f'arm_joint{i+1}':q for i,q in enumerate([0,-.33,-.175,-.33,.3])})['arm_camera_left_optical'];base,_=cloud.project(rgb,d,T,step=1);optical=(base-T[:3,3])@T[:3,:3];uv,_=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),camera_matrix(),np.zeros(5));np.testing.assert_allclose(uv[:,0,:],pixels,atol=1e-4)
   np.testing.assert_allclose(optical[:,2],2.,atol=1e-6)
 def test_optical_axis_z_not_euclidean_range(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   rgb=np.zeros((480,640,3),np.uint8);d=np.full((480,640),np.nan);d[479,639]=2.;p,_=cloud.project(rgb,d,np.eye(4),step=1)
   np.testing.assert_allclose(p[0],[1.5975,1.1975,2.],atol=1e-7);self.assertGreater(np.linalg.norm(p[0]),2.)
 def test_optical_axes_under_rigid_transform(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   rgb=np.zeros((480,640,3),np.uint8);d=np.full((480,640),np.nan);d[240,320]=1.;T=np.array([[0,0,1,1],[-1,0,0,2],[0,-1,0,3],[0,0,0,1.]])
   p,_=cloud.project(rgb,d,T,step=1);np.testing.assert_allclose(p[0],[2.,1.99875,2.99875],atol=2e-7)
 def test_ros_millimeter_transport_function(self):
  tree=ast.parse((BASE/'scripts/r12_sensor_ros.py').read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='depth_to_u16');scope={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual_depth_to_u16>','exec'),scope)
  raw=np.array([1.,1.23456,.00099,0,-1,np.nan,np.inf,10]);converted=scope['depth_to_u16'](raw);np.testing.assert_array_equal(converted,[1000,1234,0,0,0,0,0,0]);self.assertEqual(converted.dtype,np.uint16)
 def test_recorded_plane_depth_quantization(self):
  if not os.getenv('R12_PIXEL_PLANE_NPZ'):self.skipTest('Recorded Isaac plane fixture not configured')
  tree=ast.parse((BASE/'scripts/r12_sensor_ros.py').read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='depth_to_u16');scope={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual_depth_to_u16>','exec'),scope)
  d=np.load(Path(os.environ['R12_PIXEL_PLANE_NPZ']))['depth'];valid=np.isfinite(d)&(d>0)&(d<10);restored=scope['depth_to_u16'](d).astype(float)/1000;err=d[valid]-restored[valid];self.assertTrue(np.all(err>=-1e-7));self.assertLess(float(err.max()),.0010001)
 def test_ros_camera_info_and_worker_use_same_K_P(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   K=camera_matrix();P=projection_matrix();right=projection_matrix(right=True);np.testing.assert_array_equal(K,P[:,:3]);np.testing.assert_array_equal(K,right[:,:3]);self.assertEqual(K[0,2],319.5);self.assertEqual(K[1,2],239.5);self.assertEqual(right[0,3],-20.)
  ros=(BASE/'scripts/r12_sensor_ros.py').read_text();worker=(BASE/'scripts/r12_perception_worker.py').read_text();self.assertIn('info.k=camera_matrix()',ros);self.assertIn('ci.k=camera_matrix()',ros);self.assertIn('K=camera_matrix()',worker)
 def test_stereo_disparity_remains_fifty_mm(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   X=np.array([.3,-.2,2.,1.]);a=projection_matrix()@X;b=projection_matrix(right=True)@X;self.assertAlmostEqual(a[0]/a[2]-b[0]/b[2],10.)
 def test_guard_rays_match_index_intrinsics(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   g=guard.WristSurfaceGuard();u,v=639,479;ray=np.linalg.inv(camera_matrix())@np.array([u,v,1.]);np.testing.assert_allclose(g.rays[v,u],ray,atol=1e-12);self.assertIsNone(g.points);self.assertFalse(g.audit['unknown_volume_clearance_verified'])
 def test_bad_optin_is_error(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'yes'}):
   with self.assertRaises(ValueError):intrinsics()
 def test_metadata_keeps_limitations(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   m=metadata();self.assertFalse(m['hardware_calibrated']);self.assertFalse(m['uncertainty_certified']);self.assertEqual(m['native_depth_unit'],'m');self.assertEqual(m['ros_depth_unit'],'mm')
 def test_input_data_not_changed(self):
  with patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'}):
   rgb=np.zeros((480,640,3),np.uint8);d=np.full((480,640),1.,dtype=np.float32);before=d.copy();cloud.project(rgb,d,np.eye(4));np.testing.assert_array_equal(d,before)

if __name__=='__main__':unittest.main(verbosity=2)
