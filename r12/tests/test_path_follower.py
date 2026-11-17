import sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from path_follower import follow

class PathFollowerTests(unittest.TestCase):
 def run_path(self,path,grid):
  return follow(path,[.6,.6],np.pi,path[-1],grid,.02,.9,{}, {})
 def test_open_path_moves_under_cap(self):
  v,reason,state=self.run_path([[.6,.6],[.6,1.0],[1.4,1.4]],np.ones((100,100)))
  self.assertGreater(np.linalg.norm(v[:2]),0);self.assertLessEqual(np.linalg.norm(v[:2]),.9)
 def test_thin_wall_prevents_corner_cut(self):
  grid=np.ones((100,100));grid[:51,40]=0
  _,_,state=self.run_path([[.6,.6],[.6,1.4],[1.4,1.4]],grid)
  self.assertEqual(state['path_index'],1)
 def test_blocked_far_target_selects_nearer(self):
  def guard(v,*args,**kwargs):return (np.zeros(3),'limited') if abs(v[0])>.1 else (v,None)
  with patch('path_follower.constrain_velocity',side_effect=guard):
   v,_,state=self.run_path([[.6,.6],[.6,1.4],[1.4,1.4]],np.ones((100,100)))
  self.assertEqual(state['path_index'],1);self.assertGreater(np.linalg.norm(v[:2]),0)
 def test_invalid_lidar_returns_zero(self):
  with patch('path_follower.constrain_velocity',return_value=(np.zeros(3),'invalid_sensor')):
   v,reason,_=self.run_path([[.6,.6],[.6,1.4]],np.ones((100,100)))
  np.testing.assert_array_equal(v,0);self.assertEqual(reason,'invalid_sensor')

class PolygonFollowerTests(unittest.TestCase):
 def test_polygon_uses_occupied_cells_and_allows_narrower_side_clearance(self):
  occupied=np.zeros((100,100),bool);occupied[:,40]=True
  footprint=[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
  v,_,_=follow([[.6,.6],[.6,1.2]],[.6,.6],np.pi,[.6,1.2],np.ones((100,100))*.2,.02,.9,{}, {},footprint,occupied)
  self.assertGreater(np.linalg.norm(v[:2]),0)
  occupied[:,34]=True
  v,reason,_=follow([[.6,.6],[.6,1.2]],[.6,.6],np.pi,[.6,1.2],np.ones((100,100)),.02,.9,{}, {},footprint,occupied)
  np.testing.assert_array_equal(v,0);self.assertEqual(reason,'no_visible_forward_target')

class SmoothCornerTests(unittest.TestCase):
 def test_smoothing_preserves_endpoints_and_reduces_bend(self):
  from path_smoothing import smooth_path
  p=np.array([[.5,.5],[.5,.7],[.5,.9],[.7,.9],[.9,.9]])
  q=np.asarray(smooth_path(p,np.zeros((100,100),bool),np.ones((100,100)),.02,[[-.03,-.03],[.03,-.03],[.03,.03],[-.03,.03]]))
  np.testing.assert_allclose(q[[0,-1]],p[[0,-1]])
  self.assertLess(np.square(np.diff(q,n=2,axis=0)).sum(),np.square(np.diff(p,n=2,axis=0)).sum())
 def test_obstacle_blocks_simplification(self):
  from path_smoothing import smooth_path
  p=np.array([[.5,.5],[.5,.9],[.9,.9]])
  grid=np.zeros((100,100),bool);grid[25:50,24:27]=True
  q=smooth_path(p,grid,np.ones((100,100)),.02,[[-.03,-.03],[.03,-.03],[.03,.03],[-.03,.03]])
  np.testing.assert_allclose(q,p)
 def test_nearly_blocked_far_target_does_not_win(self):
  def guard(v,*args,**kwargs):return (v*.001,'limited') if abs(v[0])>.1 else (v,None)
  with patch('path_follower.constrain_velocity',side_effect=guard):
   v,_,state=follow([[.6,.6],[.6,1.4],[1.4,1.4]],[.6,.6],np.pi,[1.4,1.4],np.ones((100,100)),.02,.4,{}, {})
  self.assertEqual(state['path_index'],1)
  self.assertGreater(np.linalg.norm(v[:2]),.1)

class EndpointFollowerTests(unittest.TestCase):
 def test_nearest_endpoint_still_approaches(self):
  with patch('path_follower.constrain_velocity',side_effect=lambda v,*a,**k:(v,None)):
   v,reason,state=follow([[.5,.6],[.6,.6]],[.57,.6],np.pi,[.6,.6],np.ones((100,100)),.02,.4,{}, {})
  self.assertGreater(np.linalg.norm(v[:2]),0)
  self.assertEqual(state['path_index'],1)
 def test_nearest_endpoint_keeps_obstacle_check(self):
  grid=np.ones((100,100));grid[30,29]=0
  v,reason,_=follow([[.5,.6],[.6,.6]],[.57,.6],np.pi,[.6,.6],grid,.02,.4,{}, {})
  np.testing.assert_array_equal(v,0)
  self.assertEqual(reason,'no_visible_forward_target')

class TransitionHeadingIntegrationTests(unittest.TestCase):
 def test_heading_receives_velocity_units_not_grid_spacing(self):
  with patch.dict('os.environ',{'R12_LOOKAHEAD_HEADING':'1','R12_TRANSITION_HEADING':'1'}),patch('arm_practice.heading_feasibility.choose_transition_heading',return_value=(np.pi,.2)) as choose,patch('path_follower.constrain_velocity',side_effect=lambda v,*a,**k:(v,None)):
   v,_,_=follow([[.6,.6],[.6,.62],[.6,1.4]],[.6,.6],np.pi,[.6,1.4],np.ones((100,100)),.02,.4,{}, {})
   self.assertGreater(np.linalg.norm(v[:2]),0)
   np.testing.assert_allclose(np.linalg.norm(choose.call_args.args[0],axis=1),.4)
 def test_transition_option_still_rejects_occupied_motion(self):
  grid=np.zeros((100,100),bool);grid[:,34]=True
  with patch.dict('os.environ',{'R12_LOOKAHEAD_HEADING':'1','R12_TRANSITION_HEADING':'1'}):
   v,_,_=follow([[.6,.6],[.6,1.2]],[.6,.6],np.pi,[.6,1.2],np.ones((100,100)),.02,.4,{}, {},[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]],grid)
   np.testing.assert_array_equal(v,0)
