import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from swept_footprint import motion_clear

class SweptFootprintTests(unittest.TestCase):
 def setUp(self):
  self.grid=np.zeros((200,200),bool)
  self.hull=[[-.4,-.08],[.4,-.08],[.4,.08],[-.4,.08]]
 def clear(self,cmd,**kwargs):
  return motion_clear([1,1],0,cmd,self.hull,self.grid,.01,**kwargs)
 def test_rotation_corner_collision_despite_clear_translation(self):
  self.grid[110:112,137:140]=True
  self.assertTrue(self.clear([.03,0,0]))
  self.assertFalse(self.clear([.03,0,.2]))
 def test_pure_rotation_rejected(self):
  self.grid[110:112,137:140]=True
  self.assertFalse(self.clear([0,0,.2]))
 def test_open_combined_twist(self):
  self.assertTrue(self.clear([.2,.1,.2]))
 def test_outside_or_nonfinite_rejected(self):
  self.assertFalse(self.clear([4,0,0]))
  self.assertFalse(self.clear([0,0,float('nan')]))
 def test_intermediate_rotation_checked_not_only_endpoints(self):
  self.grid[125:128,125:128]=True
  self.assertTrue(self.clear([0,0,0]))
  self.assertFalse(self.clear([0,0,np.pi/2],horizon=1))
