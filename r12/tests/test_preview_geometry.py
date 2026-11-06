import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from preview_geometry import screen,advance
from arm_practice.transition_preview import preview
from arm_practice.swerve import RADIUS

class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.foot=[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
        self.c=preview(np.tile([0.,.1],(20,1)),0.,[0,0,np.pi/2,-np.pi/2],np.array([-1,-1,1,1])*.1/RADIUS)
    def test_clear_forward_progress(self):
        r=screen(self.c,[1,1],0,[1,2],self.foot,np.zeros((60,60),bool),.05)
        self.assertIsNotNone(r['selected']);self.assertGreater(r['selected']['ideal_rolling_goal_progress_m'],.09)
    def test_fully_blocked_has_no_selection(self):
        r=screen(self.c,[1,1],0,[1,2],self.foot,np.ones((60,60),bool),.05)
        self.assertIsNone(r['selected']);self.assertEqual(r['admissible_count'],0)
    def test_wall_crossing(self):
        grid=np.zeros((60,60),bool);grid[23,:]=True
        r=screen(self.c,[1,1],0,[1,2],self.foot,grid,.05)
        self.assertIsNone(r['selected'])
    def test_backward_progress_rejected(self):
        r=screen(self.c,[1,1],0,[1,0],self.foot,np.zeros((60,60),bool),.05)
        self.assertIsNone(r['selected'])
    def test_exact_arc(self):
        xy,yaw=advance([0,0],0,[1,0,1],np.pi/2)
        np.testing.assert_allclose(xy,[1,1],atol=1e-12);self.assertAlmostEqual(yaw,np.pi/2)
