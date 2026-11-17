import tempfile,unittest
from pathlib import Path
import numpy as np
import torch
from arm_practice.swerve_rl import Policy,ResidualController

class ResidualInferenceTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();p=Path(self.directory.name)/'policy.pt'
        torch.save({'model':Policy().state_dict()},p);self.controller=ResidualController(p)
    def tearDown(self):self.directory.cleanup()
    def test_zero_command_has_no_residual(self):
        self.assertEqual(self.controller.scale([0,0,0],[0]*4,[0]*4,[0]*3,[0,0,9.81]),1.)
        self.assertEqual(self.controller.action,0.)
    def test_invalid_encoder_stops_wheels(self):
        self.assertEqual(self.controller.scale([.1,0,0],[0]*4,[np.nan]*4,[0]*3,[0,0,9.81]),0.)
    def test_scale_is_bounded_under_large_errors(self):
        for speed in [-10,0,10]:
            scale=self.controller.scale([.1,.1,.3],[0]*4,[speed]*4,[0,0,.3],[0,0,9.81])
            self.assertGreaterEqual(scale,.7);self.assertLessEqual(scale,1.3)
