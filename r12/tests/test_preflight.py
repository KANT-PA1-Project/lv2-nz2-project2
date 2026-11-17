import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from arm_practice.preflight import validate_robot

ROOT = Path(__file__).resolve().parents[1]


class RobotPreflight(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'robot.json'
        self.config = json.loads((ROOT / 'configs/demo_robot.json').read_text())
        self.config['usd_path'] = str(Path(self.temp.name) / 'missing.usd')

    def check(self, require_usd=False):
        self.path.write_text(json.dumps(self.config))
        return validate_robot(ROOT / 'assets/demo_arm.urdf', self.path, require_usd)

    def test_demo_and_missing_usd_modes(self):
        self.assertTrue(self.check()['valid'])
        self.assertFalse(self.check(True)['valid'])

    def test_unknown_joint_and_tcp(self):
        self.config['joint_names'][0] = 'unknown'
        self.config['tcp_link'] = 'missing'
        report = self.check()
        self.assertFalse(report['valid'])
        self.assertEqual(len(report['errors']), 2)

    def test_missing_actuator(self):
        for key in ('joint_names', 'lower', 'upper', 'home', 'action_scale',
                    'effort', 'velocity', 'stiffness', 'damping'):
            self.config[key].pop()
        self.assertFalse(self.check()['valid'])

    def test_excessive_limits(self):
        for key, value in [('lower', -100), ('upper', 100),
                           ('effort', 10000), ('velocity', 10000)]:
            with self.subTest(key=key):
                old = self.config[key][0]
                self.config[key][0] = value
                self.assertFalse(self.check()['valid'])
                self.config[key][0] = old

    def test_conservative_limits_and_reordered_actions(self):
        self.config['lower'][0] = -1
        self.config['upper'][0] = 1
        self.config['effort'][0] = 1
        for key in ('joint_names', 'lower', 'upper', 'home', 'action_scale',
                    'effort', 'velocity', 'stiffness', 'damping'):
            self.config[key].reverse()
        report = self.check()
        self.assertTrue(report['valid'])
        self.assertEqual(report['joint_order'], ['joint3', 'joint2', 'joint1'])

    def test_cli_failure_writes_machine_readable_report(self):
        self.config['tcp_link'] = 'missing'
        self.check()
        output = Path(self.temp.name) / 'report.json'
        result = subprocess.run([sys.executable, '-m', 'arm_practice.preflight',
                                 str(ROOT / 'assets/demo_arm.urdf'), str(self.path),
                                 '--output', str(output)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(json.loads(result.stdout), json.loads(output.read_text()))
