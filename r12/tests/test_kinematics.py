import tempfile
import unittest
from pathlib import Path
import numpy as np
from arm_practice.kinematics import forward_position


class ForwardKinematics(unittest.TestCase):
    def test_rotation_and_fixed_tcp_offset(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'arm.urdf'
            path.write_text('''<robot name="test"><link name="base"/><link name="arm"/><link name="tcp"/>
            <joint name="j" type="revolute"><parent link="base"/><child link="arm"/>
            <origin xyz="0 0 1"/><axis xyz="0 0 1"/></joint>
            <joint name="tip" type="fixed"><parent link="arm"/><child link="tcp"/>
            <origin xyz="1 0 0"/></joint></robot>''')
            np.testing.assert_allclose(forward_position(path, 'tcp', {'j': np.pi/2}), [0,1,1], atol=1e-8)
            with self.assertRaises(ValueError):
                forward_position(path, 'absent', {'j': 0})

    def test_prismatic(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'slide.urdf'
            path.write_text('''<robot name="test"><link name="base"/><link name="tcp"/>
            <joint name="j" type="prismatic"><parent link="base"/><child link="tcp"/>
            <axis xyz="0 0 1"/></joint></robot>''')
            np.testing.assert_allclose(forward_position(path, 'tcp', {'j': .2}), [0,0,.2])
