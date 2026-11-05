#!/usr/bin/env python3
"""Create fixed-base arm variants of the supplied R12 assets, preserving originals."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from pxr import Usd, UsdPhysics
import numpy as np
from arm_practice.kinematics import forward_position

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/vendor/R12_NO_USB_URDF_USD'
OUTPUT = ROOT / 'models/r12'
OUTPUT.mkdir(parents=True, exist_ok=True)

# Freeze locomotion for this arm-only task in both representations.
tree = ET.parse(SOURCE / 'urdf/r12_no_usb.urdf')
arm = []
for joint in tree.getroot().findall('joint'):
    if joint.get('name').startswith('arm_joint'):
        arm.append(joint)
    elif joint.get('type') != 'fixed':
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit', 'dynamics'):
            for element in joint.findall(tag):
                joint.remove(element)
for mesh in tree.getroot().iter('mesh'):
    original = (SOURCE / 'urdf' / mesh.get('filename')).resolve()
    mesh.set('filename', os.path.relpath(original, OUTPUT))
tree.write(OUTPUT / 'r12_arm.urdf', encoding='utf-8', xml_declaration=True)

path = OUTPUT / 'r12_arm.usda'
if path.exists():
    path.unlink()
stage = Usd.Stage.CreateNew(str(path))
robot = stage.DefinePrim('/Robot', 'Xform')
robot.GetReferences().AddReference(os.path.relpath(SOURCE / 'usd/r12_no_usb.usdc', OUTPUT))
stage.SetDefaultPrim(robot)
stage.SetMetadata('metersPerUnit', 1.0)
stage.SetMetadata('upAxis', 'Z')
for prim in list(stage.Traverse()):
    if prim.IsA(UsdPhysics.RevoluteJoint) and not prim.GetName().startswith('arm_joint'):
        prim.SetTypeName('PhysicsFixedJoint')
        prim.RemoveAPI(UsdPhysics.DriveAPI, 'angular')
# A world-to-base fixed joint makes the supplied floating mobile robot stationary.
fixed = UsdPhysics.FixedJoint.Define(stage, '/Robot/Joints/world_fixed')
fixed.CreateBody1Rel().SetTargets(['/Robot/base_link'])
stage.GetRootLayer().Save()

config = {
 'usd_path': 'models/r12/r12_arm.usda',
 'joint_names': [j.get('name') for j in arm],
 'tcp_link': 'arm_link5', 'home': [0.0]*5,
 'lower': [float(j.find('limit').get('lower')) for j in arm],
 'upper': [float(j.find('limit').get('upper')) for j in arm],
 'action_scale': [0.5, 0.25, 0.25, 0.25, 0.25],
 'effort': [2.0]*5, 'velocity': [0.4]*5,
 'stiffness': [40.0]*5, 'damping': [4.0]*5,
 'target_range': {'x': [0.20,0.26], 'y': [-0.06,0.06], 'z': [0.48,0.55]},
 'success_distance_m': 0.015,
}
(ROOT / 'configs/r12_robot.json').write_text(json.dumps(config, indent=2)+'\n')
print(path)

# Seeded FK bank: goals come from joint values within the policy action range.
rng = np.random.default_rng(12)
qs = rng.uniform(-1, 1, size=(2048, 5)) * np.array(config['action_scale'])
goals = [forward_position(OUTPUT / 'r12_arm.urdf', 'arm_link5', dict(zip(config['joint_names'], q))).tolist() for q in qs]
(OUTPUT / 'reach_targets.json').write_text(json.dumps({'seed': 12, 'joint_names': config['joint_names'], 'joint_positions': qs.tolist(), 'positions': goals}))
config['target_positions_path'] = 'models/r12/reach_targets.json'
(ROOT / 'configs/r12_robot.json').write_text(json.dumps(config, indent=2)+'\n')
