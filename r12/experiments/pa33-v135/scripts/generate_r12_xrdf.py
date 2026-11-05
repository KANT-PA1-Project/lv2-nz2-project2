#!/usr/bin/env python3
"""Conservative XRDF draft from the supplied URDF collision primitives.

Spheres enclose each primitive. They require planning/collision review and do
not certify the detailed visual meshes or physical hardware.
"""
import json
import itertools
import numpy as np
from arm_practice.kinematics import rotation
import math
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
robot = ET.parse(root/'models/r12/r12_arm.urdf').getroot()
cfg = json.loads((root/'configs/r12_robot.json').read_text())
spheres = {}
for link in robot.findall('link'):
    entries = []
    for collision_index, collision in enumerate(link.findall('collision')):
        origin = collision.find('origin')
        xyz = [float(x) for x in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
        geom = collision.find('geometry')
        if geom.find('box') is not None:
            size = [float(x) for x in geom.find('box').get('size').split()]
            # Enclose a grid of small cuboids instead of one huge sphere around
            # long masts/arm links. Preserve conservative coverage of every box.
            refine = link.get('name') in ('arm_link2','arm_link3') or (link.get('name') == 'base_link' and collision_index in (264,312,314,340,361))
            cell_size = .02 if link.get('name') in ('arm_link2','arm_link3') else (.04 if refine else 100.)
            counts = [max(1, math.ceil(x / cell_size)) for x in size]
            cell = np.array(size) / counts
            rpy = [float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            rot = rotation([0,0,1],rpy[2]) @ rotation([0,1,0],rpy[1]) @ rotation([1,0,0],rpy[0])
            radius = float(np.linalg.norm(cell)/2)
            for index in itertools.product(*(range(n) for n in counts)):
                local = -np.array(size)/2 + (np.array(index)+.5)*cell
                center = np.array(xyz) + rot @ local
                entries.append({'center': center.tolist(), 'radius': radius})
            continue
        elif geom.find('cylinder') is not None:
            c = geom.find('cylinder')
            radius = math.hypot(float(c.get('radius')),float(c.get('length'))/2)
        elif geom.find('sphere') is not None:
            radius = float(geom.find('sphere').get('radius'))
        else:
            raise ValueError('Unsupported collision geometry')
        entries.append({'center': xyz, 'radius': radius})
    if entries:
        spheres[link.get('name')] = entries
# Exclude only adjacent connected links and links in the same fixed body.
fixed = {name: name for name in spheres}
def leader(x):
    while fixed.get(x,x) != x:
        x = fixed[x]
    return x
for j in robot.findall('joint'):
    a,b = j.find('parent').get('link'),j.find('child').get('link')
    if j.get('type') == 'fixed' and a in fixed and b in fixed:
        fixed[leader(b)] = leader(a)
ignore = {name: [] for name in spheres}
for a in spheres:
    for b in spheres:
        if a < b and leader(a)==leader(b):
            ignore[a].append(b)
for j in robot.findall('joint'):
    a,b = j.find('parent').get('link'),j.find('child').get('link')
    if a in ignore and b in spheres and b not in ignore[a]:
        ignore[a].append(b)
xrdf = {'format':'xrdf','format_version':1.0, 'modifiers':[{'set_base_frame':'base_link'}],
        'default_joint_positions':dict(zip(cfg['joint_names'],cfg['home'])),
        'cspace':{'joint_names':cfg['joint_names'],'acceleration_limits':[0.5]*5,'jerk_limits':[5.0]*5},
        'tool_frames':['arm_link5'],'collision':{'geometry':'r12_collision'},
        'self_collision':{'geometry':'r12_collision','ignore':{k:v for k,v in ignore.items() if v}},
        'geometry':{'r12_collision':{'spheres':spheres}}}
assert sum(map(len,spheres.values())) <= 1024, 'cuMotion sphere budget exceeded'
# JSON is a YAML subset and can be consumed by XRDF's YAML loader.
output=root/'ros2/r12_arm_moveit_config/config/r12.xrdf'
output.write_text(json.dumps(xrdf,indent=2)+'\n')
print('Collision sphere count:',sum(map(len,spheres.values())))
