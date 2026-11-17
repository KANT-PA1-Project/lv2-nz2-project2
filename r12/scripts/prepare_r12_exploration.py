#!/usr/bin/env python3
"""Create a simulation-only +/-180 degree exploration profile, preserving originals."""
import copy
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
from pxr import Usd,UsdPhysics
root=Path(__file__).resolve().parents[1]
for src,dst in [('models/r12/r12_arm.urdf','models/r12/r12_exploration.urdf'),('ros2/r12_arm_moveit_config/config/r12.urdf','ros2/r12_arm_moveit_config/config/r12_exploration.urdf')]:
    tree=ET.parse(root/src)
    for j in tree.getroot().findall('joint'):
        if j.get('name','').startswith('arm_joint'):
            j.find('limit').set('lower',str(-math.pi));j.find('limit').set('upper',str(math.pi))
    # Detailed source triangles for planning; fixed-body and adjacent contacts
    # still follow the SRDF exclusion semantics.
    for link in tree.getroot().findall('link'):
        if link.get('name','').startswith('arm_link'):
            for c in list(link.findall('collision')):link.remove(c)
            for visual in link.findall('visual'):
                c=ET.SubElement(link,'collision')
                for tag in ['origin','geometry']:
                    if visual.find(tag) is not None:c.append(copy.deepcopy(visual.find(tag)))
    tree.write(root/dst,encoding='utf-8',xml_declaration=True)
out=root/'models/r12/r12_exploration.usda'
if out.exists():out.unlink()
s=Usd.Stage.CreateNew(str(out));robot=s.DefinePrim('/Robot','Xform');robot.GetReferences().AddReference('r12_arm.usda');s.SetDefaultPrim(robot)
s.SetMetadata('metersPerUnit',1.0);s.SetMetadata('upAxis','Z')
for p in s.Traverse():
    if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName().startswith('arm_joint'):
        j=UsdPhysics.RevoluteJoint(p);j.GetLowerLimitAttr().Set(-180.);j.GetUpperLimitAttr().Set(180.)
s.GetRootLayer().Save()
cfg=json.loads((root/'configs/r12_robot.json').read_text());cfg.update(usd_path='models/r12/r12_exploration.usda',lower=[-math.pi]*5,upper=[math.pi]*5)
cfg.pop('target_positions_path',None)
(root/'configs/r12_exploration_robot.json').write_text(json.dumps(cfg,indent=2)+'\n')
print('Simulation exploration profile created. Existing trained policy is not validated for it.')
