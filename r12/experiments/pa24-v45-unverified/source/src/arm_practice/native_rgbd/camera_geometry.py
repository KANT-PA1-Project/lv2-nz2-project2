"""Per-camera geometry metadata; no decoder or motion activation."""
import copy

def geometry(schema):
 return dict(schema='r12-per-camera-image-geometry-v1',camera_schema_sha256=schema.sha256,calibration_sha256=schema.calibration_sha256,camera_spec=copy.deepcopy(schema.config['cameras']['arm_camera_left_optical']))

import json
import numpy as np
from arm_practice.kinematics import forward_transform, rotation

def optical_rotation(rpy):
    return rotation([0,0,1],rpy[2]) @ rotation([0,1,0],rpy[1]) @ rotation([1,0,0],rpy[0])

def transforms(root,joints):
    cfg=json.loads((root/'configs/sensors/camera_rig.json').read_text());urdf=root/'models/r12/r12_arm.urdf';result={}
    for name in cfg['enabled']:
        for side,sign in [('left',1),('right',-1)]:
            if name!='arm_camera':T=forward_transform(urdf,name+'_'+side+'_optical',joints)
            else:
                local=np.eye(4);local[:3,:3]=optical_rotation(cfg['arm_camera']['optical_rpy_rad']);local[:3,3]=cfg['arm_camera']['optical_center_xyz_m'];local[1,3]+=sign*cfg['arm_camera']['stereo_baseline_m']/2
                T=forward_transform(urdf,'arm_link5',joints)@local
            result[name+'_'+side+'_optical']=T
    return result


def wrist_transform(root, joints):
    """Compute only the requested left wrist geometry, without other camera FK."""
    cfg=json.loads((root/'configs/sensors/camera_rig.json').read_text())
    if 'arm_camera' not in cfg['enabled']:raise ValueError('arm_camera_disabled')
    c=cfg['arm_camera'];local=np.eye(4)
    local[:3,:3]=optical_rotation(c['optical_rpy_rad'])
    local[:3,3]=c['optical_center_xyz_m']
    local[1,3]+=c['stereo_baseline_m']/2
    return forward_transform(root/'models/r12/r12_arm.urdf','arm_link5',joints)@local
