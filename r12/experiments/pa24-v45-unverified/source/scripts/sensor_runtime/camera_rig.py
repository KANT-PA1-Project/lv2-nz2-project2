"""Exactly three fixed upper cameras and one joint-following wrist camera."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from arm_practice.kinematics import forward_transform

def transforms(root,joints):
    cfg=json.loads((root/'configs/sensors/camera_rig.json').read_text());urdf=root/'models/r12/r12_arm.urdf';result={}
    for name in cfg['enabled']:
        for side,sign in [('left',1),('right',-1)]:
            if name!='arm_camera':T=forward_transform(urdf,name+'_'+side+'_optical',joints)
            else:
                local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',cfg['arm_camera']['optical_rpy_rad']).as_matrix();local[:3,3]=cfg['arm_camera']['optical_center_xyz_m'];local[1,3]+=sign*cfg['arm_camera']['stereo_baseline_m']/2
                T=forward_transform(urdf,'arm_link5',joints)@local
            result[name+'_'+side+'_optical']=T
    return result


class PreparedCameraRig:
    """Run-scoped calibration snapshot; current joint positions are never cached."""
    def __init__(self, root):
        import xml.etree.ElementTree as ET
        from arm_practice.kinematics import rotation
        cfg=json.loads((root/'configs/sensors/camera_rig.json').read_text())
        tree=ET.parse(root/'models/r12/r12_arm.urdf').getroot()
        by_child={j.find('child').get('link'):j for j in tree.findall('joint')}
        links={x.get('name') for x in tree.findall('link')}
        self.chains={};self.cameras={}
        for name in cfg['enabled']:
            for side,sign in [('left',1),('right',-1)]:
                key=name+'_'+side+'_optical';target='arm_link5' if name=='arm_camera' else key
                local=np.eye(4)
                if name=='arm_camera':
                    local[:3,:3]=Rotation.from_euler('xyz',cfg['arm_camera']['optical_rpy_rad']).as_matrix()
                    local[:3,3]=cfg['arm_camera']['optical_center_xyz_m'];local[1,3]+=sign*cfg['arm_camera']['stereo_baseline_m']/2
                self.cameras[key]=(target,local)
                if target in self.chains:continue
                if target not in links:raise ValueError('Unknown camera link')
                chain=[];current=target;seen=set()
                while current in by_child:
                    if current in seen:raise ValueError('Cyclic chain')
                    seen.add(current);j=by_child[current];origin=j.find('origin')
                    xyz=np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
                    rpy=np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
                    T=np.eye(4);T[:3,3]=xyz
                    T[:3,:3]=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
                    axis_node=j.find('axis');axis=np.fromstring(axis_node.get('xyz') if axis_node is not None else '1 0 0',sep=' ')
                    chain.append((j.get('name'),j.get('type'),T,axis));current=j.find('parent').get('link')
                self.chains[target]=tuple(reversed(chain))

    def transforms(self, joints):
        from arm_practice.kinematics import rotation
        targets={}
        for target,chain in self.chains.items():
            T=np.eye(4)
            for name,kind,local,axis in chain:
                motion=np.eye(4)
                if kind!='fixed':
                    value=joints[name]
                    if kind=='prismatic':motion[:3,3]=axis/np.linalg.norm(axis)*value
                    else:motion[:3,:3]=rotation(axis,value)
                T=T@local@motion
            targets[target]=T
        return {key:targets[target]@local for key,(target,local) in self.cameras.items()}
