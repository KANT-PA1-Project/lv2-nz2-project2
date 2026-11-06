"""Capture-encoder geometry producer for the existing native PickSensorAdapter."""
import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arm_practice.kinematics import forward_transform
from .camera_geometry import wrist_transform,optical_rotation
FILES=('models/r12/r12_arm.urdf','configs/r12_tcp_calibration.json','configs/sensors/camera_rig.json')
JOINTS=['arm_joint'+str(i)for i in range(1,6)]
class CaptureFK:
    def __init__(self,root):
        self.root=Path(root);self.hashes={f:hashlib.sha256((self.root/f).read_bytes()).hexdigest()for f in FILES}
        tree=ET.parse(self.root/FILES[0]).getroot();by_child={j.find('child').get('link'):j for j in tree.findall('joint')}
        current='arm_link5';moving=[];seen=set()
        while current in by_child:
            if current in seen:raise ValueError('cyclic_URDF')
            seen.add(current);j=by_child[current]
            if j.get('type')!='fixed':moving.append(j.get('name'))
            current=j.find('parent').get('link')
        if current!='base_link' or list(reversed(moving))!=JOINTS:raise ValueError('unexpected_base_or_joint_chain')
        self.cfg=json.loads((self.root/FILES[1]).read_text())
        if self.cfg['parent']!='arm_link5' or self.cfg['frame']!='grasp_tcp':raise ValueError('unexpected_TCP_calibration_frames')
        for k in ('translation_m','rpy_rad'):
            if len(self.cfg[k])!=3 or any(type(v)not in (int,float)or not math.isfinite(v)for v in self.cfg[k]):raise ValueError('invalid_TCP_calibration')
    def produce(self,*,run_id,frame_index,capture_ns,history,reported_camera_T,now_ns):
        if {f:hashlib.sha256((self.root/f).read_bytes()).hexdigest()for f in FILES}!=self.hashes:raise ValueError('geometry_files_changed')
        h=history
        if type(run_id)is not str or not run_id or type(frame_index)is not int or frame_index<0:raise ValueError('invalid_capture_identity')
        start,end=h['encoder_read_start_ns'],h['encoder_read_end_ns']
        if any(type(v)is not int or v<0 for v in (capture_ns,start,end,now_ns,h['physics_step'])) or not capture_ns==start<=end<=now_ns or end-start>1_000_000 or now_ns-capture_ns>500_000_000:raise ValueError('capture_encoder_time_mismatch')
        if h['joint_names']!=JOINTS or len(h['encoder_q'])!=5 or any(type(v)not in (int,float)or not math.isfinite(v)for v in h['encoder_q']):raise ValueError('invalid_encoder_mapping')
        joints=dict(zip(JOINTS,h['encoder_q']));camera=wrist_transform(self.root,joints)
        for reported in (reported_camera_T,h['camera_fk']['arm_camera_left_optical']):
            t=np.asarray(reported,float)
            if t.shape!=(4,4)or not np.isfinite(t).all()or not np.allclose(camera,t,atol=1e-8,rtol=0):raise ValueError('camera_FK_mismatch')
        offset=np.eye(4);offset[:3,3]=self.cfg['translation_m'];offset[:3,:3]=optical_rotation(self.cfg['rpy_rad'])
        tcp=forward_transform(self.root/FILES[0],'arm_link5',joints)@offset
        return dict(run_id=run_id,frame_index=frame_index,capture_physics_step=h['physics_step'],capture_monotonic_ns=capture_ns,encoder_capture_monotonic_ns=(start+end)//2,parent_frame='base_link',camera_frame='arm_camera_left_optical',tcp_frame='grasp_tcp',T_base_camera=camera.tolist(),T_base_tcp=tcp.tolist(),geometry_sha256=self.hashes.copy(),TCP_calibration_method=self.cfg['method'],physically_calibrated=self.cfg['physically_calibrated'],evidence_role='historical_encoder_FK_geometry_only',uncertainty_m=None,alignment_error_rad=None,motion_authorized=False)
