"""Modeled pad/TCP frame contract audit; not independent metrology."""
from pathlib import Path
import itertools
import json
import numpy as np
from pxr import Usd,UsdGeom

BASE=Path(__file__).resolve().parent
SOURCE=BASE/'source'


def main():
    stage=Usd.Stage.Open(str(SOURCE/'models/r12/r12_mobile_gripper.usda'));cache=UsdGeom.XformCache()
    tcp=json.loads((SOURCE/'configs/r12_tcp_calibration.json').read_text())
    geometry=json.loads((SOURCE/'cad_geometry.json').read_text())
    pads=[]
    for side,sign in [('left',1),('right',-1)]:
        body=stage.GetPrimAtPath('/Robot/gripper_'+side)
        pad=stage.GetPrimAtPath('/Robot/gripper_'+side+'/pad_collision')
        if not pad.IsA(UsdGeom.Cube):raise ValueError('unsupported_pad_collider')
        M=np.asarray(cache.ComputeRelativeTransform(pad,body)[0]).T
        half=float(UsdGeom.Cube(pad).GetSizeAttr().Get())/2
        vertices=np.array(list(itertools.product([-half,half],repeat=3)))
        local=(M@np.c_[vertices,np.ones(8)].T).T[:,:3]
        minimum=local.min(axis=0);maximum=local.max(axis=0)
        inner=(minimum+maximum)/2;inner[1]=minimum[1] if sign==1 else maximum[1]
        pads.append(dict(side=side,inner_center_at_zero_slide_m=inner.tolist(),minimum_at_zero_slide_m=minimum.tolist(),maximum_at_zero_slide_m=maximum.tolist()))
    left=np.asarray(pads[0]['inner_center_at_zero_slide_m']);right=np.asarray(pads[1]['inner_center_at_zero_slide_m'])
    center=(left+right)/2;zero_gap=float(left[1]-right[1])
    if zero_gap<=0 or abs(left[0]-right[0])>1e-7 or abs(left[2]-right[2])>1e-7:raise ValueError('nonopposing_modeled_pads')
    opened=geometry['gripper_open_positions']['gripper_left_joint']
    fingertip=np.asarray(tcp['translation_m'])
    result=dict(scope='authored collider geometry audit only; not independent pad/TCP calibration',
                pads=pads,modeled_pad_contact_midpoint_link5_m=center.tolist(),
                configured_fingertip_tcp_link5_m=fingertip.tolist(),
                frame_origin_difference_m=float(np.linalg.norm(fingertip-center)),
                configured_tcp_method=tcp['method'],
                modeled_zero_slide_aperture_m=zero_gap,modeled_open_aperture_m=zero_gap+2*opened,
                configured_tcp_is_pad_midpoint=False,
                independent_pad_measurement_available=False,motor_to_finger_force_identified=False,
                calibration_accepted=False,motion_authorized=False,
                decision='Do not silently substitute a pad contact midpoint for the existing fingertip TCP. Calibrated object-to-TCP mapping and measured uncertainty are still required before terminal contact.')
    (BASE/'pad_tcp_frame_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='pads'},indent=2))


if __name__=='__main__':main()
