"""Report the current repository gates without fabricating calibration data."""
from pathlib import Path
import argparse
import dataclasses
import hashlib
import json
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--r12-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'remaining_gates.json')
    args=parser.parse_args();root=args.r12_root.resolve()
    sys.path.insert(0,str(root/'src'))
    from arm_practice.native_rgbd.calibrated_approach import CalibratedApproach
    from arm_practice.mission_markers import destination_spec
    calibration=json.loads((root/'configs/r12_tcp_calibration.json').read_text())
    gripper=json.loads((root/'configs/r12_gripper.json').read_text())
    def forbidden_fk(q):raise RuntimeError('unexpected_FK_before_valid_calibration')
    result=CalibratedApproach(forbidden_fk).check(calibration,{}, {},now_ns=time.monotonic_ns(),
        run_id='read_only_gate_audit',model_sha256=hashlib.sha256((root/'models/r12/r12_arm.urdf').read_bytes()).hexdigest(),
        fk_sha256=hashlib.sha256((root/'src/arm_practice/kinematics.py').read_bytes()).hexdigest())
    destinations={name:destination_spec(name) for name in ('L1','L2','L3')}
    report=dict(scope='read-only current repository gate audit; no control or approval',
                terminal_calibration_guard=dataclasses.asdict(result),
                tcp_calibration_method=calibration['method'],
                physically_calibrated=calibration.get('physically_calibrated',False),
                gripper_actuated_calibration=calibration.get('gripper_actuated',False),
                motor_torque_to_finger_force_mapping=gripper.get('motor_torque_to_finger_force_mapping'),
                destination_mount_geometry_verified={name:spec['mount_geometry_verified'] for name,spec in destinations.items()},
                destination_mount_transforms_available={name:spec['T_tag_grid'] is not None for name,spec in destinations.items()},
                fixture_truth_substituted_for_measurement=False,
                calibration_accepted=False,motion_authorized=False,full_mission_completed=False,
                remaining=['independent validated pad/TCP and camera calibration with object-to-TCP mapping',
                           'validated translation/rotation/robot-geometry uncertainty bound',
                           'verified destination marker-to-grid mount transforms',
                           'terminal corridor/contact exception and native grasp/lift/carry/place evidence'])
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
