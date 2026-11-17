"""Archived data readiness audit; does not calibrate or authorize control."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation

BASE = Path(__file__).resolve().parent
RUN = BASE / "runs/fresh-view-04"
SOURCE = RUN / "source"
sys.path.insert(0,str(SOURCE / "src"))
from arm_practice.kinematics import forward_transform


def main():
    rows = json.loads((RUN / "producer_capture_results.json").read_text())
    histories = [r['certificate']['body']['encoder_history'] for r in rows
                 if r.get('captured') and r.get('verification',{}).get('matched')]
    if len(histories)<2:raise ValueError('insufficient_archived_captures')
    q = np.array([h['encoder_q'] for h in histories],float)
    geometry = json.loads((SOURCE / "cad_geometry.json").read_text())
    frames=[]
    for v in q:
        joints={f'arm_joint{i+1}':float(x) for i,x in enumerate(v)}
        joints.update(geometry['gripper_open_positions'])
        frames.append(forward_transform(SOURCE / 'models/r12/r12_arm.urdf','arm_link5',joints))
    relative=[np.linalg.inv(frames[0])@T for T in frames[1:]]
    axes=np.array([Rotation.from_matrix(T[:3,:3]).as_rotvec() for T in relative])
    singular_values=np.linalg.svd(axes,compute_uv=False)
    rank=int(np.sum(singular_values>1e-5))
    config=json.loads((SOURCE/'configs/r12_tcp_calibration.json').read_text())
    report=dict(scope='archived stationary simulation data readiness only; not calibration or control approval',
                source_run=str(RUN),archived_captures=len(histories),
                encoder_joint_span_rad=np.ptp(q,axis=0).tolist(),
                relative_rotation_singular_values_rad=singular_values.tolist(),
                relative_rotation_axis_rank=rank,
                camera_extrinsic_identifiable=False,
                reason='no_relative_rotation_excitation' if rank==0 else 'independent_reference_and_observability_validation_required',
                dataset_used_to_fit_extrinsics=False,fixture_truth_used_as_calibration=False,
                current_tcp_method=config['method'],
                independent_pad_measurements_available=False,
                measured_position_error_bound_available=False,
                physically_calibrated=False,calibration_accepted=False,motion_authorized=False,
                next_requirements=['independently measured pad/TCP and marker mount references',
                                   'diverse settled sensor views with at least two nonparallel rotation axes',
                                   'separate calibration-fit and holdout datasets',
                                   'measured residual and uncertainty budget including encoder quantization and depth error'],
                model_sha256=hashlib.sha256((SOURCE/'models/r12/r12_arm.urdf').read_bytes()).hexdigest())
    (BASE/'calibration_readiness.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
