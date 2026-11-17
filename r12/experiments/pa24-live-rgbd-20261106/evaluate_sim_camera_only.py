"""Post-fit simulation truth comparison. Never imported by the fitter/controller."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation

BASE=Path(__file__).resolve().parent


def main():
    candidate=json.loads((BASE/'simulation_camera_candidate.json').read_text())
    source=BASE/'runs/fresh-view-04/source'
    config=json.loads((source/'configs/sensors/camera_rig.json').read_text())['arm_camera']
    truth=np.eye(4)
    truth[:3,:3]=Rotation.from_euler('xyz',config['optical_rpy_rad']).as_matrix()
    truth[:3,3]=config['optical_center_xyz_m'];truth[1,3]+=config['stereo_baseline_m']/2
    estimate=np.asarray(candidate['T_link_camera_candidate'])
    error=np.linalg.inv(truth)@estimate
    report=dict(scope='post-fit comparison to authored simulation camera placement only',
                camera='arm_camera_left_optical',fit_read_this_reference=False,
                simulation_mount_translation_error_m=float(np.linalg.norm(error[:3,3])),
                simulation_mount_rotation_error_rad=float(Rotation.from_matrix(error[:3,:3]).magnitude()),
                held_out_consistency_residuals=candidate['holdout_residuals'],
                physical_accuracy_verified=False,pad_calibrated=False,
                calibration_accepted=False,motion_authorized=False,error_bound_validated=False,
                conclusion='small holdout consistency residuals do not establish absolute extrinsic accuracy or a general error bound')
    (BASE/'evaluation_only_camera_candidate.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
