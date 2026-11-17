"""Offline errors. Ground truth must never enter the sensor adapter."""
import numpy as np
from scipy.spatial.transform import Rotation

def symmetry_rotations():
    return [Rotation.from_euler('z',k*90,degrees=True).as_matrix()@flip for k in range(4)for flip in (np.eye(3),np.diag([1,-1,-1]))]
def angle_deg(a,b):return float(np.degrees(Rotation.from_matrix(np.asarray(a).T@np.asarray(b)).magnitude()))
def errors(observed,truth):
    a=np.asarray(observed,float);b=np.asarray(truth,float)
    if a.shape!=(4,4)or b.shape!=(4,4)or not np.isfinite(a).all()or not np.isfinite(b).all():raise ValueError('invalid_evaluator_transform')
    return dict(translation_error_mm=float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),translation_delta_mm=((a[:3,3]-b[:3,3])*1000).tolist(),raw_orientation_error_deg=angle_deg(a[:3,:3],b[:3,:3]),square_prism_symmetry_orientation_error_deg=min(angle_deg(a[:3,:3],b[:3,:3]@S)for S in symmetry_rotations()))
