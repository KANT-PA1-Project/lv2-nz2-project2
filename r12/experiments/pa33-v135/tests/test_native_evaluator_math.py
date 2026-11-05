import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from evaluator_math import errors,symmetry_rotations
from evaluator_capture import matrix

def test_known_translation_and_rotation():
 a=np.eye(4);a[:3,3]=[.003,.004,0];a[:3,:3]=Rotation.from_euler('x',10,degrees=True).as_matrix();r=errors(a,np.eye(4));assert abs(r['translation_error_mm']-5)<1e-10;assert abs(r['square_prism_symmetry_orientation_error_deg']-10)<1e-10

def test_all_symmetries_preserve_long_axis():
 for S in symmetry_rotations():
  a=np.eye(4);a[:3,:3]=S;assert errors(a,np.eye(4))['square_prism_symmetry_orientation_error_deg']<1e-10

def test_quaternion_order():
 m=matrix([1,2,3],[np.sqrt(.5),0,0,np.sqrt(.5)]);assert np.allclose(m[:3,:3]@[1,0,0],[0,1,0]);assert np.allclose(m[:3,3],[1,2,3])

def test_no_GT_in_decoder():
 root=Path(__file__).resolve().parents[1]
 assert 'evaluator_only'not in(root/'native_live_decoder.py').read_text()
 assert 'get_world_pose'not in(root/'native_live_decoder.py').read_text()
