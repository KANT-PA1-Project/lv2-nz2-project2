"""Diagnostic CPU validation only. No actuator authority, GT or fixture fallback.
Trusted FK and trusted provenance verification must be supplied externally. Merely
setting claims or SHA256 values cannot make synthetic evidence authoritative.
"""
from dataclasses import dataclass
import hashlib,json,math
import numpy as np

def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def sha(v):return isinstance(v,str) and len(v)==64 and all(x in '0123456789abcdef'for x in v)
def rigid(v):
 a=np.asarray(v,dtype=float)
 if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1],atol=1e-10,rtol=0) or not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-8,rtol=0) or abs(np.linalg.det(a[:3,:3])-1)>1e-8:raise ValueError('rigid_transform')
 return a

def joints(q):
 if not isinstance(q,(list,tuple)) or len(q)!=5 or any(type(x)not in(int,float) or not math.isfinite(x) for x in q):raise ValueError('joint_vector')
 return np.asarray(q,float)

def angle(a,b):return float(np.arccos(np.clip((np.trace(a.T@b)-1)/2,-1,1)))
@dataclass(frozen=True)
class Result:
 diagnostic_consistent: bool
 reason: str
 binding_sha256: str|None=None
 motion_admitted: bool=False
 mission_admitted: bool=False

class CalibratedApproach:
 def __init__(self,link_fk):self.link_fk=link_fk
 def check(self,calibration,pose,proposal,*,now_ns,run_id,model_sha256,fk_sha256):
  try:return self._check(calibration,pose,proposal,now_ns,run_id,model_sha256,fk_sha256)
  except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError):return Result(False,'malformed_evidence')
 def _check(self,c,p,m,now,run,model,fkhash):
  no=lambda r:Result(False,r)
  if c.get('physically_calibrated')is not True:return no('calibration_unverified')
  if c['source']!='measured_calibration' or c['scope']!='simulation_sensor_grasp' or c['contains_fixture_offsets']is not False or c['validated']is not True:return no('calibration_provenance')
  if not all(sha(x)for x in(model,fkhash,c['report_sha256'],c['dataset_sha256'])) or c['model_sha256']!=model or c['fk_sha256']!=fkhash:return no('model_provenance')
  if c['object_frame']!='observed_cuboid' or c['tcp_frame']!='grasp_tcp' or c['link_frame']!='arm_link5' or c['camera_frame']!='arm_camera_left_optical':return no('calibration_frames')
  if not isinstance(run,str) or not run or p['run_id']!=run or m['run_id']!=run:return no('run_binding')
  if type(now)is not int or type(p['capture_ns'])is not int or not 0<=now-p['capture_ns']<=500_000_000:return no('capture_freshness')
  if p['accepted']is not True or p['source']!='wrist_rgbd_encoder_fk' or p['identity_verified']is not True or p['capture_settled']is not True:return no('unaccepted_sensor')
  if type(p['target_id'])is not int or not 0<=p['target_id']<12 or type(m['target_id'])is not int or m['target_id']!=p['target_id']:return no('identity')
  if m['phase']!='terminal_guarded_approach' or m['calibration_sha256']!=digest(c) or m['pose_sha256']!=digest(p):return no('proposal_binding')
  if m['model_sha256']!=model or m['fk_sha256']!=fkhash:return no('proposal_model')
  capture=joints(p['capture_q']);q0=joints(m['q_start']);q1=joints(m['q_end'])
  link_camera=rigid(c['T_link_camera']);link_tcp=rigid(c['T_link_tcp']);obj_tcp=rigid(c['T_object_tcp'])
  base_camera=rigid(self.link_fk(capture))@link_camera
  if not np.allclose(base_camera,rigid(p['T_base_camera']),atol=1e-8,rtol=0):return no('capture_fk_mismatch')
  base_object=base_camera@rigid(p['T_camera_object'])
  if not np.allclose(base_object,rigid(p['T_base_object']),atol=1e-8,rtol=0):return no('object_pose_mismatch')
  goal=base_object@obj_tcp
  axis_tcp=np.asarray(c['approach_axis_tcp'],float)
  if axis_tcp.shape!=(3,) or not np.isfinite(axis_tcp).all() or abs(np.linalg.norm(axis_tcp)-1)>1e-8:return no('calibration_axis')
  axis=goal[:3,:3]@axis_tcp
  if not np.allclose(axis,np.asarray(m['approach_axis_base'],float),atol=1e-8,rtol=0):return no('axis_mismatch')
  standoff=m['standoff_m']
  if type(standoff)not in(int,float) or not math.isfinite(standoff) or not 0<=standoff<=.1:return no('standoff')
  expected=goal.copy();expected[:3,3]-=axis*standoff
  declared=rigid(m['T_base_tcp_end'])
  actual0=rigid(self.link_fk(q0))@link_tcp;actual1=rigid(self.link_fk(q1))@link_tcp
  # Tight numerical consistency tolerance, NOT an empirical sensor error bound.
  for a,b in [(expected,declared),(actual1,declared)]:
   if np.linalg.norm(a[:3,3]-b[:3,3])>5e-5 or angle(a[:3,:3],b[:3,:3])>1e-4:return no('tcp_joint_fk_mismatch')
  delta=actual1[:3,3]-actual0[:3,3]
  if not np.allclose(delta,np.asarray(m['delta_tcp_m'],float),atol=1e-8,rtol=0):return no('delta_fk_mismatch')
  along=float(delta@axis);lateral=float(np.linalg.norm(delta-along*axis))
  if not 0<along<=.003 or lateral>1e-6 or angle(actual0[:3,:3],actual1[:3,:3])>.005:return no('approach_direction')
  binding=digest(dict(calibration_sha256=digest(c),pose_sha256=digest(p),proposal_sha256=digest(m),model_sha256=model,fk_sha256=fkhash))
  return Result(True,'diagnostic_mapping_consistent_no_authority',binding)
