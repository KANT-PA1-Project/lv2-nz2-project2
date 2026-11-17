"""CPU-only terminal-approach evidence validator; never authorizes actuator motion.

Inputs must come from trusted independent perception/collision validators. SHA256
binds immutable evidence; it is NOT a signature or proof that claims are true.
Only intended target-to-pad distance may be less than 3 cm, and remains positive
until the separately certified close phase. All other object/environment geometry
retains 3 cm after uncertainty. No fixture/GT fallback is implemented.
"""
from dataclasses import dataclass
import hashlib,json,math

PHASE='terminal_guarded_approach'
PADS=('/World/Spawn/R12/gripper_left/pad_collision','/World/Spawn/R12/gripper_right/pad_collision')

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def _number(x,lo,hi):
    return type(x) in (int,float) and math.isfinite(x) and lo<=x<=hi

def _sha(x):
    return isinstance(x,str) and len(x)==64 and all(c in '0123456789abcdef' for c in x)

def _vec(v):
    return isinstance(v,(list,tuple)) and len(v)==3 and all(_number(x,-10,10) for x in v)

@dataclass(frozen=True)
class Decision:
    terminal_geometry_valid: bool
    reason: str
    motion_admitted: bool=False
    mission_admitted: bool=False

class TerminalPadCorridor:
    def __init__(self):
        self._used=set()

    def check(self,pose,proposal,certificate,*,now_ns,current_model_hash,current_world_hash,current_pad_hash,current_run_id):
        """Validate one bounded proposed segment. Consumes only a passing receipt."""
        try:
            return self._check(pose,proposal,certificate,now_ns,current_model_hash,current_world_hash,current_pad_hash,current_run_id)
        except (KeyError,TypeError,ValueError,OverflowError):
            return Decision(False,'malformed_evidence')

    def _check(self,p,m,c,now,model,world,pads,run):
        no=lambda reason:Decision(False,reason)
        if type(now)is not int or now<0:return no('clock')
        if not isinstance(run,str) or not run or any(x['run_id']!=run for x in (p,m,c)):return no('run_binding')
        if m['phase']!=PHASE or c['phase']!=PHASE:return no('phase')
        if type(p['target_id'])is not int or not 0<=p['target_id']<12 or m['target_id']!=p['target_id'] or type(m['target_id'])is not int or c['target_id']!=p['target_id'] or type(c['target_id'])is not int:return no('target_identity')
        if p['source']!='wrist_rgbd_encoder_fk' or p['accepted']is not True or p['identity_verified']is not True or p['capture_settled']is not True or p['current_settled']is not True:return no('pose_not_accepted')
        for key,age in [('capture_ns',500_000_000),('feedback_ns',100_000_000)]:
            if type(p[key])is not int or not 0<=now-p[key]<=age or c[key]!=p[key]:return no('stale_evidence')
        if type(c['created_ns'])is not int or not p['capture_ns']<=c['created_ns']<=now or now-c['created_ns']>100_000_000:return no('stale_certificate')
        if not all(_sha(x)for x in (model,world,pads)) or c['model_sha256']!=model or c['world_sha256']!=world or c['pad_geometry_sha256']!=pads:return no('model_binding')
        if c['pose_sha256']!=digest(p) or c['proposal_sha256']!=digest(m):return no('proposal_binding')
        if not isinstance(c['receipt_id'],str) or not c['receipt_id'] or c['receipt_id']in self._used:return no('receipt_replay')
        if tuple(c['exception_bodies'])!=PADS or c['exception_object_id']!=p['target_id'] or type(c['exception_object_id'])is not int:return no('pad_pair')
        b=p['error_bound']
        if b['validated']is not True or b['scope']!='simulation_sensor_pose' or not _sha(b['report_sha256']):return no('error_bound_missing')
        if not _number(b['translation_m'],0,.005) or not _number(b['rotation_rad'],0,.1) or not _number(b['robot_geometry_m'],0,.005):return no('error_bound_invalid')
        dims=p['dimensions_m']
        if not _vec(dims) or any(not _number(v,.001,.2) for v in dims):return no('object_geometry')
        if dims!=[.03,.03,.06] or p['shape']!='cuboid':return no('object_geometry')
        radius=math.sqrt(sum((x/2)**2 for x in dims))
        uncertainty=b['translation_m']+2*radius*math.sin(b['rotation_rad']/2)+b['robot_geometry_m']
        if not _number(m['open_aperture_m'],.030001,.075) or m['close_during_segment']is not False or not _number(m['aperture_rate_m_s'],0,0):return no('open_aperture')
        if not _vec(m['delta_tcp_m']) or not _vec(m['approach_axis_base']):return no('direction')
        delta=m['delta_tcp_m'];axis=m['approach_axis_base'];length=math.sqrt(sum(x*x for x in delta));an=math.sqrt(sum(x*x for x in axis))
        if not .000001<=length<=.003 or abs(an-1)>1e-6:return no('segment_size')
        along=sum(a*d for a,d in zip(axis,delta))
        lateral=math.sqrt(max(0,length*length-along*along))
        if along<=0 or lateral>1e-6 or not _number(m['delta_angle_rad'],0,.005):return no('direction')
        if c['approach_axis_base']!=axis or c['swept_segment_certified']is not True or c['unknown_volume_clear']is not True:return no('swept_unknown')
        # These are certified minima over the ENTIRE segment, not endpoint values.
        for key in ('nonpad_target_min_m','other_objects_min_m','environment_min_m'):
            if not _number(c[key],0,10) or c[key]-uncertainty<.03:return no('three_cm_margin')
        if not _number(c['self_min_m'],0,10) or c['self_min_m']-2*b['robot_geometry_m']<=0:return no('self_collision')
        if not _number(c['intended_pad_target_min_m'],0,10) or c['intended_pad_target_min_m']-uncertainty<=0:return no('premature_pad_contact')
        side_gap=(m['open_aperture_m']-dims[1])/2
        if side_gap-uncertainty<=0 or c['intended_pad_target_min_m']>side_gap+1e-6:return no('aperture_geometry')
        if not _number(m['duration_s'],.001,180) or not _number(m['elapsed_cycle_s'],0,180) or not _number(m['remaining_plan_s'],0,180) or m['elapsed_cycle_s']+m['duration_s']+m['remaining_plan_s']>180:return no('cycle_budget')
        if c['joint_rate_and_limits_verified']is not True:return no('joint_plan')
        qs,qe=m['q_start'],m['q_end']
        if not all(isinstance(q,(list,tuple)) and len(q)==5 and all(_number(v,-math.pi*2,math.pi*2) for v in q) for q in (qs,qe)):return no('joint_plan')
        if qs!=p['current_encoder_q'] or max(abs(a-b) for a,b in zip(qs,qe))/m['duration_s']>.005:return no('joint_plan')
        self._used.add(c['receipt_id'])
        return Decision(True,'cpu_contract_only_no_actuator_authority')
