"""Offline snapshot feasibility screen, never a runtime controller.

Search the segment from previous to requested body twist. Require a physically
reachable steering endpoint for either signed rolling representation, wheel
setpoint acceleration, and body slew. No fallback is silently declared safe.
Recorded states remain factual; candidate commands are not fed back to physics.
"""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.swerve import teacher,BASE_DIRECTIONS,RADIUS


def screen(previous, requested, steering, wheel_reference, dt=.05):
    fractions=np.linspace(0,1,101)
    candidates=np.asarray(previous)[None,:]+fractions[:,None]*(np.asarray(requested)-previous)
    vectors=teacher(candidates).reshape(-1,4,2)
    raw=(np.arctan2(vectors[:,:,1],vectors[:,:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
    angle=np.where(raw>np.pi/2,raw-np.pi,np.where(raw< -np.pi/2,raw+np.pi,raw))
    speed=np.linalg.norm(vectors,axis=2)/RADIUS
    speed*=np.where(abs(raw)>np.pi/2,-1.,1.)
    # At zero module speed, steering can remain at its current position.
    angle=np.where(abs(speed)<1e-8,np.asarray(steering),angle)
    steer_ok=(abs(angle-steering)<=7.330382858*dt+1e-9).all(axis=1)
    wheel_ok=(abs(speed-wheel_reference)<=24.45*dt+1e-9).all(axis=1)
    body_ok=(np.linalg.norm(candidates[:,:2]-np.asarray(previous)[:2],axis=1)<=.2*dt+1e-9)&(abs(candidates[:,2]-previous[2])<=.4*dt+1e-9)
    feasible=steer_ok&wheel_ok&body_ok
    ids=np.flatnonzero(feasible)
    return {'feasible':bool(len(ids)), 'fraction':float(fractions[ids[-1]]) if len(ids) else None,
            'requested_steer_ok':bool(steer_ok[-1]),'requested_wheel_ok':bool(wheel_ok[-1]),
            'requested_body_ok':bool(body_ok[-1]),'any_steer_feasible':bool(steer_ok.any()),
            'any_wheel_feasible':bool(wheel_ok.any())}


def audit(path):
    rows=json.loads((path/'sim_results.json').read_text());samples=[]
    for previous,row in zip(rows,rows[1:]):
        if not row.get('transport_posture_ok') or row.get('finished'):continue
        if np.linalg.norm(row['command'])<.02:continue
        t=previous['drive_telemetry']
        samples.append({'frame':row['frame'],**screen(np.array(previous['command']),np.array(row['command']),np.array(t['steer_position_rad']),np.array(t['wheel_target_rad_s']))})
    blocked=[x for x in samples if not x['feasible']]
    longest=streak=0;last=None
    for x in blocked:
        streak=streak+1 if last is not None and x['frame']==last+1 else 1
        longest=max(longest,streak);last=x['frame']
    return {'samples':len(samples),'no_candidate':len(blocked),'longest_consecutive_no_candidate':longest,
            'limited_candidates':sum(x['feasible'] and x['fraction']<1 for x in samples),
            'requested_steer_violations':sum(not x['requested_steer_ok'] for x in samples),
            'requested_wheel_slew_violations':sum(not x['requested_wheel_ok'] for x in samples),
            'requested_body_slew_violations':sum(not x['requested_body_ok'] for x in samples),
            'details':samples,'scope':'101-point line search only; failure is not proof no twist exists in 3D. Full alignment by next sample is stricter than the current cosine-gated allocator. Previous logged encoder steering and wheel reference are used; no truth input. No collision/jerk certification, dynamics rollout or actual deadlock proof. Body .2m/s2 is optimistic before rotating-frame budget; emergency stops may intentionally violate slew.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    result={Path(r).name:audit(Path(r)) for r in a.runs};Path(a.output).write_text(json.dumps(result,indent=2))
    print(json.dumps({k:{x:y for x,y in v.items() if x!='details'} for k,v in result.items()},indent=2))
