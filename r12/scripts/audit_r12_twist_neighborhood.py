"""Offline 3D first-step feasibility prerequisite for a future horizon planner.

No runtime integration. A horizon cannot start under these strict endpoint
constraints when its first-step sampled set is empty. This is not a proof of
physical infeasibility: ideal alignment excludes transient lateral rolling.
"""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.swerve import teacher,BASE_DIRECTIONS,RADIUS


def feasible_candidates(previous,steering,wheel_reference,dt=.05):
    # Body acceleration disk and yaw interval; include every grid axis and zero.
    xy=np.array([(x,y) for x in np.linspace(-1,1,9) for y in np.linspace(-1,1,9) if x*x+y*y<=1+1e-12])*.2*dt
    increments=np.array([[x,y,w] for x,y in xy for w in np.linspace(-.4*dt,.4*dt,9)])
    commands=previous+increments
    vectors=teacher(commands).reshape(-1,4,2)
    raw=(np.arctan2(vectors[:,:,1],vectors[:,:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
    angles=np.where(raw>np.pi/2,raw-np.pi,np.where(raw< -np.pi/2,raw+np.pi,raw))
    speeds=np.linalg.norm(vectors,axis=2)/RADIUS*np.where(abs(raw)>np.pi/2,-1.,1.)
    angles=np.where(abs(speeds)<1e-8,steering,angles)
    steering_ok=(abs(angles-steering)<=7.330382858*dt+1e-9).all(axis=1)
    wheel_ok=(abs(speeds-wheel_reference)<=24.45*dt+1e-9).all(axis=1)
    return commands,steering_ok,wheel_ok


def audit(run,blocked):
    rows=json.loads((run/'sim_results.json').read_text());byframe={x['frame']:i for i,x in enumerate(rows)};results=[]
    for frame in blocked:
        i=byframe[frame];prev=rows[i-1];t=prev['drive_telemetry']
        cmds,steer,drive=feasible_candidates(np.asarray(prev['command']),np.asarray(t['steer_position_rad']),np.asarray(t['wheel_target_rad_s']))
        good=steer&drive;desired=np.asarray(rows[i]['command']);choice=None
        if good.any():
            options=cmds[good];cost=np.sum((options-desired)**2*np.array([1,1,.1]),axis=1);choice=options[np.argmin(cost)].tolist()
        results.append({'frame':frame,'candidate_count':len(cmds),'steering_feasible':int(steer.sum()),'drive_feasible':int(drive.sum()),'jointly_feasible':int(good.sum()),'chosen':choice})
    return {'screened_segment_failures':len(results),'recovered_by_3d_grid':sum(x['chosen'] is not None for x in results),'still_empty':sum(x['chosen'] is None for x in results),'details':results,'scope':'441 candidates around previous command at .05s; ideal steering endpoint and previous wheel-reference slew. No truth input, no physics rollout, no jerk/collision certificate, no proof of continuous feasible-set emptiness. Body acceleration bound optimistic before rotation budget. Only first-step prerequisite, not a multi-step planner.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('screen');p.add_argument('output');a=p.parse_args();run=Path(a.run)
    previous=json.loads(Path(a.screen).read_text())[run.name]
    result=audit(run,[x['frame'] for x in previous['details'] if not x['feasible']])
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='details'},indent=2))
