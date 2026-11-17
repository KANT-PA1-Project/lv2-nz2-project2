"""Causal command-trend warning replay; later labels used only for scoring."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.swerve import teacher,BASE_DIRECTIONS


def desired_angles(command, reference):
    vectors=teacher(np.asarray(command)).reshape(4,2)
    raw=(np.arctan2(vectors[:,1],vectors[:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
    angle=np.where(raw>np.pi/2,raw-np.pi,np.where(raw< -np.pi/2,raw+np.pi,raw))
    hold=(abs(abs(raw)-np.pi/2)<.06)&(abs(reference)>.7)
    return np.where(hold,np.sign(reference)*np.pi/2,angle)


def warning(previous,command,steer,reference,dt=.05):
    trend=(command-previous)/dt
    trend[:2]*=min(1,.2/max(np.linalg.norm(trend[:2]),1e-12));trend[2]=np.clip(trend[2],-.4,.4)
    # Fixed causal extrapolation horizon, not a recorded future command.
    for horizon in [.05,.10,.15,.20,.25,.30]:
        proposal=command+trend*horizon
        large=(abs(desired_angles(proposal,reference)-steer)>1.5)&(abs(steer)>1.)
        if large.any():return True
    return False


def audit(run):
    rows=json.loads((run/'sim_results.json').read_text());samples=[];previous_event=False;previous_warning=False;events=[];alerts=[]
    for i in range(2,len(rows)):
        row=rows[i];prev=rows[i-1]
        if not row.get('transport_posture_ok') or row.get('finished'):
            previous_event=previous_warning=False;continue
        cmd=np.array(row['command']);old=np.array(prev['command']);t=prev['drive_telemetry']
        steer=np.array(t['steer_position_rad']);reference=np.array(t['steer_target_rad'])
        event=bool(np.linalg.norm(cmd[:2])>=.02 and ((abs(desired_angles(cmd,reference)-steer)>1.5)&(abs(steer)>1.)).any())
        warn=bool(max(np.linalg.norm(old),np.linalg.norm(cmd))>1e-6 and warning(old,cmd,steer,reference))
        if event and not previous_event:events.append(row['frame'])
        if warn and not previous_warning:alerts.append(row['frame'])
        samples.append({'frame':row['frame'],'warning':warn,'large_transition_request':event})
        previous_event=event;previous_warning=warn
    matches=[]
    for frame in events:
        lead=[s['frame'] for s in samples if s['warning'] and frame-10<=s['frame']<frame]
        matches.append({'event_frame':frame,'warning_lead_s':(frame-min(lead))*.05 if lead else None})
    false_alerts=[f for f in alerts if not any(f<=e<=f+10 for e in events)]
    return {'samples':len(samples),'event_onsets':len(events),'early_warned_events':sum(x['warning_lead_s'] is not None for x in matches),'warning_episodes':len(alerts),'unmatched_warning_episodes':len(false_alerts),'matches':matches,'unmatched_warning_frames':false_alerts,'details':samples,'scope':'Low-speed warning samples retained; event labels require XY speed >=.02. Events are allocator large-target requests (>1.5rad from encoder near endstop), not independently measured dangerous motion. Warning uses prior/current command, previous encoder and reference, bounded linear trend over .3s. Scoring allows .5s lead. Persistent-warning episode scoring can miss secondary events; closed-loop commands would differ after intervention. No physics acceptance or runtime deployment.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    result={Path(r).name:audit(Path(r)) for r in a.runs};Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:{x:y for x,y in v.items() if x!='details'} for k,v in result.items()},indent=2))
