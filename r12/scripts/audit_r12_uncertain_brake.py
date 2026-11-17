"""Offline finite branching diagnostic; empirical intervals are not guarantees."""
import argparse,itertools,json,time
from pathlib import Path
import numpy as np
from arm_practice.latest_policy import allocate
from audit_r12_actuator_lag import valid


def calibrate(rows,model):
    errors=[]
    for old,new in zip(rows,rows[1:]):
        if not valid(old):continue
        p=old['drive_telemetry'];n=new['drive_telemetry'];v=np.array(p['wheel_velocity_rad_s']);target=np.array(n['wheel_target_rad_s'])
        errors.extend(abs(v+model['wheel_alpha']*(target-v)-n['wheel_velocity_rad_s']))
    return float(np.percentile(errors,95))


def branches(state,command,model,epsilon):
    angle,wheel,ref,wref=state;a,w=allocate(command,angle,ref,np.zeros(4))
    threshold=np.maximum(2.,abs(w)/.2);low=np.maximum(0,abs(wheel)-epsilon);high=abs(wheel)+epsilon;gate=abs(a-angle)>.3
    options=[]
    for m in range(4):
        options.append([False] if not gate[m] or high[m]<=threshold[m] else ([True] if low[m]>threshold[m] else [False,True]))
    out=[]
    for mask in itertools.product(*options):
        mask=np.array(mask);nr=np.where(mask,angle,a);nw=wref+np.clip(np.where(mask,0.,w)-wref,-24.45*.05,24.45*.05)
        out.append((angle+model['steering_alpha']*(nr-angle),wheel+model['wheel_alpha']*(nw-wheel),nr,nw))
    return out


def audit(rows,model,epsilon,cap=32):
    windows=[]
    for i in range(1,len(rows)-10,5):
        if not all(valid(x) for x in rows[i:i+10]):continue
        t=rows[i]['drive_telemetry'];states=[tuple(np.array(t[k]) for k in ['steer_position_rad','wheel_velocity_rad_s','steer_target_rad','wheel_target_rad_s'])];pruned=0;start=time.perf_counter()
        for j in range(10):
            expanded=[s for state in states for s in branches(state,rows[i+j]['command'],model,epsilon)]
            unique={tuple(np.round(np.concatenate(s),9)):s for s in expanded};states=list(unique.values())
            if len(states)>cap:
                pruned+=len(states)-cap;states=[states[k] for k in np.linspace(0,len(states)-1,cap,dtype=int)]
        actual=rows[i+10]['drive_telemetry'];angles=np.array([s[0] for s in states]);wheels=np.array([s[1] for s in states]);true_a=np.array(actual['steer_position_rad']);true_w=np.array(actual['wheel_velocity_rad_s'])
        windows.append({'frame':rows[i]['frame'],'retained_states':len(states),'pruned_states':pruned,'elapsed_ms':(time.perf_counter()-start)*1000,'steering_component_envelope_contains_actual':bool(((true_a>=angles.min(0)-1e-3)&(true_a<=angles.max(0)+1e-3)).all()),'best_steering_mse_hindsight':float(np.min(np.mean((angles-true_a)**2,axis=1))),'max_steering_span_rad':float(np.max(np.ptp(angles,axis=0)))})
    return {'windows':len(windows),'branched_windows':sum(x['retained_states']>1 for x in windows),'pruned_windows':sum(x['pruned_states']>0 for x in windows),'strict_envelope_covered_windows':sum(x['steering_component_envelope_contains_actual'] for x in windows),'best_branch_steering_rmse_hindsight':float(np.sqrt(np.mean([x['best_steering_mse_hindsight'] for x in windows]))),'latency_p95_ms':float(np.percentile([x['elapsed_ms'] for x in windows],95)),'max_span_rad':max(x['max_steering_span_rad'] for x in windows),'details':windows}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('train');p.add_argument('validation');p.add_argument('model');p.add_argument('output');a=p.parse_args()
    model=json.loads(Path(a.model).read_text())['model'];train=json.loads((Path(a.train)/'sim_results.json').read_text());test=json.loads((Path(a.validation)/'sim_results.json').read_text());epsilon=calibrate(train,model)
    result=audit(test,model,epsilon);result['calibrated_wheel_error_p95_rad_s']=epsilon;result['scope']='V97 empirical one-step wheel residual p95, fixed across horizon, not a propagated bound. Steering uncertainty omitted. Up to32 branches retained by deterministic subsampling, not safe pruning. Future recorded commands used only for replay; future masks/encoders not used in prediction. Hindsight best-branch score is coverage potential, NOT deployable prediction error or candidate ranking.'
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='details'},indent=2))
