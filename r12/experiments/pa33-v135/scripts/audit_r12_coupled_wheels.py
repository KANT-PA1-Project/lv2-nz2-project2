"""Empirical leave-one-wheel-out coupling, calibration and held-run validation."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from arm_practice.swerve import OFFSETS,BASE_DIRECTIONS,RADIUS
from arm_practice.transition_reference import moving_reference
from audit_r12_actuator_lag import valid

A=np.zeros((8,3));A[::2,0]=1;A[1::2,1]=1;A[::2,2]=-OFFSETS[:,1];A[1::2,2]=OFFSETS[:,0]
KEEP=[np.array([j for j in range(8) if j//2!=m]) for m in range(4)]
PINV=[np.linalg.pinv(A[k]) for k in KEEP]

def coupled(angles,wheels):
    axes=np.column_stack([np.cos(BASE_DIRECTIONS+angles),np.sin(BASE_DIRECTIONS+angles)])
    vectors=(RADIUS*wheels[:,None]*axes).ravel();out=[]
    for m in range(4):
        body=PINV[m]@vectors[KEEP[m]]
        out.append(float((A[2*m:2*m+2]@body)@axes[m]/RADIUS))
    return np.array(out)


def fit(rows):
    features=[];changes=[]
    for old,new in zip(rows,rows[1:]):
        if not valid(old):continue
        p=old['drive_telemetry'];n=new['drive_telemetry'];v=np.array(p['wheel_velocity_rad_s']);target=np.array(n['wheel_target_rad_s']);c=coupled(np.array(p['steer_position_rad']),v)
        features.extend(np.column_stack([target-v,c-v]));changes.extend(np.array(n['wheel_velocity_rad_s'])-v)
    x=np.array(features);y=np.array(changes)
    sol=minimize(lambda z:float(np.mean((x@z-y)**2)),[.4,.1],bounds=[(0,1),(0,1)],constraints=[{'type':'ineq','fun':lambda z:1-z.sum()}],method='SLSQP',options={'ftol':1e-12})
    if not sol.success:raise RuntimeError(sol.message)
    return {'reference_gain':float(sol.x[0]),'coupling_gain':float(sol.x[1]),'calibration_mse':float(sol.fun),'module_samples':len(y),'optimizer_success':bool(sol.success)}


def evaluate(rows,model,steer_alpha):
    out={}
    for steps in [1,5,10,20]:
        errors=[];mismatch=0
        for i in range(1,len(rows)-steps,5):
            if not all(valid(r) for r in rows[i:i+steps]):continue
            t=rows[i]['drive_telemetry'];angle=np.array(t['steer_position_rad']);wheel=np.array(t['wheel_velocity_rad_s']);ref=np.array(t['steer_target_rad']);wr=np.array(t['wheel_target_rad_s']);bad=False
            for j in range(steps):
                c=coupled(angle,wheel);ref,wr,brake=moving_reference(rows[i+j]['command'],angle,wheel,ref,wr)
                bad|=not np.array_equal(brake,rows[i+j+1]['drive_telemetry']['transition_braking'])
                wheel=wheel+model['reference_gain']*(wr-wheel)+model['coupling_gain']*(c-wheel)
                angle+=steer_alpha*(ref-angle)
            t=rows[i+steps]['drive_telemetry'];errors.append([np.mean((angle-t['steer_position_rad'])**2),np.mean((wheel-t['wheel_velocity_rad_s'])**2)]);mismatch+=bad
        mse=np.mean(errors,axis=0);out[str(steps*.05)]={'windows':len(errors),'steering_rmse_rad':float(np.sqrt(mse[0])),'wheel_rmse_rad_s':float(np.sqrt(mse[1])),'brake_mismatch_windows':int(mismatch)}
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('train');p.add_argument('validation');p.add_argument('lag_model');p.add_argument('output');a=p.parse_args()
    if Path(a.train).resolve()==Path(a.validation).resolve():raise ValueError('Separate runs required')
    train=json.loads((Path(a.train)/'sim_results.json').read_text());model=fit(train);lag=json.loads(Path(a.lag_model).read_text())['model'];test=json.loads((Path(a.validation)/'sim_results.json').read_text())
    result={'calibration_run':a.train,'validation_run':a.validation,'model':model,'validation':evaluate(test,model,lag['steering_alpha']),'scope':'Fit only V97, no V109 tuning. Other three sensor/predicted wheel vectors infer each module rolling speed; empirical coupling, not identified force/inertia. Convex coefficients constrain update but do not certify coupled-system stability. Recorded future commands only for replay evaluation. No truth observations, PPO, collision certification or runtime integration.'}
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
