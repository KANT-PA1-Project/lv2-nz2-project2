"""Sensor/reference-only empirical lag fit with separate-run validation.

A bounded first-order surrogate, not physical identification. Future recorded
commands are replay inputs for scoring only, never control observations.
"""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.transition_reference import moving_reference


def coefficient(errors,increments):
    e=np.asarray(errors).ravel();d=np.asarray(increments).ravel()
    if not np.isfinite(np.r_[e,d]).all() or e.shape!=d.shape or np.dot(e,e)<1e-12:raise ValueError('Insufficient finite excitation')
    return float(np.clip(np.dot(e,d)/np.dot(e,e),0,1))


def valid(row):
    return row.get('transport_posture_ok') and not row.get('finished') and np.linalg.norm(row['command'])>=1e-6 and row.get('motion_envelope',{}).get('reason')=='limited'


def fit(rows):
    se=[];sd=[];we=[];wd=[]
    for old,new in zip(rows,rows[1:]):
        if not valid(old):continue
        p=old['drive_telemetry'];n=new['drive_telemetry']
        # New row references were applied during this interval; old row command
        # drove that calculation. Use actual reference for identification only.
        se.append(np.array(n['steer_target_rad'])-p['steer_position_rad']);sd.append(np.array(n['steer_position_rad'])-p['steer_position_rad'])
        we.append(np.array(n['wheel_target_rad_s'])-p['wheel_velocity_rad_s']);wd.append(np.array(n['wheel_velocity_rad_s'])-p['wheel_velocity_rad_s'])
    return {'steering_alpha':coefficient(se,sd),'wheel_alpha':coefficient(we,wd),'fit_intervals':len(se),'dt_s':.05}


def evaluate(rows,alpha):
    result={}
    for steps in [1,5,10,20]:
        errors=[]
        for i in range(1,len(rows)-steps,5):
            if not all(valid(r) for r in rows[i:i+steps]):continue
            t=rows[i]['drive_telemetry'];angle=np.array(t['steer_position_rad']);wheel=np.array(t['wheel_velocity_rad_s']);ref=np.array(t['steer_target_rad']);wref=np.array(t['wheel_target_rad_s']);fast=False
            for j in range(steps):
                ref,wref,_=moving_reference(rows[i+j]['command'],angle,wheel,ref,wref)
                angle+=alpha['steering_alpha']*(ref-angle);wheel+=alpha['wheel_alpha']*(wref-wheel)
                old=np.array(rows[i+j]['drive_telemetry']['steer_position_rad']);new=np.array(rows[i+j+1]['drive_telemetry']['steer_position_rad'])
                fast|=bool(np.max(abs(new-old))/.05>2.)
            actual=rows[i+steps]['drive_telemetry']
            errors.append({'start_frame':rows[i]['frame'],'fast_transition':fast,'steering_mse':float(np.mean((angle-actual['steer_position_rad'])**2)),'wheel_mse':float(np.mean((wheel-actual['wheel_velocity_rad_s'])**2))})
        def stats(items):
            return {'samples':len(items),'steering_rmse_rad':float(np.sqrt(np.mean([x['steering_mse'] for x in items]))),'wheel_rmse_rad_s':float(np.sqrt(np.mean([x['wheel_mse'] for x in items])))} if items else {'samples':0}
        result[str(steps*.05)]={'all':stats(errors),'fast_transition':stats([x for x in errors if x['fast_transition']]),'other':stats([x for x in errors if not x['fast_transition']])}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('calibration_run');p.add_argument('validation_run');p.add_argument('output');a=p.parse_args()
    if Path(a.calibration_run).resolve()==Path(a.validation_run).resolve():raise ValueError('Separate runs required')
    train=json.loads((Path(a.calibration_run)/'sim_results.json').read_text());model=fit(train)
    test=json.loads((Path(a.validation_run)/'sim_results.json').read_text())
    output={'calibration_run':a.calibration_run,'validation_run':a.validation_run,'model':model,'baseline':evaluate(test,{'steering_alpha':1.,'wheel_alpha':1.}),'lag':evaluate(test,model),'scope':'Pooled scalar lag coefficients fit once on V97; V109 not used for fitting. Closed-loop logged commands replayed open-loop; future encoder only labels/scoring. No torque/contact/inertia model, extrapolation validity, hardware identification or runtime integration.'}
    Path(a.output).write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
