"""Offline event-time diagnostics. Future labels never enter control."""
import argparse,json
from pathlib import Path
import numpy as np
from audit_r12_transition_warning import desired_angles


def audit(run,warning):
    rows=json.loads((run/'sim_results.json').read_text());byframe={r['frame']:i for i,r in enumerate(rows)};events=[]
    for match in warning['matches']:
        frame=match['event_frame'];i=byframe[frame];prev=rows[i-1]['drive_telemetry'];current=np.array(prev['steer_position_rad']);target=desired_angles(np.array(rows[i]['command']),np.array(prev['steer_target_rad']))
        modules=np.flatnonzero((abs(target-current)>1.5)&(abs(current)>1.))
        lead=match['warning_lead_s'];warn_frame=frame-round(lead/.05) if lead is not None else frame
        start=rows[byframe[warn_frame]-1]['drive_telemetry'];results=[]
        for m in modules:
            travel=abs(target[m]-start['steer_position_rad'][m]);reference_travel=abs(target[m]-start['steer_target_rad'][m]);slew_min=reference_travel/7.330382858
            wheel_ref=abs(start['wheel_target_rad_s'][m]);brake_ref_min=max(0,wheel_ref-2.)/24.45
            # Retrospective first crossing only, not a measured isolated braking response.
            slow=None;aligned=None
            for future in rows[i:min(len(rows),i+41)]:
                t=future['drive_telemetry'];elapsed=(future['frame']-frame)*.05
                if slow is None and abs(t['wheel_velocity_rad_s'][m])<=2.:slow=elapsed
                if aligned is None and abs(t['steer_position_rad'][m]-target[m])<=.1:aligned=elapsed
            results.append({'module':int(m),'encoder_steering_travel_rad':float(travel),'reference_steering_travel_rad':float(reference_travel),'steering_reference_min_s':float(slew_min),'wheel_reference_to_2rad_s_min_s':float(brake_ref_min),'parallel_reference_min_s':float(max(slew_min,brake_ref_min)),'sequential_reference_min_s':float(slew_min+brake_ref_min),'observed_first_wheel_below2_after_event_s':slow,'observed_first_alignment_to_event_target_s':aligned,'lead_below_parallel_reference_min':None if lead is None else bool(lead<max(slew_min,brake_ref_min))})
        events.append({'frame':frame,'lead_s':lead,'modules':results})
    checked=[m for e in events for m in e['modules'] if m['lead_below_parallel_reference_min'] is not None]
    return {'events':events,'modules_with_warning':len(checked),'warning_shorter_than_reference_min':sum(m['lead_below_parallel_reference_min'] for m in checked),'scope':'Retrospective fixed event target used only for timing comparison, not prediction. Bounds describe commanded-reference slew, not certified physical stopping or steering times. Simultaneous/sequential bounds ignore torque, jerk, collisions and rolling compatibility. First encoder crossing may predate braking intervention; changing future targets make missing alignment ambiguous. No proof warning is sufficient even when it exceeds a lower bound.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('warnings');p.add_argument('runs_root');p.add_argument('output');a=p.parse_args()
    w=json.loads(Path(a.warnings).read_text());result={name:audit(Path(a.runs_root)/name,x) for name,x in w.items()};Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
