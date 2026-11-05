"""Read-only command-stage diagnostics from sensor ROS logs."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.swerve import teacher,BASE_DIRECTIONS


def margins(command):
    vectors=teacher(np.asarray(command)).reshape(4,2)
    raw=(np.arctan2(vectors[:,1],vectors[:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
    return np.abs(np.abs(raw)-np.pi/2)


def audit(path):
    rows=json.loads((path/'ros_results.json').read_text())
    samples=[]
    for row in rows:
        stages=row.get('command_stages')
        if not stages or not row.get('transport_posture_ok') or row.get('finished'):continue
        before=np.asarray(stages['before_envelope']);after=np.asarray(stages['after_envelope']);final=np.asarray(stages['final'])
        if max(np.linalg.norm(before[:2]),np.linalg.norm(final[:2]))<.02:continue
        sample={'stamp_ns':row['stamp_ns'],'envelope_xy_delta_m_s':float(np.linalg.norm(after[:2]-before[:2])),
                'post_envelope_xy_delta_m_s':float(np.linalg.norm(final[:2]-after[:2])),
                'yaw_delta_rad_s':float(final[2]-before[2]),'before_min_branch_margin_rad':float(min(margins(before))),
                'final_min_branch_margin_rad':float(min(margins(final))), 'stages':stages,'path_follower':row.get('path_follower')}
        samples.append(sample)
    def stats(key):
        v=[s[key] for s in samples]
        return {'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(max(v))} if v else None
    return {'saved_ros_rows':len(rows),'moving_samples':len(samples),'envelope_xy_delta_m_s':stats('envelope_xy_delta_m_s'),
            'post_envelope_xy_delta_m_s':stats('post_envelope_xy_delta_m_s'),
            'before_clear_final_near_branch_samples':sum(s['before_min_branch_margin_rad']>.1 and s['final_min_branch_margin_rad']<.06 for s in samples),
            'largest_envelope_changes':sorted(samples,key=lambda s:s['envelope_xy_delta_m_s'],reverse=True)[:10],
            'scope':'Sensor command logs only. Stage deltas are intentional filtering/guard effects, not automatically bugs. Branch margin ignores current module history, actuator dynamics and physical contact. Partial until completion.json exists.',
            'completed':(path/'completion.json').exists()}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');a=p.parse_args()
    result=audit(Path(a.run));Path(a.output).write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='largest_envelope_changes'},indent=2))
