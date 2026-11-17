#!/usr/bin/env python3
"""Execute recorded paths in Isaac Sim; preserve every failure and measured trace."""
import json,time,math,os
from pathlib import Path
import numpy as np
import validate_r12_paths as v
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

node=v.node
client=ActionClient(node,FollowJointTrajectory,'/r12/arm_controller/follow_joint_trajectory')
assert client.wait_for_server(timeout_sec=30)
latest=None;trace=[]
def feedback(msg):
    global latest
    m=dict(zip(msg.name,msg.position))
    if all(n in m for n in v.NAMES):
        latest=[m[n] for n in v.NAMES]
        trace.append({'wall':time.monotonic(),'q':latest,'v':[dict(zip(msg.name,msg.velocity)).get(n,0.) for n in v.NAMES]})
node.create_subscription(JointState,'/r12/joint_states',feedback,10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:v.rclpy.spin_once(node,timeout_sec=.02)
spin(1);assert latest
scene_path=v.ROOT/os.environ.get('R12_SCENE_FILE','reports/local/r12_active_scene.json')
def scene(box):
    v.scene(box)
    token=time.time_ns();tmp=scene_path.with_suffix('.tmp');tmp.write_text(json.dumps({'token':token,'box':box}));tmp.replace(scene_path)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        spin(.05)
        if scene_path.with_suffix('.ack').exists() and scene_path.with_suffix('.ack').read_text()==str(token):return
    raise RuntimeError('Isaac scene acknowledgement missing')
def convert(tr):return [{'q':list(p.positions),'t':p.time_from_start.sec+p.time_from_start.nanosec*1e-9} for p in tr.points]
def execute(points):
    trace.clear();spin(.1)
    if any(max(lo-q,q-hi,0.)>1e-4 for q,lo,hi in zip(latest,v.CFG['lower'],v.CFG['upper'])):
        return {'accepted':False,'initial_state_out_of_bounds':True}
    # Bridge initial measurement to stored start at <= 0.15 rad/s.
    start=[min(hi,max(lo,q)) for q,lo,hi in zip(latest,v.CFG['lower'],v.CFG['upper'])];shift=max(abs(a-b) for a,b in zip(start,points[0]['q']))/.15+.2
    goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=v.NAMES
    for p in [{'q':start,'t':0.}]+[{'q':p['q'],'t':p['t']+shift} for p in points]:
        pt=JointTrajectoryPoint();pt.positions=p['q'];pt.time_from_start.sec=int(p['t']);pt.time_from_start.nanosec=int((p['t']%1)*1e9);goal.trajectory.points.append(pt)
    f=client.send_goal_async(goal);v.rclpy.spin_until_future_complete(node,f,timeout_sec=10)
    h=f.result()
    if not h or not h.accepted:return {'accepted':False}
    f=h.get_result_async();deadline=time.monotonic()+points[-1]['t']+shift+15
    while not f.done() and time.monotonic()<deadline:spin(.1)
    if not f.done():
        h.cancel_goal_async();spin(1);return {'accepted':True,'timeout':True}
    r=f.result();spin(.4)
    error=max(abs(a-b) for a,b in zip(latest,points[-1]['q']))
    speed=max((abs(x) for row in trace for x in row['v']),default=0.)
    limit=max((max(lo-q,q-hi,0.) for row in trace for q,lo,hi in zip(row['q'],v.CFG['lower'],v.CFG['upper'])),default=0.)
    return {'accepted':True,'status':r.status,'error_code':r.result.error_code,'message':r.result.error_string,'final':latest,'max_final_error':error,'max_measured_speed':speed,'max_limit_excess':limit,'feedback_samples':len(trace),'trace':list(trace),'passed':r.status==4 and error<=.02 and speed<=.5 and limit<=.002}

source=json.loads(v.OUT.read_text());results=[]
prior=json.loads((v.ROOT/os.environ['R12_RESUME']).read_text())['cases'] if os.environ.get('R12_RESUME') else []
completed={(x['kind'],x['index']):x for x in prior if x.get('passed')}
output=v.ROOT/os.environ.get('R12_EXEC_OUTPUT','reports/local/r12_paths_execution.json')
def save():output.write_text(json.dumps({'cases':results,'endpoint_tolerance_rad':.02,'limit_numerical_tolerance_rad':.002},indent=2)+'\n')
for case in source['cases']:
    key=(case['kind'],case['index'])
    if key in completed:
        results.append(completed[key]);save();continue
    if os.environ.get('R12_EXEC_SAMPLE') == '1' and (case['kind']=='limit_combination' or (case['kind']=='random' and case['index']%10)):
        results.append({'kind':case['kind'],'index':case['index'],'selected_for_execution':False});save();continue
    row={'kind':case['kind'],'index':case['index'],'original_plan_error':case['plan_error']}
    scene(None)
    # Explain invalid extreme poses rather than commanding a collision.
    if not v.valid(case['goal']):
        row.update(skipped='goal_in_self_collision',passed=False);results.append(row);save();print(row,flush=True);continue
    target_path=case['trajectory']
    if case['box']:
        scene(case['box'])
        response=v.plan(case['start'],case['goal'],5.)
        row['replan_error']=response.error_code.val
        target_path=convert(response.trajectory.joint_trajectory) if response.error_code.val==1 else []
        scene(None)
    if not target_path:
        row.update(skipped='no_valid_plan',passed=False);results.append(row);save();print(row,flush=True);continue
    response=v.plan(latest,case['start'],5.)
    if response.error_code.val!=1:
        row.update(skipped='setup_plan_failed',passed=False);results.append(row);save();print(row,flush=True);continue
    setup=execute(convert(response.trajectory.joint_trajectory));row['setup']={k:x for k,x in setup.items() if k!='trace'}
    if not setup.get('passed'):
        row.update(skipped='setup_execution_failed',passed=False);results.append(row);save();print(case['kind'],case['index'],row['skipped'],flush=True);continue
    scene(case['box'])
    # Independently check the sampled path, including measured start connector.
    bad=0;prev=latest
    for p in target_path:
        n=max(2,math.ceil(max(abs(a-b) for a,b in zip(prev,p['q']))/.01)+1)
        for q in v.lerp(prev,p['q'],n):bad+=not v.valid(q)
        prev=p['q']
    row['invalid_planned_samples']=bad
    if bad:
        row.update(skipped='interpolated_collision',passed=False)
    else:
        row['execution']=execute(target_path)
        # Check measured feedback against the exact planning scene (no distances available).
        measured=row['execution'].get('trace',[])
        row['invalid_measured_states']=sum(not v.valid(x['q']) for x in measured[::5])
        row['passed']=row['execution'].get('passed',False) and row['invalid_measured_states']==0
    results.append(row);save();print(case['kind'],case['index'],'PASS' if row['passed'] else 'FAIL',row.get('execution',{}).get('max_final_error'),flush=True)
scene(None);print('DONE',sum(x.get('passed',False) for x in results),len(results),flush=True)
node.destroy_node();v.rclpy.shutdown()
