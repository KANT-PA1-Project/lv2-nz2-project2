#!/usr/bin/env python3
"""Seeded fixed/random/obstacle planning regression against a local MoveIt server.

Checks collision validity on interpolated joint samples (not continuous proof).
Stores exact scene, endpoints and time-parameterized paths for reproduction.
"""
import json, math, random, time, os
from pathlib import Path
import numpy as np
import rclpy
from moveit_msgs.srv import GetMotionPlan, GetStateValidity, ApplyPlanningScene
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from arm_practice.kinematics import forward_position

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/os.environ.get('R12_PATH_OUTPUT','reports/local/r12_paths.json')
CFG=json.loads((ROOT/os.environ.get('R12_ROBOT_CONFIG','configs/r12_robot.json')).read_text())
NAMES=CFG['joint_names']
rng=random.Random(20261030)
rclpy.init();node=rclpy.create_node('r12_path_validation')
clients={k:node.create_client(t,n) for k,t,n in [('plan',GetMotionPlan,'/plan_kinematic_path'),('valid',GetStateValidity,'/check_state_validity'),('scene',ApplyPlanningScene,'/apply_planning_scene')]}
for c in clients.values():
    assert c.wait_for_service(timeout_sec=180)
def call(k,req,timeout=20):
    f=clients[k].call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
    if f.result() is None:raise RuntimeError(k+' timed out')
    return f.result()
def state(q):
    s=RobotState();s.joint_state.name=NAMES;s.joint_state.position=list(map(float,q));return s
def valid(q):
    req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q)
    return call('valid',req).valid

active_box=False
def scene(box=None):
    global active_box
    if box is None and not active_box:return
    req=ApplyPlanningScene.Request();req.scene.is_diff=True
    obj=CollisionObject();obj.header.frame_id='base_link';obj.id='r12_test_box'
    obj.operation=CollisionObject.REMOVE if box is None else CollisionObject.ADD
    if box:
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=box['size'])]
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=box['center'];pose.orientation.w=1.
        obj.primitive_poses=[pose]
    req.scene.world.collision_objects=[obj];assert call('scene',req).success
    active_box=box is not None

def plan(a,b,seconds=3.):
    req=GetMotionPlan.Request();m=req.motion_plan_request
    m.group_name='arm';m.allowed_planning_time=seconds;m.num_planning_attempts=1
    m.max_velocity_scaling_factor=.5;m.max_acceleration_scaling_factor=.5
    # Sim joint feedback is quantized; normalize only <= 0.0001 rad excess.
    normalized=[min(hi,max(lo,q)) if max(lo-q,q-hi,0.)<=1e-4 else q for q,lo,hi in zip(a,CFG['lower'],CFG['upper'])]
    m.start_state=state(normalized);m.planner_id='RRTConnectkConfigDefault'
    c=Constraints()
    c.joint_constraints=[JointConstraint(joint_name=n,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,q in zip(NAMES,b)]
    m.goal_constraints=[c]
    return call('plan',req).motion_plan_response

def sample():return [rng.uniform(lo,hi) for lo,hi in zip(CFG['lower'],CFG['upper'])]
def lerp(a,b,n):return [(1-t)*np.array(a)+t*np.array(b) for t in np.linspace(0,1,n)]
results=[];rejected=0
def save():
    OUT.write_text(json.dumps({'seed':20261030,'planner':'MoveIt OMPL RRTConnect','collision_interpolation_max_joint_step_rad':.02,'random_invalid_samples_rejected':rejected,'cases':results},indent=2)+'\n')
def check(kind,i,a,b,box=None):
    t=time.monotonic();r=plan(a,b);tr=r.trajectory.joint_trajectory
    row={'kind':kind,'index':i,'start':a,'goal':b,'box':box,'plan_error':r.error_code.val,'planning_seconds':r.planning_time,'trajectory':[]}
    checked=0;bad=0;maxv=0.
    if r.error_code.val==1:
        assert list(tr.joint_names)==NAMES
        prev=None
        for p in tr.points:
            q=list(p.positions);stamp=p.time_from_start.sec+p.time_from_start.nanosec*1e-9
            row['trajectory'].append({'q':q,'t':stamp})
            assert all(math.isfinite(x) and lo<=x<=hi for x,lo,hi in zip(q,CFG['lower'],CFG['upper']))
            if prev:
                n=max(2,math.ceil(max(abs(x-y) for x,y in zip(prev[0],q))/.02)+1)
                for v in lerp(prev[0],q,n):bad+=not valid(v);checked+=1
                dt=stamp-prev[1]
                if dt>0:maxv=max(maxv,max(abs(x-y)/dt for x,y in zip(prev[0],q)))
            else:bad+=not valid(q);checked+=1
            prev=(q,stamp)
    row.update(collision_samples=checked,invalid_collision_samples=bad,max_segment_speed=maxv,passed=r.error_code.val==1 and bad==0 and maxv<=.4,wall_seconds=time.monotonic()-t)
    results.append(row);save();print(kind,i,'PASS' if row['passed'] else 'FAIL',r.error_code.val,'samples',checked,flush=True)

def main():
    global rejected
    scene()
    home=[0.]*5
    fixed=[[.8,0,0,0,0],[-.8,0,0,0,0],[0,.25,.25,0,0],[0,-.25,-.25,0,0],[.5,.2,-.2,.2,-.2],[-.5,-.2,.2,-.2,.2],[1.2,.1,.1,.1,.1],[-1.2,-.1,-.1,-.1,-.1],[0,.3,-.3,.3,-.3],home]
    a=home
    for i,b in enumerate(fixed):check('fixed',i,a,b);a=b
    for j in range(5):
        for sign,limit in [('lower',CFG['lower'][j]),('upper',CFG['upper'][j])]:
            b=home.copy();b[j]=limit
            check('joint_limit_'+sign,j,home,b)
    # All 32 lower/upper corner combinations test simultaneous extreme poses.
    for mask in range(32):
        b=[CFG['upper'][j] if mask & (1<<j) else CFG['lower'][j] for j in range(5)]
        check('limit_combination',mask,home,b)
    for i in range(100):
        pair=[]
        while len(pair)<2:
            q=sample()
            if valid(q):pair.append(q)
            else:rejected+=1
        check('random',i,*pair)
    # Require valid endpoints and a blocked straight joint interpolation, so this
    # actually tests detours rather than merely placing an irrelevant obstacle.
    for i in range(10):
        found=False
        for attempt in range(300):
            scene();a,b=sample(),sample()
            if not valid(a) or not valid(b):continue
            mid=(np.array(a)+b)/2
            center=forward_position(ROOT/'models/r12/r12_arm.urdf','arm_link5',dict(zip(NAMES,mid))).tolist()
            box={'center':center,'size':[.035,.035,.035]};scene(box)
            if valid(a) and valid(b) and not valid(mid):
                found=True;break
        if not found:raise RuntimeError('Could not construct obstacle case')
        check('obstacle',i,a,b,box)
        results[-1]['straight_midpoint_in_collision']=True;save()
    scene();node.destroy_node();rclpy.shutdown()
    print('SUMMARY',sum(r['passed'] for r in results),'/',len(results),flush=True)

if __name__ == '__main__':
    main()
