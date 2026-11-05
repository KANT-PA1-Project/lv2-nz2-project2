"""Exercise the actual mission transport branch without starting ROS/Isaac."""
import ast
from pathlib import Path
import numpy as np
from arm_practice.arm_command_gate import ArmCommandGate
from arm_practice.transport_posture import TransportPosture

def branch():
 tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/r12_sensor_ros.py').read_text())
 matches=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="arm_gate.mode == 'transport'"]
 assert len(matches)==1
 return compile(ast.fix_missing_locations(ast.Module(body=matches,type_ignores=[])),'mission_transport_branch','exec')

def test_missing_clearance_revokes_cached_transport_target():
 names=[f'arm_joint{i+1}'for i in range(5)]
 gate=ArmCommandGate(names,[-1]*5,[1]*5,[.15]*5)
 gate.select('transport',[0]*5);assert gate.submit('transport',names,[.33]*5,0)
 gate.reference=[.2]*5;gate.hold_target=[.1]*5
 t=TransportPosture(dict(arm_target=[.33]*5,joint_tolerance_rad=.04,joint_velocity_limit_rad_s=.15,settle_seconds=.5),require_clearance=True)
 ctx=dict(arm_gate=gate,transport=t,arm_cfg={'joint_names':names},base_stopped=True,encoder_valid=True,arm_velocity=[0]*5)
 code=branch()
 for k in range(80):
  q=np.full(5,.001*k);ctx.update(ns=k*50_000_000,arm_position=q)
  exec(code,ctx)
  assert gate.target is None and gate.reference is None and not t.ready
  np.testing.assert_array_equal(gate.step(k*.05,q,k*.05),q)
 assert t.state=='await_swept_volume_clearance'

def test_all_mission_transport_constructors_require_clearance():
 tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/r12_sensor_ros.py').read_text())
 calls=[n for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id=='TransportPosture']
 assert len(calls)==3
 assert all(any(k.arg=='require_clearance' and isinstance(k.value,ast.Constant) and k.value.value is True for k in c.keywords)for c in calls)
