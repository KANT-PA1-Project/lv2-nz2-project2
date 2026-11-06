import ast,json,os
from pathlib import Path
import numpy as np
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12'
s=(repo/'scripts/r12_sensor_sim.py').read_text();tree=ast.parse(s)
block=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='stationary_capture' and any('stationary_reference.copy()' in ast.unparse(b) for b in n.body)][0]
assert ast.dump(block.orelse[0],include_attributes=False)==(out/'v334_original_allocator_ast.txt').read_text()
code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'allocator','exec')
for mode in (False,True):
 reference=np.arange(16,dtype=float)*.01;env=dict(np=np,stationary_capture=True,stationary_reference=reference,q=np.ones(16)*99,vel=np.ones(16)*10,steer=[0,2,4,6],latest_mode=mode)
 exec(code,env);assert np.array_equal(env['q'],reference) and not np.any(env['vel']);assert not np.any(env['w']);env['q'][0]=100;assert reference[0]==0
# Execute actual startup conflict checks without importing simulator/ROS.
conflicts=0
for file in ['r12_sensor_sim.py','r12_sensor_ros.py']:
 tree=ast.parse((repo/'scripts'/file).read_text());guard=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test).startswith('stationary_capture and any(')][0]
 code=compile(ast.fix_missing_locations(ast.Module(body=[guard],type_ignores=[])),'guard','exec')
 for key in ['R12_MISSION','R12_SENSOR_MOTION']:
  class FakeOS:environ={key:'1'}
  FakeOS.getenv=staticmethod(lambda k,d='0':FakeOS.environ.get(k,d))
  try:exec(code,dict(stationary_capture=True,os=FakeOS))
  except ValueError:conflicts+=1
assert conflicts==4
r=dict(stationary_allocator_modes_tested=2,original_allocator_branch_ast_unchanged=True,startup_conflicts_rejected=conflicts,reference_copy_independent=True,learned_allocator_not_called=True,runtime_launched=False,scope='Checks commanded joint references only;physical drift and settling require measurement.')
(out/'v334_verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
