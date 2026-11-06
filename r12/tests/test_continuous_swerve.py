import ast,math,types,unittest
from pathlib import Path
import numpy as np
import torch
from arm_practice.latest_policy import allocate

class ContinuousTests(unittest.TestCase):
 def test_one_misaligned_wheel_does_not_stop_others(self):
  current=np.array([math.pi/2-.5,math.pi/2,0,-math.pi/2])
  _,w=allocate([.2,0,0],current,current,np.zeros(4))
  self.assertGreater(np.count_nonzero(np.abs(w)>.01),1)
 def test_continuity_across_old_cutoff_and_residual_gating(self):
  def wheels(error):
   q=np.array([math.pi/2-error,math.pi/2,0,-math.pi/2]);return allocate([.2,0,0],q,q,np.ones(4))[1]
  self.assertLess(np.max(np.abs(wheels(.24999)-wheels(.25001))),.01)
  self.assertLess(abs(wheels(math.pi/2)[0]),1e-8)
 def test_training_allocator_matches_runtime_on_random_batch(self):
  # Execute the actual environment method without starting Isaac Sim.
  tree=ast.parse((Path(__file__).resolve().parents[1]/'rl/swerve_highspeed_v2_env.py').read_text())
  cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Env')
  method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_pre_physics_step')
  scope={'torch':torch,'math':math};exec(compile(ast.Module(body=[method],type_ignores=[]),'<env allocator>','exec'),scope)
  rng=np.random.default_rng(42);n=64;command=rng.uniform(-.8,.8,(n,3));command[0]=0
  current=rng.uniform(-math.pi/2,math.pi/2,(n,4));actions=rng.uniform(-1,1,(n,4))
  fake=types.SimpleNamespace(evaluation=True,command=torch.tensor(command),actions=torch.zeros(n,4),device='cpu',steer=torch.arange(4),steering=torch.tensor(current),step_dt=.05,gain=torch.ones(n,4),sensor_bad=torch.zeros(n,dtype=torch.bool))
  fake.robot=types.SimpleNamespace(data=types.SimpleNamespace(joint_pos=types.SimpleNamespace(torch=torch.tensor(current))))
  scope['_pre_physics_step'](fake,torch.tensor(actions))
  for i in range(n):
   a,w=allocate(command[i],current[i],current[i],actions[i])
   np.testing.assert_allclose(fake.steering[i],a,atol=1e-6);np.testing.assert_allclose(fake.wheels[i],w,atol=1e-5)
