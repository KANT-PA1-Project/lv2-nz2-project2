#!/usr/bin/env python3
import sys,json,numpy as np,torch
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.swerve import SwervePolicy,teacher,actuate
rng=np.random.default_rng(20261030);torch.manual_seed(20261030);torch.set_num_threads(4)
x=rng.uniform([-.2,-.2,-.4],[.2,.2,.4],(50000,3)).astype('float32');y=teacher(x).astype('float32')
x=torch.from_numpy(x);y=torch.from_numpy(y);net=SwervePolicy();opt=torch.optim.Adam(net.parameters(),lr=.003)
for epoch in range(50):
 for ix in torch.randperm(len(x)).split(1024):
  loss=((net(x[ix])-y[ix])**2).mean();opt.zero_grad();loss.backward();opt.step()
net.eval();torch.save(net.state_dict(),root/'models/swerve/policy.pt');torch.jit.trace(net,torch.zeros(1,3)).save(str(root/'models/swerve/policy.ts'))
rng=np.random.default_rng(20261104);test=rng.uniform([-.2,-.2,-.4],[.2,.2,.4],(10000,3)).astype('float32')
with torch.no_grad():error=net(torch.from_numpy(test)).numpy()-teacher(test)
report={'method':'Supervised inverse-kinematics warm start, not learned traction/dynamics or SLAM','samples':50000,'epochs':50,'seed':20261030,'heldout_seed':20261104,'heldout_samples':10000,'wheel_velocity_rmse_m_s':float(np.sqrt(np.mean(error**2))),'wheel_velocity_max_abs_error_m_s':float(abs(error).max()),'input':'body vx vy yaw_rate','output':'4 wheel planar velocity vectors; deterministic angle wrapping, steering slew and wheel speed limits'}
(root/'reports/local/swerve/training.json').write_text(json.dumps(report,indent=2));print(report)
