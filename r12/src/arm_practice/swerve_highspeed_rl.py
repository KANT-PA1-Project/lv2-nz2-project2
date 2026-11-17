"""Inference architecture for scalar PPO wheel-speed residual, version 4."""
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal
from .swerve import wheel_odometry

class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(21,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        self.critic=nn.Sequential(nn.Linear(21,64),nn.Tanh(),nn.Linear(64,1))
        self.logstd=nn.Parameter(torch.full((1,),-2.5))
        nn.init.zeros_(self.actor[-1].weight);nn.init.zeros_(self.actor[-1].bias)
    def prior(self,x):
        command=x[:,3:6]*x.new_tensor([.9,.9,1.5]);estimate=command-x[:,:3]*x.new_tensor([.9,.9,1.5])
        weights=x.new_tensor([1.,1.,.02]);den=(command.square()*weights).sum(-1,keepdim=True).clamp(min=1e-5)
        ratio=(command*estimate*weights).sum(-1,keepdim=True)/den
        gain=(ratio/(1+.3*x[:,-1:])).clamp(.7,1.4)
        return ((1/gain-1)/.3).clamp(-1,1)
    def forward(self,x):return Normal(self.prior(x)+.1*self.actor(x),self.logstd.exp()),self.critic(x).squeeze(-1)

class ResidualController:
    def __init__(self,path):
        checkpoint=torch.load(path,map_location='cpu',weights_only=True)
        self.model=Policy();self.model.load_state_dict(checkpoint['model']);self.model.eval();self.action=0.
    def scale(self,command,steering,wheel_velocity,gyro,accel):
        command=np.asarray(command);gyro=np.asarray(gyro)
        try:
            estimate,_=wheel_odometry(steering,wheel_velocity)
        except ValueError:
            self.action=0.;return 0.
        estimate[2]=gyro[2]
        obs=np.r_[(command-estimate)/[.9,.9,1.5],command/[.9,.9,1.5],np.asarray(steering)/(np.pi/2),np.asarray(wheel_velocity)/29.321531434,gyro/2,np.asarray(accel)/10,self.action]
        if not np.isfinite(obs).all():self.action=0.;return 0.
        if np.linalg.norm(command)<1e-6:self.action=0.;return 1.
        with torch.no_grad():self.action=float(self.model(torch.tensor(obs,dtype=torch.float32)[None].clamp(-10,10))[0].mean.clamp(-1,1)[0,0])
        return 1+.3*self.action
