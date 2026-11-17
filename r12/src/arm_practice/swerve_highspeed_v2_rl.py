"""Four independent wheel corrections with sensor-only body feedback prior."""
import math
import torch
from torch import nn
from torch.distributions import Normal
class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(24,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,4))
        self.critic=nn.Sequential(nn.Linear(24,128),nn.Tanh(),nn.Linear(128,1))
        self.logstd=nn.Parameter(torch.full((4,),-2.))
        nn.init.zeros_(self.actor[-1].weight);nn.init.zeros_(self.actor[-1].bias)
    def prior(self,x):
        error=x[:,:3]*x.new_tensor([.9,.9,1.5])
        correction=error*x.new_tensor([.7,.7,.8])
        heading=x[:,6:10]*math.pi/2+x.new_tensor([-math.pi/2,-math.pi/2,0,math.pi])
        px=x.new_tensor([.1,.1,-.1,-.1]);py=x.new_tensor([.1,-.1,.1,-.1])
        u=correction[:,0,None]-correction[:,2,None]*py
        v=correction[:,1,None]+correction[:,2,None]*px
        return ((u*heading.cos()+v*heading.sin())/.0306872/6).clamp(-1,1)
    def forward(self,x):
        return Normal(self.prior(x)+.3*self.actor(x),self.logstd.exp()),self.critic(x).squeeze(-1)
