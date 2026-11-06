"""Bounded body-twist and steering residual PPO; 35 observations including target state; incompatible with v2/v3 runtime."""
import torch
from torch import nn
from torch.distributions import Normal
class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(35,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,7))
        self.critic=nn.Sequential(nn.Linear(35,128),nn.Tanh(),nn.Linear(128,1))
        self.logstd=nn.Parameter(torch.full((7,),-2.5))
        nn.init.zeros_(self.actor[-1].weight);nn.init.zeros_(self.actor[-1].bias)
    def prior(self,x):
        error=x[:,:3]*x.new_tensor([.9,.9,1.5])
        correction=(error*x.new_tensor([.5,.5,.5])/x.new_tensor([.04,.04,.15])).clamp(-.8,.8)
        return torch.cat([torch.atanh(correction),torch.zeros_like(x[:,:4])],dim=-1)
    def forward(self,x):
        # Gaussian latent actions are transformed by tanh in Env, without clipping.
        return Normal(self.prior(x)+.25*torch.tanh(self.actor(x)),self.logstd.clamp(-4,-1).exp()),self.critic(x).squeeze(-1)
