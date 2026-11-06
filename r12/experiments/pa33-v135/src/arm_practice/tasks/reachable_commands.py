"""Sample an FK-derived bank instead of unreachable Cartesian box corners."""
import json
from pathlib import Path
import torch
from isaaclab.envs.mdp.commands.pose_command import UniformPoseCommand
from isaaclab.envs.mdp.commands.commands_cfg import UniformPoseCommandCfg
from isaaclab.utils import configclass


class ReachablePoseCommand(UniformPoseCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        positions = json.loads(Path(cfg.target_positions_path).read_text())['positions']
        self.targets = torch.tensor(positions, dtype=torch.float32, device=self.device)
        if self.targets.ndim != 2 or self.targets.shape[1] != 3 or not len(self.targets):
            raise ValueError('Expected a nonempty N x 3 target bank')
        if not torch.isfinite(self.targets).all():
            raise ValueError('Nonfinite target bank')

    def _resample_command(self, env_ids):
        indices = torch.randint(len(self.targets), (len(env_ids),), device=self.device)
        self.pose_command_b[env_ids, :3] = self.targets[indices]
        self.pose_command_b[env_ids, 3:] = torch.tensor([0., 0., 0., 1.], device=self.device)


@configclass
class ReachablePoseCommandCfg(UniformPoseCommandCfg):
    class_type: type = ReachablePoseCommand
    target_positions_path: str = ''
