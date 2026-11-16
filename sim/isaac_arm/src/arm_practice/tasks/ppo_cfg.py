"""Reuse the EA release's small MLP PPO settings; the robot comes from reach_cfg."""
from isaaclab.utils import configclass
from isaaclab_tasks.core.reach.config.franka.agents.rsl_rl_ppo_cfg import FrankaReachPPORunnerCfg


@configclass
class OwnArmPPORunnerCfg(FrankaReachPPORunnerCfg):
    experiment_name = "own_arm_reach"
    max_iterations = 500
    save_interval = 50
