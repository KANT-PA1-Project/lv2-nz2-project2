"""Loaded by the Isaac Lab CLI's isaaclab.tasks entry-point group."""
import gymnasium as gym

gym.register(
    id="Practice-Reach-OwnArm",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.reach_cfg:OwnArmReachCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.ppo_cfg:OwnArmPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)


gym.register(
    id="Practice-Reach-R12",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.reach_cfg:R12ReachCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.ppo_cfg:OwnArmPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
