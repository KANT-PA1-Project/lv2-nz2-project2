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

# NZ2 팀 R12 로봇: 전체 로봇(차체 고정) + 팔·그리퍼 Reach (R12_NO_USB_URDF_USD/isaac/nz2_reach_cfg.py 와 같은 파일)
gym.register(
    id="NZ2-Reach-Arm",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_reach_cfg:NZ2ArmReachCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_reach_cfg:NZ2ArmPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# NZ2 팀 R12 로봇: 선반(박스 70 mm) 위 퍽 3종(직육면체 세움/누움, 원기둥 세움) 집어 들기 (nz2_pick_cfg.py)
gym.register(
    id="NZ2-Pick-Arm",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_pick_cfg:NZ2ArmPickCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_pick_cfg:NZ2ArmPickPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# NZ2 최대 도달 집기 실험: mission.pdf 맵(벽·박스) + 팔 중앙 + 앞 D435 충돌 끔, 스크립트 IK 로 거리×하향각 시험 (학습 없음)
gym.register(
    id="NZ2-MaxReach-Sweep",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_maxreach_cfg:NZ2MaxReachSweepCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_maxreach_cfg:NZ2PickFarPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# NZ2 원거리 집기 잔차 RL: IK 궤적 + 정책 잔차(±0.08 rad), 먼 줄 퍽·간격 0~60 mm·인식 오차 6 mm
gym.register(
    id="NZ2-PickFar-Arm",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_maxreach_cfg:NZ2PickFarCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_maxreach_cfg:NZ2PickFarPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# NZ2 바닥 고정 팔 + 카메라(손목 D435 + 상단 D435) 영상 end-to-end RL: 빨강/초록 퍽 중 명령 색 집기 (nz2_vispick_cfg.py)
gym.register(
    id="NZ2-VisPick-Fixed",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# 같은 장면, 카메라 없이 특권 상태로 학습 — 환경·보상이 풀리는지 먼저 확인하는 용도
gym.register(
    id="NZ2-VisPick-Fixed-State",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickStateCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickStatePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# IK + 잔차 RL (영상) / 같은 장면 카메라 없음 = expert-check (IK 단독 성공률, zero_agent)  — nz2_vispick_residual.py
gym.register(
    id="NZ2-VisPick-Fixed-Residual",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickResidualCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickResidualPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
gym.register(
    id="NZ2-VisPick-Fixed-Residual-State",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickResidualStateCfg",
        "rsl_rl_cfg_entry_point": "arm_practice.tasks.nz2_vispick_cfg:NZ2VisPickStatePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
