#!/usr/bin/env python3
"""Finite held-out evaluation of R12 reach policies in Isaac Sim PhysX."""
import argparse
import json
from pathlib import Path
import warp as wp
wp.config.enable_backward = False
import gymnasium as gym
import numpy as np
import torch
from isaaclab.app import launch_simulation
from isaaclab_physx.physics import PhysxCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from importlib.metadata import version
from rsl_rl.runners import OnPolicyRunner
import arm_practice.tasks
from arm_practice.tasks.reach_cfg import OwnArmReachCfg
from arm_practice.tasks.ppo_cfg import OwnArmPPORunnerCfg
from arm_practice.kinematics import forward_position

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--checkpoint', type=Path)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--num-envs', type=int, default=16)
parser.add_argument('--episodes', type=int, default=64)
parser.add_argument('--seed', type=int, default=2026)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
cfg = OwnArmReachCfg()
cfg.scene.num_envs = args.num_envs
cfg.sim.physics = PhysxCfg()
cfg.seed = args.seed
cfg.observations.policy.enable_corruption = False
cfg.commands.ee_pose.debug_vis = False
agent_cfg = handle_deprecated_rsl_rl_cfg(OwnArmPPORunnerCfg(), version('rsl-rl-lib'))
with launch_simulation(cfg, {'headless': True, 'device': 'cuda:0'}):
    raw = gym.make('Practice-Reach-OwnArm', cfg=cfg).unwrapped
    env = RslRlVecEnvWrapper(raw)
    command = raw.command_manager.get_term('ee_pose')
    # Fresh continuous joint samples: no training goal bank reused for evaluation.
    robot_cfg = json.loads((root/'configs/r12_robot.json').read_text())
    rng = np.random.default_rng(args.seed)
    qs = rng.uniform(-1, 1, (4096,5))*np.array(robot_cfg['action_scale'])
    positions = np.array([forward_position(root/'models/r12/r12_arm.urdf', 'arm_link5',
                         dict(zip(robot_cfg['joint_names'], q))) for q in qs])
    command.targets = torch.tensor(positions, dtype=torch.float32, device=raw.device)
    obs, _ = env.reset()
    policy = None
    if args.checkpoint:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device='cuda:0')
        runner.load(str(args.checkpoint))
        policy = runner.get_inference_policy(device=raw.device)
        export = args.output.parent/'exported'
        runner.export_policy_to_jit(path=str(export), filename='policy.pt')
        runner.export_policy_to_onnx(path=str(export), filename='policy.onnx')
    robot = raw.scene['robot']
    initial_base = robot.data.root_pos_w.torch.clone()
    completed, success, violations, nonfinite = 0, 0, 0, 0
    max_drift, max_speed = 0., 0.
    quotas = torch.tensor([args.episodes // args.num_envs + int(i < args.episodes % args.num_envs)
                           for i in range(args.num_envs)], device=raw.device)
    episode_counts = torch.zeros(args.num_envs, dtype=torch.long, device=raw.device)
    velocity_violations = 0
    errors = []
    samples = []
    with torch.inference_mode():
        for step in range(360*max((args.episodes+args.num_envs-1)//args.num_envs+1, 3)):
            eligible = episode_counts < quotas
            actions = policy(obs) if policy else torch.zeros((args.num_envs,5), device=raw.device)
            obs, reward, done, extras = env.step(actions)
            q = robot.data.joint_pos.torch
            limits = robot.data.joint_pos_limits.torch
            violations += int(((q < limits[...,0]-1e-3) | (q > limits[...,1]+1e-3)).sum())
            nonfinite += int((~torch.isfinite(q)).sum()) + int((~torch.isfinite(reward)).sum())
            max_speed = max(max_speed, float(robot.data.joint_vel.torch.abs().max()))
            velocity_violations += int((robot.data.joint_vel.torch.abs() > .51).sum())
            max_drift = max(max_drift, float((robot.data.root_pos_w.torch-initial_base).norm(dim=-1).max()))
            errors.append(float(command.metrics['position_error'].mean()))
            if step in (0, 30, 120, 359):
                samples.append({'step': step, 'q': q[0].tolist(),
                                'tcp_world': robot.data.body_pos_w.torch[0,command.body_idx].tolist(),
                                'goal_base': command.command[0,:3].tolist()})
            counted = done.bool() & eligible
            episode_counts += counted.long()
            completed += int(counted.sum())
            success += int((raw.termination_manager.get_term('success') & counted).sum())
            if policy:
                policy.reset(done)
            if completed >= args.episodes:
                break
    report = {'checkpoint': str(args.checkpoint) if args.checkpoint else None,
              'seed': args.seed, 'goal_source': 'held-out FK joint samples', 'episode_quota_per_env': quotas.tolist(),
              'configured_velocity_rad_s': robot_cfg['velocity'], 'requested_episodes': args.episodes, 'evaluation_complete': completed == args.episodes,
              'completed_episodes': completed, 'successful_episodes': success,
              'success_rate': success/completed if completed else None,
              'mean_step_position_error_m': float(np.mean(errors)),
              'joint_limit_violation_samples': violations, 'nonfinite_samples': nonfinite,
              'velocity_above_0_51_samples': velocity_violations, 'max_base_drift_m': max_drift, 'max_joint_speed_rad_s': max_speed,
              'samples': samples, 'steps': step+1, 'success_distance_m': robot_cfg['success_distance_m'],
              'limitations': ['arm_link5 origin, not calibrated tool tip',
                              'source self-collision filtered; collision safety not evaluated',
                              'source assumed 4 kg masses; not hardware qualified']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    env.close()
