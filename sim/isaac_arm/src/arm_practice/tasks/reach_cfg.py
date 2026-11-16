"""Position-only reach adapter for Isaac Lab v3.0.0-EA. GPU execution unverified.

Built on the official ReachEnvCfg (BSD-3-Clause), not a copied training loop.
"""
import os
import re
from pathlib import Path
import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass
from isaaclab_tasks.core.reach.reach_env_cfg import ReachEnvCfg
from arm_practice.config import load_robot_config


def position_success(env, asset_cfg, tolerance):
    error = mdp.position_command_error(env, command_name="ee_pose", asset_cfg=asset_cfg)
    return error <= tolerance


@configclass
class OwnArmReachCfg(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        config_path = os.environ.get("ARM_CONFIG")
        if not config_path:
            raise ValueError("Set ARM_CONFIG to the absolute robot JSON path")
        c = load_robot_config(config_path)
        if not Path(c["usd_path"]).is_file():
            raise FileNotFoundError(c["usd_path"])
        names = [re.escape(n) for n in c["joint_names"]]
        values = lambda key: dict(zip(names, c[key], strict=True))
        self.scene.num_envs = 64
        self.scene.table = None
        self.scene.ground.init_state.pos = (0.0, 0.0, 0.0)
        self.scene.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=c["usd_path"]),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=values("home")),
            actuators={"arm": ImplicitActuatorCfg(
                joint_names_expr=names, joint_effort_limit=values("effort"),
                joint_velocity_limit=values("velocity"), stiffness=values("stiffness"), damping=values("damping"))},
        )
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=names, preserve_order=True,
            scale=values("action_scale"), use_default_offset=True,
            # Lab clips after scale + offset, so these bounds are radians/metres.
            clip={n: (max(lo, q-s), min(hi, q+s)) for n, lo, hi, q, s in zip(
                names, c["lower"], c["upper"], c["home"], c["action_scale"], strict=True)},
        )
        self.commands.ee_pose.body_name = c["tcp_link"]
        r = self.commands.ee_pose.ranges
        r.pos_x, r.pos_y, r.pos_z = [tuple(c["target_range"][a]) for a in ("x", "y", "z")]
        r.roll = r.pitch = r.yaw = (0.0, 0.0)
        self.commands.ee_pose.resampling_time_range = (12.0, 12.0)
        tcp = SceneEntityCfg("robot", body_names=[re.escape(c["tcp_link"])])
        self.rewards.end_effector_position_tracking.params["asset_cfg"] = tcp
        self.rewards.end_effector_orientation_tracking = None
        self.rewards.joint_vel.params["asset_cfg"].joint_names = names
        self.terminations.success = TerminationTermCfg(
            func=position_success, params={"asset_cfg": tcp, "tolerance": c["success_distance_m"]})
        # Start at the authored home pose before adding reset randomization.
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
