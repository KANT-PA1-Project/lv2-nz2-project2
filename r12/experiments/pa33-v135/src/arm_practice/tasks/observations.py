"""Position error observable for a fixed-base reach task."""
from isaaclab.utils.math import subtract_frame_transforms


def tool_position_error(env):
    robot = env.scene['robot']
    command = env.command_manager.get_term('ee_pose')
    pos, _ = subtract_frame_transforms(robot.data.root_pos_w.torch,
                                       robot.data.root_quat_w.torch,
                                       robot.data.body_pos_w.torch[:, command.body_idx])
    return (command.command[:, :3] - pos) * 10.0
