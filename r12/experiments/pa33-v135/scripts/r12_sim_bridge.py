#!/usr/bin/env python3
"""Run the fixed-base R12 asset with the bundled ROS bridge, for simulated I/O."""
import argparse
import time
import json
import numpy as np
from pathlib import Path
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=60)
parser.add_argument('--usd-path', default='models/r12/r12_arm.usda')
parser.add_argument('--scene-file', type=Path, help='Local validation scene JSON; mirrored in MoveIt')
args = parser.parse_args()
app = SimulationApp({'headless': True, 'disable_viewport_updates': True})
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension

enable_extension('isaacsim.ros2.bridge')
app.update()
from isaacsim.ros2.nodes.impl.graph_api import (
    create_ros2_joint_states_graph, Ros2JointStatesGraphConfig,
    create_ros2_clock_graph, Ros2ClockGraphConfig,
    create_ros2_tf_graph, Ros2TfGraphConfig)
root = Path(__file__).resolve().parents[1]
world = World(stage_units_in_meters=1.0, physics_dt=1/120, rendering_dt=1/60)
add_reference_to_stage(str(root/args.usd_path), '/World/R12')
world.scene.add_default_ground_plane()
create_ros2_joint_states_graph(Ros2JointStatesGraphConfig(
    articulation_root='/World/R12', publish_joint_states=True,
    subscribe_joint_states=True, publish_topic='/r12/joint_states',
    subscribe_topic='/r12/joint_commands'))
create_ros2_clock_graph(Ros2ClockGraphConfig())
create_ros2_tf_graph(Ros2TfGraphConfig(target_prim='/World/R12', parent_prim='/World', topic='/r12/tf'))
world.reset()
scene_token = None
start = time.monotonic()
print('R12_BRIDGE_READY', flush=True)
while time.monotonic()-start < args.seconds:
    if args.scene_file and args.scene_file.exists():
        data = json.loads(args.scene_file.read_text())
        if data['token'] != scene_token:
            if world.scene.object_exists('validation_box'):
                world.scene.remove_object('validation_box')
            if data.get('box'):
                box = data['box']
                world.scene.add(FixedCuboid(prim_path='/World/ValidationBox',name='validation_box',
                    position=np.array(box['center']),scale=np.array(box['size']),size=1.,color=np.array([.8,.2,.1])))
            scene_token = data['token']
            args.scene_file.with_suffix('.ack').write_text(str(scene_token))
    world.step(render=True)
    time.sleep(1/120)
world.stop()
app.close()
