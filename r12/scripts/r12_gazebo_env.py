"""Run Gazebo/ROS with this experiment's user-local Gazebo dependencies."""
import os
import sys
from pathlib import Path


def environment():
    repo = Path(__file__).resolve().parents[1]
    root = repo.parents[1] / 'outputs/r12/gazebo_runtime/root'
    prefixes = [root / 'opt/ros/lyrical', Path('/opt/ros/lyrical')]
    vendor = [p for prefix in prefixes for p in (prefix / 'opt').glob('*')]
    env = os.environ.copy()
    paths = {
        'PATH': [p / 'bin' for p in prefixes + vendor],
        'LD_LIBRARY_PATH': [p / 'lib' for p in prefixes + vendor] +
                           [root / 'usr/lib/x86_64-linux-gnu'],
        'GZ_CONFIG_PATH': [p / 'share/gz' for p in vendor],
        'GZ_SIM_SYSTEM_PLUGIN_PATH': [p / 'lib/gz-sim-10/plugins' for p in vendor],
        'GZ_SIM_RESOURCE_PATH': [p / 'share/gz/gz-sim' for p in vendor],
        'GZ_RENDERING_PLUGIN_PATH': [p / 'lib/gz-rendering-10/engine-plugins' for p in vendor],
        'GZ_PHYSICS_ENGINE_PATH': [p / 'lib/gz-physics-9/engine-plugins' for p in vendor],
        'GZ_SIM_PHYSICS_ENGINE_PATH': [p / 'lib/gz-physics-9/engine-plugins' for p in vendor],
        'GZ_GUI_PLUGIN_PATH': [p/'lib/gz-gui-10/plugins' for p in vendor] + [p/'lib/gz-sim-10/plugins/gui' for p in vendor],
        'OGRE2_RESOURCE_PATH': [p/'lib/OGRE-Next' for p in vendor],
        'AMENT_PREFIX_PATH': prefixes,
        'PYTHONPATH': [p / 'lib/python3.12/site-packages' for p in prefixes + vendor] +
                      [p / 'lib/python3.12/dist-packages' for p in prefixes + vendor] +
                      [p / 'lib/python' for p in vendor] + [root/'usr/lib/python3/dist-packages'],
    }
    for key, items in paths.items():
        env[key] = ':'.join([str(p) for p in items if p.exists()] +
                            ([env[key]] if env.get(key) else []))
    env['GZ_SIM_SERVER_CONFIG_PATH'] = str(root/'opt/ros/lyrical/opt/gz_sim_vendor/share/gz/gz-sim/server.config')
    env['GZ_RENDERING_RESOURCE_PATH'] = str(root/'opt/ros/lyrical/opt/gz_rendering_vendor/share/gz/gz-rendering')
    return env


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('Usage: r12_gazebo_env.py COMMAND [ARG ...]')
    os.execvpe(sys.argv[1], sys.argv[1:], environment())
