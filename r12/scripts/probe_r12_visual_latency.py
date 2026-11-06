"""Isolated static archived-image timing probe; no simulator or control commands.

Synthetic replay stamps identify repeated publications, never fresh captures.
Continuously services callbacks and records the installed cuVSLAM timing fields.
Run separate processes/domains for transport comparisons; no mission validation.
"""
import argparse
from array import array
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts/sensor_runtime')]
from camera_rig import PreparedCameraRig
from r12_owned_session import OwnedSession
from r12_resource_admission import check


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--frames', type=int, default=200)
    parser.add_argument('--period', type=float, default=.35)
    parser.add_argument('--container', choices=('multi', 'single'), default='multi')
    parser.add_argument('--profile', type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain <= 101 or not 1 <= args.frames <= 500 or not .05 <= args.period <= 1:
        parser.error('Invalid bounded probe settings')
    args.output.mkdir(exist_ok=False)
    lockdir = args.output.parent / 'runtime_locks'
    lockdir.mkdir(exist_ok=True)
    lock = (lockdir / f'domain_{args.domain}.lock').open('a')
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    if not check(args.output / 'resource_admission.json', minimum_available_gib=4)['admitted']:
        raise RuntimeError('Resource admission deferred')
    meta = json.loads(args.frame.with_suffix('.json').read_text())
    digest = hashlib.sha256(args.frame.read_bytes()).hexdigest()
    if digest != meta['sha256']:
        raise ValueError('Archive checksum mismatch')
    with np.load(args.frame, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in ('run_id', 'frame_index', 'stamp_ns'):
        if data[key].item() != meta[key]:
            raise ValueError('Capture identity mismatch: ' + key)
    cfg = json.loads((ROOT / 'configs/r12_robot.json').read_text())
    q = np.asarray(meta['arm_measured_position'])
    if q.shape != (5,) or not np.isfinite(q).all() or meta['arm_feedback_stamp_ns'] != meta['stamp_ns']:
        raise ValueError('Capture-time arm feedback invalid')
    transforms = PreparedCameraRig(ROOT).transforms(dict(zip(cfg['joint_names'], q)))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    profile = None
    if args.profile:
        profile = args.output.resolve() / 'transport.xml'
        profile.write_bytes(args.profile.read_bytes())
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(profile)
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
    rclpy.init()
    node = rclpy.create_node('r12_visual_latency_probe', enable_rosout=False)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=.1)
    if any(name != node.get_name() for name, _ in node.get_node_names_and_namespaces()):
        node.destroy_node(); rclpy.shutdown()
        raise RuntimeError('Probe domain occupied')
    rows, callbacks, owned, child = [], [], None, None
    report = dict(scope='Repeated static archived image replay, not fresh sensing or mission',
                  source=str(args.frame.resolve()), source_sha256=digest,
                  original_capture_metadata=meta, synthetic_simulation_dt_s=.05,
                  wall_publication_period_s=args.period, gui=False, rviz=False,
                  simulator=False, mapping=False, inference=False, motion=False,
                  domain=args.domain, transport=os.getenv('FASTDDS_BUILTIN_TRANSPORTS', 'DEFAULT'),
                  container=args.container,
                  profile=None if profile is None else str(profile),
                  profile_sha256=None if profile is None else hashlib.sha256(profile.read_bytes()).hexdigest(),
                  frames_requested=args.frames, mission_pass=False)
    log = (args.output / 'cuvslam.log').open('x')
    try:
        clock = node.create_publisher(Clock, '/clock', 10)
        tf = StaticTransformBroadcaster(node)
        pairs, tfmessages = [], []
        for camera_index, camera in enumerate(('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera')):
            for side_index, side in enumerate(('left', 'right')):
                key = camera + '_' + side + '_optical'
                base = '/r12/' + ('stereo' if camera_index == 0 else camera) + '/' + side
                rgb = data['rgb'][side_index] if camera_index == 0 else data[key]
                im = Image(); im.header.frame_id = key
                im.width = 640; im.height = 480; im.encoding = 'rgb8'; im.step = 1920
                im.data = array('B', rgb.tobytes())
                ci = CameraInfo(); ci.header.frame_id = key
                ci.width = 640; ci.height = 480; ci.distortion_model = 'plumb_bob'; ci.d = [0.] * 5
                ci.k = [400., 0., 320., 0., 400., 240., 0., 0., 1.]
                ci.r = np.eye(3).flatten().tolist()
                ci.p = [400., 0., 320., -20. if side_index else 0., 0., 400., 240., 0., 0., 0., 1., 0.]
                depth = 10 if camera_index == 0 else 3
                pairs.append((base, node.create_publisher(Image, base + '/image', depth),
                              node.create_publisher(CameraInfo, base + '/camera_info', depth), im, ci))
                T = transforms[key]; m = TransformStamped()
                m.header.frame_id = 'base_link'; m.child_frame_id = key
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = T[:3, 3].tolist()
                quat = Rotation.from_matrix(T[:3, :3]).as_quat().tolist()
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = quat
                tfmessages.append(m)
        tf.sendTransform(tfmessages)

        def received(message, info, kind):
            mono = time.monotonic_ns(); wall = time.time_ns()
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            def field(key):
                return info.get(key) if isinstance(info, dict) else getattr(info, key, None)
            entry = dict(kind=kind, stamp_ns=stamp, callback_monotonic_ns=mono,
                         callback_wall_ns=wall, middleware_source_timestamp_ns=field('source_timestamp'),
                         middleware_received_timestamp_ns=field('received_timestamp'))
            if kind == 'status':
                entry.update(vo_state=int(message.vo_state),
                             node_callback_execution_time_s=message.node_callback_execution_time,
                             track_execution_time_s=message.track_execution_time)
            callbacks.append(entry)

        node.create_subscription(VisualSlamStatus, '/visual_slam/status', lambda m, i: received(m, i, 'status'), 100)
        node.create_subscription(Odometry, '/visual_slam/tracking/odometry', lambda m, i: received(m, i, 'odom'), 100)
        launch_source = (ROOT / 'configs/sensors/cuvslam.launch.py').read_text()
        if launch_source.count("executable='component_container_mt'") != 1:
            raise ValueError('Unexpected cuVSLAM launch structure')
        if args.container == 'single':
            launch_source = launch_source.replace("executable='component_container_mt'", "executable='component_container'")
        launch_file = args.output.resolve() / 'probe_cuvslam.launch.py'
        launch_file.write_text(launch_source)
        report['launch_sha256'] = hashlib.sha256(launch_source.encode()).hexdigest()
        child = subprocess.Popen(['ros2', 'launch', str(launch_file)],
                                 cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        owned = OwnedSession(child)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('cuVSLAM exited before readiness')
            owned.refresh(); rclpy.spin_once(node, timeout_sec=.05)
            if all(any(e.node_name == 'r12_visual_slam' for e in node.get_subscriptions_info_by_topic(base + '/image'))
                   for base, *_ in pairs[:6]):
                break
        else:
            raise RuntimeError('cuVSLAM image subscription discovery timeout')
        report['ready'] = True
        (args.output / 'running.json').write_text(json.dumps(dict(report, probe_pid=os.getpid(), slam_pid=child.pid), indent=2))
        start = time.monotonic()
        for index in range(args.frames):
            while time.monotonic() < start + index * args.period:
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., start + index * args.period - time.monotonic())))
            if child.poll() is not None:
                raise RuntimeError('cuVSLAM exited during replay')
            owned.refresh()
            stamp = 10**9 + index * 50_000_000
            c = Clock(); c.clock.sec = stamp // 10**9; c.clock.nanosec = stamp % 10**9
            begin = time.monotonic_ns(); clock.publish(c)
            for base, pub, infopub, im, ci in pairs:
                im.header.stamp = c.clock; ci.header.stamp = c.clock
                infopub.publish(ci); pub.publish(im)
            rows.append(dict(index=index, synthetic_stamp_ns=stamp, publication_start_ns=begin,
                             publication_complete_ns=time.monotonic_ns()))
            rclpy.spin_once(node, timeout_sec=0)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
        report['completed'] = True
    except BaseException as error:
        report['error'] = dict(type=type(error).__name__, message=str(error))
        raise
    finally:
        if owned:
            report['cleanup'] = owned.stop(); owned.close()
        elif child and child.poll() is None:
            child.terminate(); child.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown(); log.close()
        report.update(publications=rows, callbacks=callbacks)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(dict(frames=len(rows), callbacks=len(callbacks), completed=report.get('completed', False))))


if __name__ == '__main__':
    main()
