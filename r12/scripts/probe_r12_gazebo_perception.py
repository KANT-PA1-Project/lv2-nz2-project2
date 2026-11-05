"""Bounded, read-only Gazebo camera/TensorRT probe. Never authorizes motion or IDs.

ROS receive age is measured separately from simulation timestamp age. Acquisition
to-receipt wall latency is unknown; receive time is never called capture time.
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.append('/usr/lib/python3/dist-packages')  # ROS empy dependency
repo = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(repo/'src'), str(repo/'work/perception_deps'),
               '/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts']
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import torch
from arm_practice.trt_perception import DetectorEngine, PoseEngine
from arm_practice.tag_detection import detect_tags
from train_centerpose import infer

CAMERAS = ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera')


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def image_array(msg):
    if msg.encoding not in ('rgb8', '32FC1') or msg.is_bigendian:
        raise ValueError('Unsupported image encoding/endian')
    dtype, channels = (np.uint8, 3) if msg.encoding == 'rgb8' else (np.float32, 1)
    row_bytes = msg.width * channels * np.dtype(dtype).itemsize
    if msg.step < row_bytes or len(msg.data) != msg.step * msg.height:
        raise ValueError('Invalid image stride/length')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :row_bytes].copy().view(dtype)
    shape = (msg.height, msg.width, 3) if channels == 3 else (msg.height, msg.width)
    return pixels.reshape(shape)


def serializable(value):
    if isinstance(value, dict):
        return {k: serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [serializable(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--engine-dir', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=40)
    args = parser.parse_args()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '0':
        raise RuntimeError('This probe requires explicit NVIDIA device 0')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    metadata = json.loads((args.engine_dir/'manifest.json').read_text())
    selection = json.loads((args.engine_dir/'selection.json').read_text())
    if selection['selected_for_simulation_trial'] != 'tensorrt_fp16':
        raise RuntimeError('FP16 trial backend is not selected')
    for source, digest in metadata['sources'].items():
        if hashlib.sha256(Path(source).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Model source changed: ' + source)
    for name in ('yolo_fp16.engine', 'centerpose_fp16.engine'):
        if hashlib.sha256((args.engine_dir/name).read_bytes()).hexdigest() != selection['engine_sha256'][name]:
            raise RuntimeError('Engine hash mismatch: ' + name)
    cfg = json.loads(Path(next(p for p in metadata['sources'] if p.endswith('inference.json'))).read_text())
    torch.set_num_threads(2)
    detector = DetectorEngine(args.engine_dir/'yolo_fp16.engine', metadata['names'])
    pose = PoseEngine(args.engine_dir/'centerpose_fp16.engine', metadata['pose_metric_depth_head'])
    K = np.array([[400., 0, 320], [0, 400, 240], [0, 0, 1.]])
    blank = np.zeros((480, 640, 3), np.uint8)
    detector.predict(blank, imgsz=cfg['imgsz'], conf=cfg['detector_confidence'])
    infer(pose, blank, [160, 120, 320, 360], K, device='cpu', dimensions_m=(.03, .03, .06), views=cfg['pose_views'])
    tags = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    rclpy.init()
    node = rclpy.create_node('r12_gazebo_perception_probe')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(String, '/r12/gazebo/perception_diagnostic', qos)
    lock = threading.Lock()
    frames = {c: collections.OrderedDict() for c in CAMERAS}
    infos, clock, errors, subscriptions = {}, {}, [], []

    def image_cb(camera, kind, msg):
        received = time.monotonic_ns()
        try:
            array = image_array(msg)
            if msg.header.frame_id != camera + '_left_optical' or array.shape[:2] != (480, 640):
                raise ValueError('Unexpected frame or image dimensions')
            with lock:
                rows = frames[camera]
                row = rows.setdefault(stamp_ns(msg.header.stamp), {})
                row[kind] = (array, received)
                while len(rows) > 4:
                    rows.popitem(last=False)
        except ValueError as error:
            with lock:
                errors.append(str(error))

    def info_cb(camera, msg):
        with lock:
            infos[camera] = msg

    def clock_cb(msg):
        with lock:
            clock.update(stamp=stamp_ns(msg.clock), received=time.monotonic_ns())

    subscriptions.append(node.create_subscription(Clock, '/clock', clock_cb, qos))
    for camera in CAMERAS:
        base = '/r12/' + ('stereo' if camera == 'r07_camera1' else camera) + '/left/'
        for kind, topic in [('rgb', 'image'), ('depth', 'depth')]:
            subscriptions.append(node.create_subscription(Image, base + topic,
                lambda m, c=camera, k=kind: image_cb(c, k, m), qos))
        subscriptions.append(node.create_subscription(CameraInfo, base+'camera_info',
            lambda m, c=camera: info_cb(c, m), qos))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    run_id = uuid.uuid4().hex
    started = time.monotonic()
    seen, samples, rejected, saved = {}, [], collections.Counter(), set()
    print(json.dumps({'ready': True, 'pid': os.getpid(), 'backend': 'tensorrt_fp16', 'run_id': run_id}), flush=True)
    try:
        with (out/'observations.jsonl').open('w') as log:
            while time.monotonic()-started < args.seconds and not (out/'STOP').exists():
                for camera in CAMERAS:
                    with lock:
                        choices = [(s, dict(row)) for s, row in frames[camera].items()
                                   if 'rgb' in row and 'depth' in row and s > seen.get(camera, -1)]
                        info, source_clock = infos.get(camera), dict(clock)
                    if not choices or info is None or not source_clock:
                        continue
                    stamp, frame = max(choices, key=lambda row: row[0])
                    seen[camera] = stamp
                    received = min(frame['rgb'][1], frame['depth'][1])
                    now = time.monotonic_ns()
                    if now-received > 500_000_000 or now-source_clock['received'] > 500_000_000:
                        rejected['receive_or_clock_stale'] += 1
                        continue
                    if abs(source_clock['stamp']-stamp) > 100_000_000:
                        rejected['simulation_stamp_gap'] += 1
                        continue
                    if info.header.frame_id != camera+'_left_optical' or not np.allclose(np.array(info.k).reshape(3, 3), K, atol=1e-4):
                        rejected['intrinsics_mismatch'] += 1
                        continue
                    image = cv2.cvtColor(frame['rgb'][0], cv2.COLOR_RGB2BGR)
                    before = time.monotonic_ns()
                    detected_tags = detect_tags(image, tags)
                    result = detector.predict(image, imgsz=cfg['imgsz'], conf=cfg['detector_confidence'])[0]
                    detector_done = time.monotonic_ns()
                    detections = []
                    for box in result.boxes:
                        cls, xy = int(box.cls.item()), box.xyxy[0].tolist()
                        record = dict(class_id=cls, class_name=result.names[cls], bbox=xy, confidence=float(box.conf.item()))
                        if cls < 4:
                            record['pose'] = infer(pose, image, xy, K, device='cpu', dimensions_m=(.03, .03, .06), views=cfg['pose_views'])
                            cx, cy = int((xy[0]+xy[2])/2), int((xy[1]+xy[3])/2)
                            depth = frame['depth'][0][max(0,cy-2):min(480,cy+3), max(0,cx-2):min(640,cx+3)]
                            valid = depth[np.isfinite(depth) & (depth > .02) & (depth < 10)]
                            record['depth_surface_m'] = float(np.median(valid)) if valid.size else None
                        detections.append(record)
                    done = time.monotonic_ns()
                    observation = serializable(dict(run_id=run_id, camera=camera, stamp_ns=stamp,
                        clock_domain='gazebo_sim_time', received_monotonic_ns=received,
                        completed_monotonic_ns=done, receive_to_completion_ms=(done-received)/1e6,
                        inference_ms=(done-before)/1e6, tags_and_detector_ms=(detector_done-before)/1e6,
                        fresh_since_receive=done-received <= 500_000_000,
                        acquisition_to_receive_wall_latency_verified=False,
                        acquisition_to_receive_wall_latency_ms=None,
                        execution_authorized=False, confirmed_inventory_count=0,
                        final_wrist_confirmation=False, box_pose_estimation_implemented=False,
                        candidate_detections=detections, raw_tag_observations=detected_tags))
                    if not observation['fresh_since_receive']:
                        rejected['inference_exceeded_receive_deadline'] += 1
                    log.write(json.dumps(observation)+'\n'); log.flush()
                    pub.publish(String(data=json.dumps(observation)))
                    samples.append(observation)
                    if camera not in saved:
                        cv2.imwrite(str(out/(camera+'.png')), image)
                        np.savez(out/(camera+'_sensor.npz'), rgb=frame['rgb'][0], depth=frame['depth'][0], stamp_ns=stamp)
                        saved.add(camera)
                time.sleep(.005)
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node(); rclpy.shutdown()
        summary = dict(run_id=run_id, backend='tensorrt_fp16', gpu_index=0,
            duration_wall_s=time.monotonic()-started, samples=len(samples),
            samples_by_camera=dict(collections.Counter(s['camera'] for s in samples)),
            candidate_classes=dict(collections.Counter(d['class_name'] for s in samples for d in s['candidate_detections'])),
            rejected=dict(rejected), errors=errors, confirmed_inventory_count=0,
            motion_commands_sent=0, arrivals=0, arm_observation_cycle_executed=False,
            scope='Stationary camera inference only; candidates are not confirmed IDs; no grasp/place',
            image_transport='raw RGB8 + float32 depth; no image compression',
            acquisition_to_receive_wall_latency_verified=False)
        if samples:
            summary['inference_median_ms'] = float(np.median([s['inference_ms'] for s in samples]))
            summary['receive_to_completion_median_ms'] = float(np.median([s['receive_to_completion_ms'] for s in samples]))
        (out/'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
