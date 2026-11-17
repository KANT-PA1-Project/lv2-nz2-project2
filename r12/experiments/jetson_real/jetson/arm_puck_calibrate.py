#!/usr/bin/env python3
"""Bounded local arm hold compensation and fixed-tag calibration acquisition."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3DArray

sys.path.insert(0, "/home/nz2/nz2/arm_j2_diag")
from arm_v2_protocol import ArmSerial, ProtocolError, validate_fold, validate_pose
from arm_j2_tune_test import check


def corrected_goal(desired, actual, bias, dt):
    """Slow bounded integral compensation, with gripper position unchanged."""
    updated = list(bias)
    for index in range(5):
        error = desired[index] - actual[index]
        step = max(-dt, min(dt, 0.8 * error * dt))
        limit = 6.0 if index == 2 else 4.0
        updated[index] = max(-limit, min(limit, bias[index] + step))
    goal = [value + offset for value, offset in zip(desired, updated)]
    return goal, updated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM2")
    parser.add_argument("--output", required=True)
    parser.add_argument("--hold-only", action="store_true")
    parser.add_argument("--joints", nargs="+", choices=("j1", "j2", "j3", "j4", "roll"),
                        default=["j1", "j2", "j3", "j4", "roll"])
    parser.add_argument("--wrist-delta", type=float, default=0.0)
    parser.add_argument("--target-deg", type=float, nargs=6)
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    args = parser.parse_args()
    if not math.isfinite(args.wrist_delta) or abs(args.wrist_delta) > 5.0:
        parser.error("Wrist adjustment must be within5deg")
    if not math.isfinite(args.hold_seconds) or not 0.0 <= args.hold_seconds <= 60.0:
        parser.error("Extra holding time must be within0..60seconds")
    rclpy.init()
    node = rclpy.create_node("arm_puck_calibration")
    observations = deque(maxlen=200)
    depth_frames = deque(maxlen=20)
    intrinsics = {}
    face_reference = {}
    bridge = CvBridge()

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def camera_info(message):
        intrinsics["matrix"] = np.asarray(message.k).reshape(3, 3)
        intrinsics["distortion"] = np.asarray(message.d)

    def depth_image(message):
        data = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        scale = 0.001 if message.encoding == "16UC1" else 1.0
        depth_frames.append((stamp(message), np.asarray(data).copy(), scale))

    def observe(message):
        for detection in message.detections:
            if detection.id != "0" or not detection.results:
                continue
            pose = detection.results[0].pose.pose
            point = [pose.position.x, pose.position.y, pose.position.z]
            quat = [pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w]
            if not all(math.isfinite(value) for value in point + quat):
                continue
            quaternion = np.asarray(quat, dtype=float)
            length = np.linalg.norm(quaternion)
            if length < 0.5:
                continue
            xq, yq, zq, wq = quaternion / length
            normal = np.array([2 * (xq * zq + wq * yq),
                               2 * (yq * zq - wq * xq),
                               1 - 2 * (xq * xq + yq * yq)])
            if "normal" not in face_reference:
                face_reference["normal"] = normal
            if normal @ face_reference["normal"] < 0.85:
                continue
            depth_valid = False
            pnp_point = list(point)
            if intrinsics and depth_frames and point[2] > 0:
                image_time, depth, scale = min(depth_frames,
                    key=lambda item: abs(item[0] - stamp(message)))
                pixels, _ = cv2.projectPoints(
                    np.asarray(point, dtype=float).reshape(1, 3), np.zeros(3),
                    np.zeros(3), intrinsics["matrix"], intrinsics["distortion"])
                pixel = pixels.reshape(2)
                x, y = np.rint(pixel).astype(int)
                patch = depth[max(0, y - 2):max(0, y + 3),
                              max(0, x - 2):max(0, x + 3)] * scale
                valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 1.5)]
                if abs(image_time - stamp(message)) <= 0.1 and len(valid) >= 8:
                    ray = cv2.undistortPoints(pixel.reshape(1, 1, 2),
                        intrinsics["matrix"], intrinsics["distortion"]).reshape(2)
                    distance = float(np.median(valid))
                    point = [float(ray[0] * distance), float(ray[1] * distance), distance]
                    depth_valid = True
            if all(math.isfinite(value) for value in point + quat) and point[2] > 0:
                observations.append(dict(depth_valid=depth_valid, pnp_position_m=pnp_point,
                                         received=time.monotonic(),
                                         stamp_sec=message.header.stamp.sec,
                                         stamp_nanosec=message.header.stamp.nanosec,
                                         position_m=point, quaternion_xyzw=quat))

    subscription = node.create_subscription(
        Detection3DArray, "/nz2/tags", observe, qos_profile_sensor_data)
    depth_subscription = node.create_subscription(
        Image, "/gripper/aligned_depth_to_color/image_raw", depth_image, qos_profile_sensor_data)
    info_subscription = node.create_subscription(
        CameraInfo, "/gripper/color/camera_info", camera_info, qos_profile_sensor_data)
    report = dict(pid=os.getpid(), samples=[], passed=False, torque_off_commands=0)
    arm = None
    attempted = False
    previous_handler = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt("Termination requested")

    signal.signal(signal.SIGTERM, terminate)
    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and len(observations) < 8:
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(observations) < 8 and (not args.hold_only or args.target_deg is not None) and args.wrist_delta == 0.0:
            raise ProtocolError("Fixed tag0 observations unavailable")
        arm = ArmSerial.discover(port=args.port, timeout=0.12, settle_sec=0.1)
        report["identity"] = arm.identity()
        initial = arm.snapshot()
        check(initial, True, stopped=True)
        home = list(initial["position_deg"])
        home[3] += args.wrist_delta
        if args.target_deg is not None:
            if not args.hold_only or args.wrist_delta != 0.0:
                raise ProtocolError("Explicit small target requires hold-only and zero wrist delta")
            if max(abs(a - b) for a, b in zip(args.target_deg[:5], home[:5])) > 5.0:
                raise ProtocolError("Explicit target exceeds5deg local movement")
            if abs(args.target_deg[5] - home[5]) > 0.01:
                raise ProtocolError("Calibration target must preserve gripper opening")
            home = list(args.target_deg)
        validate_pose(home)
        validate_fold(initial["position_deg"], home)
        report["initial"] = initial
        bias = [0.0] * 6
        targets = [("hold", home, 12.0)]
        if not args.hold_only:
            for index, amplitude in enumerate((2.0, 2.0, 2.0, 2.0, 3.0)):
                if ("j1", "j2", "j3", "j4", "roll")[index] not in args.joints:
                    continue
                for direction in (1, -1):
                    target = list(home)
                    target[index] += direction * amplitude
                    validate_pose(target)
                    validate_fold(home, target)
                    targets.append((f"joint{index + 1}_{direction:+d}", target, 8.0))
                targets.append((f"joint{index + 1}_return", home, 6.0))
        attempted = True
        arm.query("R", "OK R", timeout=1.0)
        arm.send("A " + " ".join(f"{value:.3f}" for value in initial["position_deg"]))
        previous_time = time.monotonic()
        desired = list(initial["position_deg"])
        for name, destination, duration in targets:
            phase_start = time.monotonic()
            origin = list(desired)
            observations.clear()
            while time.monotonic() - phase_start < duration:
                now = time.monotonic()
                dt = max(0.001, min(0.08, now - previous_time))
                previous_time = now
                elapsed = now - phase_start
                # At most 1.5 degrees/s for the largest four-degree transition.
                span = max(abs(a - b) for a, b in zip(origin, destination))
                ramp = max(1.0, span)
                fraction = min(elapsed / ramp, 1.0)
                blend = fraction * fraction * (3.0 - 2.0 * fraction)
                desired = [a + (b - a) * blend for a, b in zip(origin, destination)]
                status = arm.snapshot()
                check(status, True)
                if max(abs(a - b) for a, b in zip(status["position_deg"], desired)) > 5.0:
                    raise ProtocolError("Measured error exceeds5deg")
                goal, bias = corrected_goal(desired, status["position_deg"], bias, dt)
                validate_pose(goal)
                validate_fold(status["position_deg"], goal)
                arm.send("A " + " ".join(f"{value:.3f}" for value in goal))
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0.0)
                if name != "hold" and not observations and elapsed > 2.0:
                    raise ProtocolError("Tag0 lost during local motion")
                if name != "hold" and observations and now - observations[-1]["received"] > 0.8:
                    raise ProtocolError("Tag0 observations stale")
                report["samples"].append(dict(
                    time=now, phase=name, desired_deg=list(desired),
                    goal_deg=goal, bias_deg=list(bias), status=status,
                    tag=observations[-1] if observations else None))
                time.sleep(0.015)
            error = [a - b for a, b in zip(status["position_deg"], destination)]
            print(json.dumps(dict(phase=name, error_deg=error,
                                  bias_deg=bias, position_deg=status["position_deg"])), flush=True)
            if max(abs(value) for value in error[:5]) > 0.8:
                raise ProtocolError("Settled position error exceeds0.8deg")
            if not args.hold_only and (not observations or time.monotonic() - observations[-1]["received"] > 0.8):
                raise ProtocolError("Fresh tag0 required before calibration sweep")
        report["passed"] = True
        report["holding_goal_deg"] = goal
        report["holding_bias_deg"] = bias
        Path(args.output).write_text(json.dumps(report, indent=2))
        hold_deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < hold_deadline:
            arm.send("K")
            status = arm.snapshot()
            check(status, True)
            if max(abs(a - b) for a, b in zip(status["position_deg"], desired)) > 3.0:
                raise ProtocolError("Holding error exceeds3deg")
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.02)
    except BaseException as error:
        report["passed"] = False
        report["error"] = repr(error)
        print("ERROR", repr(error), flush=True)
    finally:
        if arm is not None:
            if attempted:
                try:
                    report["stop"] = arm.query("S", "OK S", timeout=1.0)
                except BaseException as error:
                    report["stop_error"] = repr(error)
                    report["passed"] = False
            try:
                report["final"] = arm.snapshot()
                check(report["final"], True, stopped=True)
            except BaseException as error:
                report["final_error"] = repr(error)
                report["passed"] = False
            arm.close()
        Path(args.output).write_text(json.dumps(report, indent=2))
        signal.signal(signal.SIGTERM, previous_handler)
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps({key: value for key, value in report.items()
                          if key != "samples"}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
