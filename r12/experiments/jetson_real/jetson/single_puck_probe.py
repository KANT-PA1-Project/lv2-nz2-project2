#!/usr/bin/env python3
"""Capture a bounded RGBD burst and measure a single puck without motion commands."""
import argparse
import importlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

SERIALS = {"gripper": "261322074035", "top": "261722075504"}
CLASSES = ("red_cylinder", "red_cuboid", "green_cylinder", "green_cuboid", "box", "apriltag")
ARM_JOINTS = ("arm_yaw_joint", "shoulder_joint", "elbow_joint",
              "wrist_pitch_joint", "wrist_roll_joint")


def camera_transform(urdf, camera, motor_deg):
    joints = {joint.find("child").get("link"): joint
              for joint in ET.parse(urdf).getroot().findall("joint")}
    # Final v99 axes include the base_footprint yaw flip and reversed yaw/roll axes.
    # Motor direction checks remain approximate until the physical zero is calibrated.
    angles = dict(zip(ARM_JOINTS, -np.deg2rad(motor_deg[:5])))
    link = "wrist_d435_color_optical_frame" if camera == "gripper" else "mast_d435_color_optical_frame"
    chain = []
    while link != "base_link":
        joint = joints[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    transform = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        roll, pitch, yaw = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        rx = cv2.Rodrigues(np.array([roll, 0., 0.]))[0]
        ry = cv2.Rodrigues(np.array([0., pitch, 0.]))[0]
        rz = cv2.Rodrigues(np.array([0., 0., yaw]))[0]
        local = np.eye(4)
        local[:3, :3] = rz @ ry @ rx
        local[:3, 3] = xyz
        transform = transform @ local
        if joint.get("type") in ("revolute", "continuous"):
            turn = np.eye(4)
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            turn[:3, :3] = cv2.Rodrigues(axis*angles[joint.get("name")])[0]
            transform = transform @ turn
    return transform


def measure_detection(image, depth, box, intrinsics, rs):
    x1, y1, x2, y2, confidence, class_id = box
    class_id = int(class_id)
    row = dict(class_id=class_id, class_name=CLASSES[class_id],
               confidence=confidence, bbox_xyxy=[x1, y1, x2, y2], depth_valid=False)
    if class_id > 3:
        return row
    left, right = max(0, int(x1)), min(image.shape[1], int(np.ceil(x2)))
    top, bottom = max(0, int(y1)), min(image.shape[0], int(np.ceil(y2)))
    if left >= right or top >= bottom:
        return row
    hsv = cv2.cvtColor(image[top:bottom, left:right], cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    color = ((hue < 13) | (hue > 167)) if class_id < 2 else ((hue > 34) & (hue < 91))
    color &= (saturation > 70) & (value > 40)
    row["color_fraction"] = float(color.mean())
    z = depth[top:bottom, left:right]
    valid = color & (z >= .10) & (z <= 1.5)
    yy, xx = np.nonzero(valid)
    row["valid_depth_pixels"] = len(xx)
    if row["color_fraction"] < .18 or len(xx) < 20:
        return row
    median = np.median(z[valid])
    selected = np.abs(z[yy, xx]-median) <= .015
    yy, xx = yy[selected], xx[selected]
    if len(xx) < 20:
        return row
    # Sample actual colored pixels, preserving each pixel's own depth and distortion model.
    indices = np.linspace(0, len(xx)-1, min(200, len(xx))).astype(int)
    points = [rs.rs2_deproject_pixel_to_point(
        intrinsics, [float(left+xx[i]), float(top+yy[i])], float(z[yy[i], xx[i]]))
        for i in indices]
    row.update(depth_valid=True, camera_surface_xyz_m=np.median(points, axis=0).tolist(),
               depth_m=float(median), depth_spread_m=float(np.ptp(z[yy, xx])))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=SERIALS, required=True)
    parser.add_argument("--pipeline-root", type=Path,
                        default=Path("/home/nz2/models/perception_6class_20261115"))
    parser.add_argument("--arm-root", type=Path, default=Path("/home/nz2/nz2/arm_j2_diag"))
    parser.add_argument("--urdf", type=Path,
                        default=Path("/home/nz2/nz2_urdf/real_final_plz_no_more_edit_v99_nz2fix.urdf"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--arm-port", default="")
    args = parser.parse_args()
    if args.frames < 3:
        parser.error("--frames must be at least 3")
    args.out.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(args.pipeline_root))
    from jetson_pipeline import Cuda, TrtModel, letterbox, yolo_decode
    import pyrealsense2 as rs
    import tensorrt as trt

    def arm_snapshot():
        sys.path.insert(0, str(args.arm_root))
        protocol = importlib.import_module("arm_v2_protocol")
        arm = protocol.ArmSerial.discover(port=args.arm_port, timeout=.2, settle_sec=.1)
        try:
            return arm.snapshot()
        finally:
            arm.close()

    before = arm_snapshot() if args.arm_port else None
    engine = TrtModel(trt, trt.Logger(trt.Logger.WARNING), Cuda(),
                      str(args.pipeline_root/"yolo_fp16.engine"))
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(SERIALS[args.camera])
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    frames = []
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        for index in range(args.frames):
            aligned = align.process(pipeline.wait_for_frames(5000))
            color = aligned.get_color_frame()
            image = np.asanyarray(color.get_data()).copy()
            depth = np.asanyarray(aligned.get_depth_frame().get_data()).astype(np.float32)*scale
            intr = color.profile.as_video_stream_profile().get_intrinsics()
            tensor, ratio, pad_x, pad_y = letterbox(image, engine.in_shape[2], engine.in_shape[3])
            prediction = engine(tensor)[engine.outputs[0]]
            boxes = yolo_decode(prediction, .4, .7, ratio, pad_x, pad_y, image.shape[:2])
            rows = [measure_detection(image, depth, box, intr, rs) for box in boxes]
            frames.append(dict(index=index, timestamp_ms=color.get_timestamp(), detections=rows))
        cv2.imwrite(str(args.out/"color.jpg"), image)
        np.savez_compressed(args.out/"rgbd.npz", bgr=image, depth_m=depth)
        (args.out/"intrinsics.json").write_text(json.dumps(dict(
            fx=intr.fx, fy=intr.fy, ppx=intr.ppx, ppy=intr.ppy,
            coeffs=list(intr.coeffs), model=str(intr.model)), indent=2))
    finally:
        if started:
            pipeline.stop()
    after = arm_snapshot() if args.arm_port else None
    observations = []
    for frame in frames:
        valid = [row for row in frame["detections"] if row["depth_valid"]]
        if len(valid) == 1:
            observations.append(valid[0])
    stable = len(observations) >= 3 and len({row["class_id"] for row in observations}) == 1
    summary = dict(camera=args.camera, serial=SERIALS[args.camera], frames=frames,
                   single_target_observations=len(observations), stable_depth_target=False,
                   arm_before=before, arm_after=after, robot_motion_commands_sent=0)
    if stable:
        points = np.array([row["camera_surface_xyz_m"] for row in observations])
        point = np.median(points, axis=0)
        spread = float(np.linalg.norm(points-point, axis=1).max())
        summary.update(camera_surface_xyz_m=point.tolist(), temporal_spread_m=spread,
                       class_name=observations[0]["class_name"], stable_depth_target=spread <= .02)
        if before is not None:
            drift = max(abs(a-b) for a, b in zip(before["position_deg"], after["position_deg"]))
            summary["arm_drift_deg"] = drift
            transform = camera_transform(args.urdf, args.camera, after["position_deg"])
            summary["cad_base_link_surface_xyz_m"] = (transform @ np.r_[point, 1])[:3].tolist()
            summary["extrinsic_status"] = "CAD transform; hardware calibration pending"
            summary["stable_depth_target"] &= drift <= .5
    (args.out/"result.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps({key: value for key, value in summary.items() if key != "frames"}, indent=2))


if __name__ == "__main__":
    main()
