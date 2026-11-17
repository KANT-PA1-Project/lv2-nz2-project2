"""Additional actual sensor diagnostics over the existing owned local ROS child.

Images retain capture time, lidar retains its own current native epoch. These
messages never authorize motion or turn archived image data into current data.
"""
import hashlib
import math
import secrets
import time
import numpy as np
from .renderer_epoch_issuer import canonical

LIDARS = ("front_right", "rear_left", "low_front_left", "low_rear_right")


def seal(payload):
    payload = dict(payload)
    payload.pop("payload_sha256", None)
    payload["payload_sha256"] = hashlib.sha256(canonical(payload)).hexdigest()
    return payload


def make_payload(sample, schema, *, lidar, lidar_step, read_start_ns, read_end_ns, sequence):
    body = sample["certificate"]["body"]
    clouds = {}
    for name, spec in schema.config["depth"].items():
        camera = spec["rgb_camera"]
        depth, rgb = sample["depths"][name], sample["pixels"][camera]
        if list(depth.shape)!=schema.descriptor(name,"depth")["shape"] or list(rgb.shape)!=schema.descriptor(camera,"rgb")["shape"]:
            raise ValueError("coverage_raw_shape_mismatch")
        fx, fy, cx, cy = schema.config["cameras"][camera]["intrinsics"]
        v, u = np.mgrid[0:depth.shape[0]:64, 0:depth.shape[1]:64]
        z = depth[::64, ::64]
        valid = np.isfinite(z) & (z > .05) & (z < 5.)
        with np.errstate(invalid="ignore"):
            points = np.stack(((u-cx)*z/fx, (v-cy)*z/fy, z), axis=-1)[valid].astype("<f4")
        colors = rgb[::64, ::64][valid].astype("uint32")
        packed = (colors[:,0] << 16) | (colors[:,1] << 8) | colors[:,2]
        clouds[name] = dict(frame=camera, xyz=points.tolist(), rgb=packed.tolist(), stride=64)
    return seal(dict(kind="sensor_coverage", run_id=body["run_id"],
        source_sha256=body["source_sha256"], sequence=sequence, nonce=secrets.token_hex(16),
        capture_stamp_ns=body["capture_physics_step"]*10**9//960,
        lidar_step=lidar_step, lidar_stamp_ns=lidar_step*10**9//960,
        lidar_read_start_ns=read_start_ns, lidar_read_end_ns=read_end_ns,
        clouds=clouds, lidar=lidar, motion_authorized=False))


def validate(payload, *, run_id, source, schema, sequence, seen, now_ns):
    if payload.get("run_id") != run_id or payload.get("source_sha256") != source:
        raise ValueError("coverage_source_or_run_mismatch")
    nonce = payload.get("nonce")
    if type(nonce) is not str or len(nonce) != 32 or any(c not in "0123456789abcdef" for c in nonce):
        raise ValueError("coverage_invalid_nonce")
    if nonce in seen:
        raise ValueError("coverage_replayed_nonce")
    if type(payload.get("sequence")) is not int or payload["sequence"] != sequence + 1:
        raise ValueError("coverage_sequence_mismatch")
    if payload.get("payload_sha256") != seal(payload)["payload_sha256"]:
        raise ValueError("coverage_payload_hash_mismatch")
    start, end = payload["lidar_read_start_ns"], payload["lidar_read_end_ns"]
    if type(start) is not int or type(end) is not int or not 0 <= start <= end <= now_ns or now_ns-start > 100_000_000:
        raise ValueError("coverage_lidar_stale_or_invalid_time")
    step = payload["lidar_step"]
    if type(step) is not int or step < 0 or payload["lidar_stamp_ns"] != step*10**9//960:
        raise ValueError("coverage_lidar_epoch_mismatch")
    if not 0 <= payload["capture_stamp_ns"] <= payload["lidar_stamp_ns"]:
        raise ValueError("coverage_capture_current_order")
    if set(payload["clouds"]) != set(schema.config["depth"]) or set(payload["lidar"]) != set(LIDARS):
        raise ValueError("coverage_sensor_set_mismatch")
    for name, cloud in payload["clouds"].items():
        xyz, rgb = np.asarray(cloud["xyz"], dtype="<f4").reshape(-1,3), np.asarray(cloud["rgb"])
        if cloud["frame"] != schema.config["depth"][name]["rgb_camera"] or cloud["stride"] != 64:
            raise ValueError("coverage_cloud_calibration_mismatch")
        if len(xyz) != len(rgb) or len(xyz) > 1000 or not np.isfinite(xyz).all():
            raise ValueError("coverage_cloud_shape")
        if len(rgb) and (not np.issubdtype(rgb.dtype, np.integer) or (rgb<0).any() or (rgb>0xffffff).any()):
            raise ValueError("coverage_cloud_rgb")
    for distances in payload["lidar"].values():
        if len(distances) != 90 or any(not math.isfinite(x) or not 0 <= x <= 6. for x in distances):
            raise ValueError("coverage_lidar_ranges")
    if payload.get("motion_authorized") is not False:
        raise ValueError("coverage_never_authorizes_motion")


class CoverageEndpoint:
    def __init__(self, pubnode, subnode, executor, qos, *, schema, run_id, source, output):
        from sensor_msgs.msg import PointCloud2, PointField, LaserScan
        self.PointCloud2, self.PointField, self.LaserScan = PointCloud2, PointField, LaserScan
        self.executor, self.schema, self.run_id, self.source, self.output = executor, schema, run_id, source, output
        self.sequence, self.seen, self.records = 0, set(), []
        self.expected, self.received, self.errors = {}, {}, []
        self.publishers, self.subscriptions = {}, []
        prefix = "/r12/native_diagnostic/" + run_id + "/coverage/"
        topics = [(name + "/" + kind, PointCloud2) for name in schema.config["depth"] for kind in ("xyz","xyzrgb")]
        topics += [("lidar/" + name, LaserScan) for name in LIDARS]
        for name, kind in topics:
            self.publishers[name] = pubnode.create_publisher(kind, prefix+name, qos)
            self.subscriptions.append(subnode.create_subscription(
                kind, prefix+name, lambda message, key=name: self.observe(key, message), qos))
        deadline = time.monotonic() + 10
        while not all(pub.get_subscription_count() for pub in self.publishers.values()):
            if time.monotonic() > deadline:
                raise RuntimeError("coverage_ROS_discovery_timeout")
            executor.spin_once(timeout_sec=.01)

    def observe(self, key, message):
        stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
        if isinstance(message, self.PointCloud2):
            actual = dict(stamp_ns=stamp, frame=message.header.frame_id,
                width=message.width, height=message.height, point_step=message.point_step,
                row_step=message.row_step, is_dense=message.is_dense, is_bigendian=message.is_bigendian,
                fields=[(f.name,f.offset,f.datatype,f.count) for f in message.fields],
                sha256=hashlib.sha256(bytes(message.data)).hexdigest())
        else:
            actual = dict(stamp_ns=stamp, frame=message.header.frame_id,
                ranges=list(message.ranges), angle_min=message.angle_min,
                angle_max=message.angle_max, angle_increment=message.angle_increment)
        if key not in self.expected or actual != self.expected[key] or key in self.received:
            self.errors.append("coverage_modified_unexpected_or_duplicate:" + key)
        else:
            self.received[key] = actual

    def handle(self, payload):
        import json
        from builtin_interfaces.msg import Time
        try:
            validate(payload, run_id=self.run_id, source=self.source, schema=self.schema,
                     sequence=self.sequence, seen=self.seen, now_ns=time.monotonic_ns())
        except (ValueError, KeyError, TypeError) as error:
            result = dict(kind="sensor_coverage_rejected", reason=str(error),
                nonce=payload.get("nonce"), payload_sha256=payload.get("payload_sha256"),
                sequence=payload.get("sequence"), published_samples=0)
            self.records.append(result)
            (self.output/"coverage_results.json").write_text(json.dumps(self.records, indent=2))
            return result
        self.sequence = payload["sequence"]
        self.seen.add(payload["nonce"])
        self.expected, self.received, self.errors = {}, {}, []
        pending = []
        for name, cloud in payload["clouds"].items():
            xyz = np.asarray(cloud["xyz"], dtype="<f4").reshape(-1,3)
            for kind in ("xyz", "xyzrgb"):
                width = len(xyz)
                message = self.PointCloud2()
                stamp = payload["capture_stamp_ns"]
                message.header.stamp = Time(sec=stamp//10**9,nanosec=stamp%10**9)
                message.header.frame_id = cloud["frame"]
                message.height, message.width = 1, width
                message.is_bigendian, message.is_dense = False, True
                fields = [("x",0,7,1),("y",4,7,1),("z",8,7,1)]
                if kind == "xyzrgb":
                    structured = np.empty(width, dtype=[("xyz","<f4",(3,)),("rgb","<u4")])
                    structured["xyz"], structured["rgb"] = xyz, cloud["rgb"]
                    raw, size = structured.tobytes(), 16
                    fields.append(("rgb",12,6,1))
                else:
                    raw, size = xyz.tobytes(), 12
                message.fields = [self.PointField(name=n,offset=o,datatype=d,count=c) for n,o,d,c in fields]
                message.point_step, message.row_step, message.data = size, width*size, raw
                key = name+"/"+kind
                self.expected[key] = dict(stamp_ns=stamp,frame=cloud["frame"],width=width,height=1,
                    point_step=size,row_step=width*size,is_dense=True,is_bigendian=False,
                    fields=fields,sha256=hashlib.sha256(raw).hexdigest())
                pending.append((key,message))
        for name, distances in payload["lidar"].items():
            message = self.LaserScan()
            stamp = payload["lidar_stamp_ns"]
            message.header.stamp = Time(sec=stamp//10**9,nanosec=stamp%10**9)
            message.header.frame_id = "lidar_"+name
            message.angle_min, message.angle_max = float(np.float32(-np.pi/2)), float(np.float32(np.pi/2))
            message.angle_increment = float(np.float32(np.pi/89))
            message.range_min, message.range_max = 0., 6.
            message.ranges = np.asarray(distances,dtype=np.float32).tolist()
            key = "lidar/"+name
            self.expected[key] = dict(stamp_ns=stamp,frame=message.header.frame_id,
                ranges=list(message.ranges),angle_min=message.angle_min,angle_max=message.angle_max,
                angle_increment=message.angle_increment)
            pending.append((key,message))
        for key, message in pending:
            self.publishers[key].publish(message)
        deadline = time.monotonic()+1
        while len(self.received)<12 and not self.errors and time.monotonic()<deadline:
            self.executor.spin_once(timeout_sec=.002)
        result = dict(kind="sensor_coverage_result", sequence=self.sequence, nonce=payload["nonce"],
            payload_sha256=payload["payload_sha256"], all_received=len(self.received)==12 and not self.errors,
            expected=12, received=len(self.received), errors=self.errors,
            topic_names=sorted(self.received), source_sha256=self.source,
            capture_stamp_ns=payload["capture_stamp_ns"], lidar_stamp_ns=payload["lidar_stamp_ns"],
            motion_authorized=False, scope="actual derived cloud and current raycast ROS receipt")
        self.records.append(result)
        (self.output/"coverage_results.json").write_text(json.dumps(self.records,indent=2))
        return result
