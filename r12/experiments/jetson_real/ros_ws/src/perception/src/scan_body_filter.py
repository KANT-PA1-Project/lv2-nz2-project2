#!/usr/bin/env python3
"""라이다 4개 몸체 가림 필터 [2026-11-02]

왜: 실물 LDS-03 은 360° 를 다 내보냄 → 모서리 라이다가 로봇 자기 몸(차체·바퀴·브래킷)을 보고
    그 점이 지도에 "로봇을 따라다니는 장애물"로 찍혀 지도·위치 추정이 깨짐. (Gazebo 는 미리 270° 로 잘라서 안 보였음)
방법: 각 빔 끝점을 base_link 좌표로 바꿔서 로봇 몸 상자(축 정렬 사각형) 안이면 그 빔을 지움(NaN = 무효).
      라이다가 어떤 각도로 붙어 있든 tf(base_link → 라이다)만 맞으면 됨 → 캘리브레이션 결과가 그대로 반영됨
      지운 빔은 카토그래퍼가 아예 안 씀 (range_min~range_max 밖 → 버림, "빈 공간"으로 칠하지도 않음)

입력 → 출력 (기본, config/scan_filter.yaml 에서 변경):
    /scan/front_right_raw → /scan/front_right      (라이다 드라이버는 *_raw 로 내보내게 remap)
    /scan/front_left_raw  → /scan/front_left
    /scan/rear_left_raw   → /scan/rear_left
    /scan/rear_right_raw  → /scan/rear_right
    → 카토그래퍼 launch(slam / localization)는 그대로 /scan/<이름> 을 씀

파라미터:
    inputs / outputs        : 토픽 목록 (같은 순서)
    base_frame              : "base_link"
    body_box                : [x_min, x_max, y_min, y_max] (m, base_link 기준) — 기본 차체 400×400 + 사방 3 cm 여유
    min_range               : 이보다 가까운 빔도 지움 (m)
    extra_drop_deg          : 라이다별 추가로 지울 각도 구간 [시작°, 끝°, …] (라이다 자기 좌표, 선택. 예: 케이블·기둥)
    stats_period_s          : 몇 초마다 라이다별 지운 비율 로그 (0 = 안 함)

실행 (Jetson, 라이다 드라이버 4개가 *_raw 로 나오는 상태):
    ros2 launch perception scan_filter.launch.py
    ros2 run perception scan_body_filter.py --ros-args --params-file <scan_filter.yaml>
안전: tf(base_link → 라이다)가 없으면 그 라이다는 내보내지 않음(엉뚱한 점이 지도에 들어가는 것보다 나음) + 경고
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

DEFAULT_IN = ["/scan/front_right_raw", "/scan/front_left_raw", "/scan/rear_left_raw", "/scan/rear_right_raw"]
DEFAULT_OUT = ["/scan/front_right", "/scan/front_left", "/scan/rear_left", "/scan/rear_right"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_mask(ranges, angles, pose, box, min_range):
    """지울 빔 = True. pose = 라이다의 base_link 기준 (x, y, yaw)"""
    x0, y0, th = pose
    a = angles + th
    px = x0 + ranges * np.cos(a)
    py = y0 + ranges * np.sin(a)
    inside = (px > box[0]) & (px < box[1]) & (py > box[2]) & (py < box[3])
    return inside | (ranges < min_range)


class ScanBodyFilter(Node):
    def __init__(self):
        super().__init__("scan_body_filter")
        d = self.declare_parameter
        self.inputs = list(d("inputs", DEFAULT_IN).value)
        self.outputs = list(d("outputs", DEFAULT_OUT).value)
        if len(self.inputs) != len(self.outputs):
            raise ValueError("inputs 와 outputs 개수가 달라요")
        self.base = d("base_frame", "base_link").value
        self.box = [float(v) for v in d("body_box", [-0.23, 0.23, -0.23, 0.23]).value]
        if len(self.box) != 4 or self.box[0] >= self.box[1] or self.box[2] >= self.box[3]:
            raise ValueError(f"body_box 는 [x_min, x_max, y_min, y_max]: {self.box}")
        self.min_range = float(d("min_range", 0.05).value)
        drop = [float(v) for v in d("extra_drop_deg", [0.0]).value]
        self.drop = [(math.radians(drop[i]), math.radians(drop[i + 1])) for i in range(0, len(drop) - 1, 2)]
        self.stats_period = float(d("stats_period_s", 10.0).value)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.pose = {}                 # frame_id → (x, y, yaw)  (정적이라 한 번만 찾음)
        self.cache = {}                # (topic, n, amin, ainc) → angles
        self.stat = {t: [0, 0] for t in self.inputs}
        self.pubs = {}
        for i, o in zip(self.inputs, self.outputs):
            self.pubs[i] = self.create_publisher(LaserScan, o, qos_profile_sensor_data)
            self.create_subscription(LaserScan, i, lambda m, t=i: self.on_scan(m, t), qos_profile_sensor_data)
        if self.stats_period > 0:
            self.create_timer(self.stats_period, self.log_stats)
        self.get_logger().info(f"몸체 필터: {len(self.inputs)}개 라이다, 몸 상자 {self.box} m ({self.base}), "
                               f"최소 거리 {self.min_range} m, 추가 각도 {drop if self.drop else '없음'}")

    def lidar_pose(self, frame):
        if frame in self.pose:
            return self.pose[frame]
        try:
            tf = self.buf.lookup_transform(self.base, frame, Time())
        except Exception as e:
            self.get_logger().warning(f"tf {self.base} → {frame} 없음 → 이 라이다 안 내보냄 ({e.__class__.__name__})",
                                      throttle_duration_sec=5.0)
            return None
        t = tf.transform
        p = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        q = t.rotation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        if abs(roll) > 90:
            self.get_logger().warning(f"{frame}: 뒤집혀 장착(roll {roll:.0f}°) → 각도 방향 반대일 수 있음, 확인 필요")
        self.pose[frame] = p
        self.get_logger().info(f"{frame}: base_link 기준 x {p[0]:+.3f} y {p[1]:+.3f} yaw {math.degrees(p[2]):+.1f}°")
        return p

    def on_scan(self, m, topic):
        pose = self.lidar_pose(m.header.frame_id)
        if pose is None:
            return
        r = np.asarray(m.ranges, dtype=np.float32)
        key = (topic, len(r), m.angle_min, m.angle_increment)
        ang = self.cache.get(key)
        if ang is None:
            ang = (m.angle_min + m.angle_increment * np.arange(len(r))).astype(np.float32)
            self.cache[key] = ang
        rr = np.where(np.isfinite(r), r, np.inf)          # inf/NaN 은 계산에서 제외(멀리 있는 걸로)
        cut = body_mask(rr, ang, pose, self.box, self.min_range) & np.isfinite(r)
        for a0, a1 in self.drop:
            aw = np.arctan2(np.sin(ang), np.cos(ang))
            cut |= (aw >= a0) & (aw <= a1) if a0 <= a1 else ((aw >= a0) | (aw <= a1))
        out = LaserScan()
        out.header = m.header
        out.angle_min, out.angle_max, out.angle_increment = m.angle_min, m.angle_max, m.angle_increment
        out.time_increment, out.scan_time = m.time_increment, m.scan_time
        out.range_min, out.range_max = m.range_min, m.range_max
        r2 = r.copy()
        r2[cut] = np.nan
        out.ranges = r2.tolist()
        out.intensities = list(m.intensities)
        self.pubs[topic].publish(out)
        s = self.stat[topic]
        s[0] += int(cut.sum())
        s[1] += len(r)

    def log_stats(self):
        parts = []
        for t, (c, n) in self.stat.items():
            parts.append(f"{t.split('/')[-1]} {100 * c / n:.0f}%" if n else f"{t.split('/')[-1]} 입력없음")
            self.stat[t] = [0, 0]
        self.get_logger().info("지운 빔 비율: " + ", ".join(parts) + "  (실물 360° 모서리 라이다는 약 15~30%, Gazebo 는 이미 잘려서 약 0%)")


def main():
    rclpy.init()
    n = ScanBodyFilter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
