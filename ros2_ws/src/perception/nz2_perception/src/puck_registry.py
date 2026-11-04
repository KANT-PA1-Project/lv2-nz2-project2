#!/usr/bin/env python3
# =====================================================================
# puck_registry  (NZ2 인지) — 퍽 등록·추적
#   역할: YOLO 퍽 박스 + AprilTag ID(0~11) + (선택) depth·tf → 퍽마다 ID·색·모양·자세·중심 위치
#         3프레임 연속 확인된 퍽만 내보내고, 태그 ID 별 등록표를 파일로 남김 (재검증 근거)
#   입력: /perception/detections  (vision_msgs/Detection2DArray, yolo_detector)
#         /nz2/tags              (vision_msgs/Detection3DArray, tag_reader, 카메라 광학 프레임)
#         /gripper/color/camera_info
#         /gripper/aligned_depth_to_color/image_raw   (선택, 16UC1 mm / 32FC1 m)
#         /tf                    (선택: base_frame 이 있으면 자세 판정, track_frame 이 있으면 이동 중 추적)
#   출력: /perception/puck_tracks (nz2_interfaces/PuckArray) — 확정 트랙, frame = track_frame (없으면 카메라)
#         /nz2/puck_registry      (std_msgs/String JSON, transient_local) — {tag_id: 색·모양·자세·관측수·위치}
#         ~/nz2_logs/puck_registry.json (+ 변경 기록 puck_registry_log.jsonl)
#         /perception/puck_markers    (visualization_msgs/MarkerArray) — RViz: 퍽 모양·색·자세 + 중심점 + "ID 색 모양 자세 (x,y,z)"
#         /perception/puck_debug_image (sensor_msgs/Image bgr8) — 영상 위에 YOLO 박스, 퍽 중심(투영 ●), 연결된 태그(□),
#                                       확정 퍽의 3D 위치(mm). 확인: ros2 run rqt_image_view rqt_image_view /perception/puck_debug_image
#
#   규칙 (미션 v5)
#     - 원통은 항상 서 있음 → YOLO 가 cyl_*_down 이라고 하면 box_*_down 으로 바꿈
#     - 신호등 박스와 겹치는 퍽 박스는 버림 (v6 혼동: 초록 원통 12% 를 신호등 근처에서 헷갈림)
#     - 태그 연결: 태그 중심(3D → 영상 투영)이 퍽 박스 안에 있는 것. 다른 ID 가 둘 이상이면 가까운 쪽이
#       확실할 때만 연결, 아니면 미연결(ambiguous). 같은 태그가 여러 박스에 걸리면 둘 다 미연결
#     - 중심: 태그 면에서 안쪽으로 15 mm(옆면) 또는 30 mm(끝면). 어느 쪽인지는 depth 추정 중심에 가까운 쪽
#     - 자세(직육면체): tf 로 '위' 방향을 알면 퍽 윗면 높이로 판정 (서있음 130 mm / 누움 100 mm, 경계 115)
#     - [2026-10-30] 위치 기준 = map (없으면 odom). 시작할 때 등록표 비움 (지난 판 = puck_registry_prev.json)
#     - [2026-10-30] 태그 오독 대비: 트랙 연결은 위치 먼저(같은 색 30 mm 안), ID 는 투표(3번 이상·70% 이상)로 확정 →
#       한 번 잘못 읽어도 ID 안 바뀜. 같은 ID 가 10 cm 넘게 떨어진 곳에서 읽히면 따로 추적, 동시에 두 곳이면
#       등록표에 "conflict" (위치들·득표율) 표시하고 득표율 높은 쪽만 등록 → 미션 노드가 가서 다시 확인
#
#   ※ 파이썬 노드 기본 구조는 signal_recorder.py 와 같음 (rclpy.Node 상속 → 구독 콜백 → 발행)
# =====================================================================
import json
import math
import os
import time
from collections import Counter, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from nz2_interfaces.msg import Puck, PuckArray

try:                                   # tf 는 선택 (없어도 카메라 좌표로 동작)
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.time import Time
    HAVE_TF = True
except ImportError:                    # pragma: no cover
    HAVE_TF = False

try:                                   # 디버그 영상용 (없으면 영상만 생략)
    import cv2
    HAVE_CV = True
except ImportError:                    # pragma: no cover
    HAVE_CV = False

PUCK_W, PUCK_H = 0.030, 0.060          # 퍽 단면·높이 (m)
COLOR = {"red": Puck.COLOR_RED, "green": Puck.COLOR_GREEN}
SHAPE = {"cyl": Puck.SHAPE_CYLINDER, "box": Puck.SHAPE_BOX}
POSTURE = {"up": Puck.POSTURE_UP, "down": Puck.POSTURE_DOWN}


def parse_class(name):
    """'box_red_down' → ('box','red','down'). 퍽이 아니면 None"""
    p = name.split("_")
    if len(p) != 3 or p[0] not in SHAPE or p[1] not in COLOR or p[2] not in POSTURE:
        return None
    return p[0], p[1], p[2]


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class Track:
    """퍽 1개의 추적 상태"""
    _next = 0

    def __init__(self, now):
        Track._next += 1
        self.id = Track._next
        self.tag_id = -1
        self.pos = None              # track_frame 좌표 (np 3)
        self.hits = 0                # 연속 관측 수 (max_gap_s 넘게 끊기면 1 부터)
        self.total = 0
        self.first = now
        self.last = now
        self.votes = {"color": Counter(), "shape": Counter(), "posture": Counter()}
        self.conf = 0.0
        self.confirmed = False       # 한 번이라도 confirm_hits 연속 관측되면 True (hold_s 동안 출력 유지)
        self.id_votes = Counter()    # [2026-10-30] 태그 ID 투표 (한 번 잘못 읽어도 ID 가 바로 안 바뀌게)

    def vote(self, key):
        c = self.votes[key]
        return c.most_common(1)[0][0] if c else None

    def decide_id(self, min_votes, min_ratio):
        """ID 확정: 최다 득표가 min_votes 이상이고 전체의 min_ratio 이상일 때만. 아니면 -1 (모름)"""
        if not self.id_votes:
            return -1
        best, n = self.id_votes.most_common(1)[0]
        return best if n >= min_votes and n / sum(self.id_votes.values()) >= min_ratio else -1

    def id_conf(self):
        n = sum(self.id_votes.values())
        return round(self.id_votes[self.tag_id] / n, 2) if n and self.tag_id >= 0 else 0.0


class PuckRegistry(Node):
    def __init__(self):
        super().__init__("puck_registry")
        d = self.declare_parameter
        self.det_topic = d("detections_topic", "/perception/detections").value
        self.tag_topic = d("tags_topic", "/nz2/tags").value
        self.info_topic = d("info_topic", "/gripper/color/camera_info").value
        self.depth_topic = d("depth_topic", "/gripper/aligned_depth_to_color/image_raw").value
        self.out_topic = d("output_topic", "/perception/puck_tracks").value
        self.reg_topic = d("registry_topic", "/nz2/puck_registry").value
        self.conf_min = d("conf_min", 0.35).value              # v6 최고 F1 근처
        self.puck_ids = list(d("puck_tag_ids", list(range(12))).value)
        self.tower_iou = d("tower_iou", 0.15).value            # 신호등 박스와 이만큼 겹치면 퍽 아님
        self.tag_margin = d("tag_margin_px", 4.0).value        # 박스 경계 여유
        self.sync_tol = d("sync_tol_s", 0.08).value            # 검출·태그·depth 시각 차 허용
        self.max_input_age = d("max_input_age_s", 1.0).value   # 이보다 오래된 태그·depth 는 안 씀 (0 = 검사 안 함, bag 재생용)
        self.confirm_hits = d("confirm_hits", 3).value
        self.max_gap = d("max_gap_s", 0.3).value               # 이보다 오래 안 보이면 연속 수 초기화
        self.hold = d("hold_s", 1.0).value                     # 마지막 관측 후 이만큼 출력 유지
        self.assoc_dist = d("assoc_dist_m", 0.03).value        # ID 없는 퍽: 이 거리 안이면 같은 퍽
        self.track_frame = d("track_frame", "map").value       # [2026-10-30] 지도 기준으로 저장 (나중에 찾아갈 위치)
        self.track_fallback = d("track_frame_fallback", "odom").value   # map tf 가 없을 때 (SLAM 안 켬)
        self.clear_on_start = d("clear_on_start", True).value   # [2026-10-30] 시작할 때 등록표 비움 (매 판 새로 탐색)
        self.id_min_votes = d("id_min_votes", 3).value          # [2026-10-30] 같은 ID 를 이만큼 읽어야 확정
        self.id_min_ratio = d("id_min_ratio", 0.7).value        #   그 트랙에서 읽힌 ID 중 이 비율 이상
        self.relocate_dist = d("relocate_dist_m", 0.10).value   #   같은 ID 트랙이 이보다 멀면 같은 퍽으로 안 봄 (오독 의심)
        self.dup_dist = d("duplicate_dist_m", 0.05).value       #   같은 ID 가 동시에 이보다 떨어진 두 곳 → 충돌
        self.base_frame = d("base_frame", "base_link").value   # '위' 방향·바닥 높이 기준 ("" = 자세 판정 안 함)
        self.floor_z = d("floor_z", 0.0).value                 # base_frame 에서 바닥 높이 (m)
        self.box_top = d("box_top_z", 0.070).value             # 박스 윗면 = 퍽 바닥 (m)
        self.posture_split = d("posture_split_m", 0.045).value  # 퍽 윗면 - 박스 윗면 > 이 값 → 서있음 (60 vs 30 mm)
        self.registry_path = os.path.expanduser(d("registry_path", "~/nz2_logs/puck_registry.json").value)
        self.min_reg_obs = d("registry_min_obs", 3).value      # 이 횟수 이상 본 트랙만 등록
        self.pub_markers = d("publish_markers", True).value     # RViz 마커
        self.marker_topic = d("marker_topic", "/perception/puck_markers").value
        self.pub_debug = d("publish_debug_image", True).value   # 디버그 영상 (그리퍼 컬러 영상 구독 → CPU 사용)
        self.image_topic = d("image_topic", "/gripper/color/image_raw").value
        self.debug_topic = d("debug_topic", "/perception/puck_debug_image").value

        img_qos = qos_profile_sensor_data
        self.create_subscription(Detection2DArray, self.det_topic, self.on_det, 10)
        self.create_subscription(Detection3DArray, self.tag_topic, self.on_tags, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, img_qos)
        self.create_subscription(Image, self.depth_topic, self.on_depth, img_qos)
        self.pub = self.create_publisher(PuckArray, self.out_topic, 10)
        self.mk_pub = self.create_publisher(MarkerArray, self.marker_topic, 10) if self.pub_markers else None
        self.dbg_pub = None
        self.images = deque(maxlen=5)
        if self.pub_debug and HAVE_CV:
            self.dbg_pub = self.create_publisher(Image, self.debug_topic, 5)
            self.create_subscription(Image, self.image_topic, lambda m: self.images.append(m), img_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.reg_pub = self.create_publisher(String, self.reg_topic, latched)

        self.tags = deque(maxlen=15)       # 최근 태그 메시지
        self.depths = deque(maxlen=6)      # 최근 depth (stamp, np)
        self.K = None
        self.tracks = []
        self.registry = {} if self.clear_on_start else self._load_registry()
        if self.clear_on_start:
            self._backup_registry()                      # 지난 판 등록표는 _prev 로 남김
        self.conflicts = {}                              # [2026-10-30] tag_id → 충돌 정보 (등록표에 같이 기록)
        self._reg_dirty = False
        self._last_save = 0.0
        if HAVE_TF:
            self.tf_buf = tf2_ros.Buffer()
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        try:
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "~/reset", self.on_reset)
        except ImportError:
            pass
        self.create_timer(1.0, self._save_registry)
        self.get_logger().info(
            f"퍽 등록 시작: {self.det_topic} + {self.tag_topic} → {self.out_topic}, 등록표 {self.registry_path} "
            f"(기존 {len(self.registry)}개, 시작 시 비움={self.clear_on_start}, 기준 {self.track_frame}→{self.track_fallback})")

    # ---------------------------------------------------------------- 입력 보관
    def on_info(self, m):
        if m.k[0] > 0:
            self.K = np.array(m.k, dtype=float).reshape(3, 3)

    def on_tags(self, m):
        self.tags.append(m)

    def on_depth(self, m):
        try:
            if m.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width] * 0.001
            elif m.encoding == "32FC1":
                arr = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.step // 4)[:, :m.width]
            else:
                self.get_logger().warning(f"depth 인코딩 {m.encoding} 미지원", throttle_duration_sec=10.0)
                return
        except ValueError:
            return
        self.depths.append((stamp_s(m.header.stamp), arr.astype(np.float32)))

    def on_reset(self, req, res):
        self.tracks.clear()
        self.registry.clear()
        self.conflicts.clear()
        self._reg_dirty = True
        self._save_registry(force=True)
        res.success, res.message = True, "트랙·등록표 초기화"
        return res

    # ---------------------------------------------------------------- 도우미
    def _nearest(self, buf, t, key=lambda x: x):
        best, bd = None, 1e9
        for item in buf:
            s = key(item)
            if abs(s - t) < bd:
                best, bd = item, abs(s - t)
        return (best, bd) if best is not None and bd <= self.sync_tol else (None, bd)

    def _tf(self, target, source, stamp):
        """source → target 4x4 행렬. 실패하면 None"""
        if not HAVE_TF or not target or target == source:
            return np.eye(4) if target == source else None
        try:
            tr = self.tf_buf.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.02))
        except Exception:
            try:      # 시각이 안 맞으면 최신 값으로 (정지 상태 가정)
                tr = self.tf_buf.lookup_transform(target, source, Time())
            except Exception:
                return None
        q, t = tr.transform.rotation, tr.transform.translation
        M = np.eye(4)
        M[:3, :3] = quat_to_R(q)
        M[:3, 3] = [t.x, t.y, t.z]
        return M

    def _depth_center(self, depth, box):
        """박스 가운데 50% 영역 depth 중앙값 → 보이는 표면 점 → 시선 방향으로 15 mm 더 들어간 점 (대략 중심)"""
        if depth is None or self.K is None:
            return None, None
        h, w = depth.shape
        x1, y1, x2, y2 = box
        cx, cy, bw, bh = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
        u0, u1 = int(max(0, cx - bw / 4)), int(min(w, cx + bw / 4 + 1))
        v0, v1 = int(max(0, cy - bh / 4)), int(min(h, cy + bh / 4 + 1))
        roi = depth[v0:v1, u0:u1]
        z = roi[(roi > 0.05) & (roi < 3.0) & np.isfinite(roi)]
        if z.size < 10:
            return None, None
        zm = float(np.median(z))
        fx, fy, ppx, ppy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        ray = np.array([(cx - ppx) / fx, (cy - ppy) / fy, 1.0])
        surf = ray * zm
        center = surf + ray / np.linalg.norm(ray) * (PUCK_W / 2)
        # 윗면 높이 계산용: 박스 전체 영역 점들 (카메라 좌표)
        U0, U1, V0, V1 = int(max(0, x1)), int(min(w, x2 + 1)), int(max(0, y1)), int(min(h, y2 + 1))
        sub = depth[V0:V1, U0:U1]
        vv, uu = np.nonzero((sub > 0.05) & (sub < 3.0) & np.isfinite(sub))
        if vv.size < 10:
            return center, None
        zz = sub[vv, uu]
        pts = np.stack([((uu + U0) - ppx) / fx * zz, ((vv + V0) - ppy) / fy * zz, zz], axis=1)
        return center, pts

    # ---------------------------------------------------------------- 메인 처리
    def on_det(self, msg):
        now = time.monotonic()
        t_det = stamp_s(msg.header.stamp)
        cam_frame = msg.header.frame_id
        tag_msg, _ = self._nearest(self.tags, t_det, key=lambda m: stamp_s(m.header.stamp))
        dep, _ = self._nearest(self.depths, t_det, key=lambda x: x[0])
        if self.max_input_age > 0:
            wall = self.get_clock().now().nanoseconds * 1e-9
            if t_det > 1e6 and wall - t_det > self.max_input_age:
                return                        # 너무 오래된 검출 (지연) → 버림
        depth = dep[1] if dep else None

        # 1) 퍽 후보 / 신호등 박스
        pucks, towers = [], []
        for d in msg.detections:
            if not d.results:
                continue
            h = max(d.results, key=lambda r: r.hypothesis.score)
            name, score = h.hypothesis.class_id, h.hypothesis.score
            b = d.bbox
            box = (b.center.position.x - b.size_x / 2, b.center.position.y - b.size_y / 2,
                   b.center.position.x + b.size_x / 2, b.center.position.y + b.size_y / 2)
            if name == "tower_light":
                towers.append(box)
                continue
            cls = parse_class(name)
            if cls is None or score < self.conf_min:
                continue
            shape, color, posture = cls
            if shape == "cyl" and posture == "down":       # 미션: 원통은 항상 서 있음
                shape = "box"
            pucks.append({"box": box, "shape": shape, "color": color, "posture": posture, "score": score})
        dropped = [p for p in pucks if any(iou(p["box"], t) >= self.tower_iou for t in towers)]
        pucks = [p for p in pucks if p not in dropped]

        # 2) 태그 투영 → 박스 안 후보
        tags = []
        all_tags = []                     # [2026-10-30] 디버그 영상용: 보이는 태그 전부 (장소 12~14·퍽 박스 밖 태그 포함)
        if tag_msg is not None and self.K is not None and tag_msg.header.frame_id in ("", cam_frame):
            for td in tag_msg.detections:
                try:
                    tid = int(td.id)
                except ValueError:
                    continue
                if not td.results:
                    continue
                pp = td.results[0].pose.pose
                p = np.array([pp.position.x, pp.position.y, pp.position.z])
                if p[2] <= 0.02:
                    continue
                uvw = self.K @ p
                all_tags.append({"id": tid, "uv": uvw[:2] / uvw[2], "z": float(p[2])})
                if tid not in self.puck_ids or td.results[0].hypothesis.score < 0.99:
                    continue
                tags.append({"id": tid, "p": p, "R": quat_to_R(pp.orientation), "uv": uvw[:2] / uvw[2]})
        m = self.tag_margin
        for p in pucks:
            x1, y1, x2, y2 = p["box"]
            p["cands"] = [t for t in tags if x1 - m <= t["uv"][0] <= x2 + m and y1 - m <= t["uv"][1] <= y2 + m]
        use = Counter(id(t) for p in pucks for t in p["cands"])
        for p in pucks:
            p["tag"], p["assoc"] = None, "tag_not_visible"
            c = [t for t in p["cands"] if use[id(t)] == 1]          # 다른 박스와 공유된 태그는 뺌
            if p["cands"] and not c:
                p["assoc"] = "ambiguous_shared_tag"
                continue
            ids = {t["id"] for t in c}
            if len(ids) == 1:
                p["tag"] = c[0]                 # 같은 ID 가 여러 면에 보이면 아무거나 (같은 퍽)
                p["assoc"] = "matched"
            elif len(ids) > 1:                  # 서로 다른 ID: 박스 중심에 확실히 가까운 것만
                bc = np.array([(p["box"][0] + p["box"][2]) / 2, (p["box"][1] + p["box"][3]) / 2])
                ds = sorted((float(np.linalg.norm(t["uv"] - bc)), k) for k, t in enumerate(c))
                if ds[1][0] > 1.5 * ds[0][0] + 3:
                    p["tag"], p["assoc"] = c[ds[0][1]], "matched_nearest"
                else:
                    p["assoc"] = "ambiguous"

        # 3) '위' 방향 (카메라 좌표)
        up_cam, M_cb = None, None
        if self.base_frame:
            M_cb = self._tf(self.base_frame, cam_frame, msg.header.stamp)   # camera → base
            if M_cb is not None:
                up_cam = M_cb[:3, :3].T @ np.array([0, 0, 1.0])

        # 4) 퍽마다 중심·자세
        obs = []
        for p in pucks:
            dc, pts = self._depth_center(depth, p["box"])
            center, src = None, "none"
            t = p["tag"]
            if t is not None:
                n = t["R"][:, 2]                                      # 태그 바깥 법선 (카메라 쪽)
                n_up = float(n @ up_cam) if up_cam is not None else None
                # 자세 먼저: 위를 보는 태그면 그 높이로 바로 판정 (서있음 윗면 130 mm / 누운 직육면체 옆면 100 mm)
                if p["shape"] == "box" and n_up is not None and n_up > 0.8:
                    zt = (M_cb @ np.append(t["p"], 1))[2] - self.floor_z - self.box_top
                    if 0.015 < zt < 0.08:
                        p["posture"] = "up" if zt > self.posture_split else "down"
                        p["posture_src"] = "tag_height"
                # 중심 = 태그 면에서 안쪽으로 15 mm(직육면체 긴 옆면) 또는 30 mm(끝면·원통 윗면)
                if p["shape"] == "cyl":
                    ds = [PUCK_H / 2]
                elif n_up is not None and n_up > 0.8:                  # 위를 보는 면
                    ds = [PUCK_H / 2] if p["posture"] == "up" else [PUCK_W / 2]
                elif n_up is not None and abs(n_up) < 0.5 and p["posture"] == "up":
                    ds = [PUCK_W / 2]                                  # 서있는 직육면체의 옆면
                else:
                    ds = [PUCK_W / 2, PUCK_H / 2]                      # 누운 직육면체 옆·끝면 → depth 로 고름
                cands = [t["p"] - n * dd for dd in ds]
                if len(cands) == 1:
                    center = cands[0]
                elif dc is not None:
                    center = min(cands, key=lambda c: float(np.linalg.norm(c - dc)))
                else:
                    center = (cands[0] + cands[1]) / 2                 # 모르면 가운데 (최대 7.5 mm 오차)
                src = "tag"
            elif dc is not None:
                center, src = dc, "depth"
            ref = center if center is not None else dc
            if pts is not None and ref is not None:      # 박스 안 배경(벽·바닥) 점 제거: 퍽 중심 45 mm 안만
                pts = pts[np.linalg.norm(pts - ref, axis=1) < 0.045]
                if len(pts) < 10:
                    pts = None
            if p["shape"] == "box" and "posture_src" not in p and pts is not None and M_cb is not None:
                zb = (M_cb[:3, :3] @ pts.T).T[:, 2] + M_cb[2, 3] - self.floor_z
                ztop = float(np.percentile(zb, 95)) - self.box_top
                if 0.015 < ztop < 0.08:
                    p["posture"] = "up" if ztop > self.posture_split else "down"
                    p["posture_src"] = "depth_height"
            if center is None:
                continue
            p["center_cam"], p["src"] = center, src
            obs.append(p)

        # 5) 추적 (track_frame 으로 옮겨서)
        frame, M_tc = cam_frame, None
        for fr in (self.track_frame, self.track_fallback):      # map → odom → 카메라
            if fr:
                M_tc = self._tf(fr, cam_frame, msg.header.stamp)
                if M_tc is not None:
                    frame = fr
                    break
        if frame != getattr(self, "_frame_used", frame):        # 기준이 바뀌면 트랙 좌표가 안 맞음 → 새로
            self.get_logger().warning(f"추적 기준 바뀜 {self._frame_used} → {frame}: 트랙 초기화")
            self.tracks.clear()
        self._frame_used = frame
        if M_tc is None:
            M_tc = np.eye(4)
        for p in obs:
            pos = (M_tc @ np.append(p["center_cam"], 1))[:3]
            tid = p["tag"]["id"] if p["tag"] else -1
            # [2026-10-30] 위치 먼저: 퍽은 (집기 전엔) 안 움직임 → 가까운 같은 색 트랙 = 같은 퍽. ID 는 투표로
            near = [(float(np.linalg.norm(x.pos - pos)), x) for x in self.tracks
                    if x.pos is not None and x.vote("color") == p["color"]]
            near = [z for z in near if z[0] < self.assoc_dist]
            tr = min(near, key=lambda z: z[0])[1] if near else None
            if tr is None and tid >= 0:                           # 위치로 못 찾으면 같은 ID 트랙 (가까울 때만)
                same = [(float(np.linalg.norm(x.pos - pos)), x) for x in self.tracks
                        if x.tag_id == tid and x.pos is not None]
                if same:
                    d0, x0 = min(same, key=lambda z: z[0])
                    if d0 < self.relocate_dist:
                        tr = x0
                    else:
                        self.get_logger().warning(f"태그 {tid}: 기존 위치와 {d0 * 1000:.0f} mm 떨어진 곳에서 또 읽힘 "
                                               "→ 다른 퍽으로 따로 추적 (오독·연결 오류 의심)",
                                               throttle_duration_sec=2.0)
            if tr is None:
                tr = Track(now)
                self.tracks.append(tr)
            if tid >= 0:
                tr.id_votes[tid] += 1
                new_id = tr.decide_id(self.id_min_votes, self.id_min_ratio)
                if tr.tag_id >= 0 and new_id >= 0 and new_id != tr.tag_id:
                    self.get_logger().warning(f"트랙 {tr.id}: ID {tr.tag_id} → {new_id} (투표 {dict(tr.id_votes)})")
                tr.tag_id = new_id
            p["track"] = tr
            tr.hits = tr.hits + 1 if now - tr.last <= self.max_gap or tr.total == 0 else 1
            tr.total += 1
            tr.last = now
            tr.pos = pos if tr.pos is None else 0.6 * tr.pos + 0.4 * pos
            tr.conf = p["score"]
            tr.confirmed = tr.confirmed or tr.hits >= self.confirm_hits
            tr.votes["color"][p["color"]] += 1
            tr.votes["shape"][p["shape"]] += 1
            tr.votes["posture"][p["posture"]] += 2 if "posture_src" in p else 1   # 높이로 판정한 표는 2배
        self.tracks = [x for x in self.tracks if now - x.last <= self.hold]
        self._check_duplicates()

        # 6) 출력
        out = PuckArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame
        for tr in self.tracks:
            if not tr.confirmed:
                continue
            pk = Puck()
            pk.header = out.header
            pk.color = COLOR.get(tr.vote("color"), Puck.COLOR_UNKNOWN)
            pk.shape = SHAPE.get(tr.vote("shape"), Puck.SHAPE_UNKNOWN)
            pk.posture = POSTURE.get(tr.vote("posture"), Puck.POSTURE_UNKNOWN)
            pk.position.x, pk.position.y, pk.position.z = (float(v) for v in tr.pos)
            pk.yaw = float("nan")
            pk.confidence = float(tr.conf)
            pk.track_id = tr.id
            pk.tag_id = tr.tag_id
            out.pucks.append(pk)
            c = self.conflicts.get(tr.tag_id)
            if tr.tag_id >= 0 and tr.total >= self.min_reg_obs and (c is None or c["winner_track"] == tr.id):
                self._register(tr, frame)                    # 충돌 중이면 이긴 트랙만 등록 (위치가 왔다갔다 안 하게)
        self.pub.publish(out)
        if self.mk_pub is not None:
            self._publish_markers(out)
        if self.dbg_pub is not None:
            self._publish_debug(msg, pucks, dropped, towers, frame, all_tags)

    # ---------------------------------------------------------------- 시각화
    def _publish_markers(self, out):
        """RViz: 퍽마다 모양 마커(빨강/초록, 누우면 옆으로) + 중심 흰 점 + 글자"""
        arr = MarkerArray()
        clr = Marker()
        clr.header = out.header
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)
        rgb = {Puck.COLOR_RED: (0.9, 0.1, 0.1), Puck.COLOR_GREEN: (0.1, 0.8, 0.2)}
        name = {Puck.COLOR_RED: "red", Puck.COLOR_GREEN: "green"}
        for k, pk in enumerate(out.pucks):
            body = Marker()
            body.header = out.header
            body.ns, body.id = "puck", k
            body.type = Marker.CYLINDER if pk.shape == Puck.SHAPE_CYLINDER else Marker.CUBE
            body.action = Marker.ADD
            body.pose.position = pk.position
            if pk.posture == Puck.POSTURE_DOWN:          # 누움: 길이축을 옆으로 (방향 yaw 는 모름)
                body.pose.orientation.x, body.pose.orientation.w = 0.7071068, 0.7071068
            else:
                body.pose.orientation.w = 1.0
            body.scale.x, body.scale.y, body.scale.z = PUCK_W, PUCK_W, PUCK_H
            body.color.r, body.color.g, body.color.b = rgb.get(pk.color, (0.6, 0.6, 0.6))
            body.color.a = 0.6
            dot = Marker()
            dot.header, dot.ns, dot.id, dot.type, dot.action = out.header, "center", k, Marker.SPHERE, Marker.ADD
            dot.pose.position = pk.position
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.008
            dot.color.r = dot.color.g = dot.color.b = dot.color.a = 1.0
            txt = Marker()
            txt.header, txt.ns, txt.id, txt.type, txt.action = out.header, "label", k, Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y = pk.position.x, pk.position.y
            txt.pose.position.z = pk.position.z + 0.06
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.02
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            shape = "cyl" if pk.shape == Puck.SHAPE_CYLINDER else "box"
            post = {Puck.POSTURE_UP: "up", Puck.POSTURE_DOWN: "down"}.get(pk.posture, "?")
            txt.text = (f"ID {pk.tag_id if pk.tag_id >= 0 else '?'} {name.get(pk.color, '?')} {shape} {post}\n"
                        f"({pk.position.x:.3f}, {pk.position.y:.3f}, {pk.position.z:.3f})")
            arr.markers += [body, dot, txt]
        self.mk_pub.publish(arr)

    def _publish_debug(self, msg, pucks, dropped, towers, frame, all_tags=()):
        """영상 위: YOLO 박스(연결 상태별 색), 퍽 중심 투영 ●, 연결된 태그 □, 확정 트랙의 3D 위치
        [2026-10-30] 보이는 태그 전부에 큰 글씨로 ID (퍽 0~11 = 하늘색, 장소 12~14 = 분홍 L1~L3, 그 밖 = 회색)"""
        t_det = stamp_s(msg.header.stamp)
        im, dt = self._nearest(self.images, t_det, key=lambda m: stamp_s(m.header.stamp))
        if im is None and self.images:
            im = self.images[-1]                       # 시각이 안 맞으면 최신 영상 (표시용)
        if im is None:
            return
        try:
            a = np.frombuffer(im.data, dtype=np.uint8).reshape(im.height, im.step)[:, :im.width * 3]
            img = a.reshape(im.height, im.width, 3)
            img = np.ascontiguousarray(img[..., ::-1]) if im.encoding == "rgb8" else img.copy()
        except ValueError:
            return
        f = cv2.FONT_HERSHEY_SIMPLEX
        for tb in towers:
            cv2.rectangle(img, (int(tb[0]), int(tb[1])), (int(tb[2]), int(tb[3])), (0, 255, 255), 1)
            cv2.putText(img, "tower", (int(tb[0]), int(tb[1]) - 3), f, 0.4, (0, 255, 255), 1)
        for p in dropped:
            b = p["box"]
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (128, 128, 128), 1)
            cv2.putText(img, "drop:tower", (int(b[0]), int(b[3]) + 12), f, 0.4, (128, 128, 128), 1)
        col = {"matched": (0, 220, 0), "matched_nearest": (0, 220, 0), "tag_not_visible": (0, 165, 255)}
        for p in pucks:
            b = p["box"]
            c = col.get(p.get("assoc"), (0, 0, 255))              # 빨강 = 모호(태그 공유·여러 ID)
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), c, 2)
            tid = p["tag"]["id"] if p.get("tag") else "?"
            tr = p.get("track")
            if tid == "?" and tr is not None and tr.tag_id >= 0:
                tid = f"{tr.tag_id}*"                                 # 이번 프레임엔 태그 안 보임, 트랙이 기억하는 ID
            lab = f"ID {tid}  {p['shape']}_{p['color']}_{p['posture']} {p['score']:.2f}"
            org = (int(b[0]), max(16, int(b[1]) - 6))
            cv2.putText(img, lab, org, f, 0.55, (0, 0, 0), 3)          # 검은 테두리 → 밝은 배경에서도 보이게
            cv2.putText(img, lab, org, f, 0.55, c, 1)
            if p.get("tag") is not None:
                u, v = p["tag"]["uv"]
                cv2.rectangle(img, (int(u) - 4, int(v) - 4), (int(u) + 4, int(v) + 4), (255, 255, 0), 1)
            if "center_cam" in p and self.K is not None:
                uvw = self.K @ p["center_cam"]
                if uvw[2] > 0:
                    u, v = int(uvw[0] / uvw[2]), int(uvw[1] / uvw[2])
                    cv2.circle(img, (u, v), 4, (255, 255, 255), -1)
                    cv2.circle(img, (u, v), 5, (0, 0, 0), 1)
                tr = p.get("track")
                if tr is not None and tr.pos is not None:
                    tag = "OK" if tr.confirmed else f"{tr.hits}/{self.confirm_hits}"
                    cv2.putText(img, f"[{tag}] {p.get('src', '')} ({tr.pos[0]*1000:.0f},{tr.pos[1]*1000:.0f},{tr.pos[2]*1000:.0f})mm",
                                (int(b[0]), int(b[3]) + 14), f, 0.4, c, 1)
        place = {12: "L1", 13: "L2", 14: "L3"}
        for t in all_tags:                                            # 태그마다 큰 ID
            u, v = int(t["uv"][0]), int(t["uv"][1])
            tid = t["id"]
            if tid in self.puck_ids:
                tc, lab = (255, 255, 0), f"#{tid}"
            elif tid in place:
                tc, lab = (255, 0, 255), f"#{tid} {place[tid]}"
            else:
                tc, lab = (160, 160, 160), f"#{tid}?"
            cv2.rectangle(img, (u - 7, v - 7), (u + 7, v + 7), tc, 2)
            cv2.putText(img, lab, (u + 9, v + 6), f, 0.7, (0, 0, 0), 4)
            cv2.putText(img, lab, (u + 9, v + 6), f, 0.7, tc, 2)
        ids = " ".join(str(t["id"]) for t in sorted(all_tags, key=lambda t: t["id"]))
        cv2.putText(img, f"tags: {ids if ids else '-'}", (6, 22), f, 0.6, (0, 0, 0), 3)
        cv2.putText(img, f"tags: {ids if ids else '-'}", (6, 22), f, 0.6, (255, 255, 255), 1)
        cv2.putText(img, f"frame: {frame}   pucks {len(pucks)}  tower {len(towers)}  drop {len(dropped)}",
                    (6, im.height - 8), f, 0.45, (255, 255, 255), 1)
        out = Image()
        out.header = im.header
        out.height, out.width, out.encoding, out.step = img.shape[0], img.shape[1], "bgr8", img.shape[1] * 3
        out.data = img.tobytes()
        self.dbg_pub.publish(out)

    # ---------------------------------------------------------------- 등록표
    def _register(self, tr, frame):
        key = str(tr.tag_id)
        rec = self.registry.get(key, {})
        new = {"tag_id": tr.tag_id, "color": tr.vote("color"), "shape": tr.vote("shape"),
               "posture": tr.vote("posture"),
               "obs": rec.get("obs", 0) + 1,
               "position": [round(float(v), 4) for v in tr.pos], "frame": frame,
               "first_seen": rec.get("first_seen", time.strftime("%Y-%m-%d %H:%M:%S")),
               "last_seen": time.strftime("%Y-%m-%d %H:%M:%S"),
               "id_conf": tr.id_conf(), "track_id": tr.id}               # [2026-10-30] ID 득표율·트랙
        if tr.tag_id in self.conflicts:                                 # 같은 ID 가 두 곳 → 미션이 가서 다시 확인
            new["conflict"] = self.conflicts[tr.tag_id]
        changed = any(rec.get(k) != new[k] for k in ("color", "shape", "posture"))
        self.registry[key] = new
        if changed:
            self.get_logger().info(f"등록: 태그 {tr.tag_id} = {new['color']} {new['shape']} {new['posture']}")
            self._log_change(new, rec)
        self._reg_dirty = True

    def _check_duplicates(self):
        """[2026-10-30] 동시에 살아있는 트랙 둘이 같은 ID 인데 떨어져 있으면 충돌로 기록 (덮어쓰지 않음)"""
        by_id = {}
        for x in self.tracks:
            if x.tag_id >= 0 and x.confirmed and x.pos is not None:
                by_id.setdefault(x.tag_id, []).append(x)
        for tid, xs in by_id.items():
            if len(xs) < 2:
                continue
            far = max(float(np.linalg.norm(a.pos - b.pos)) for a in xs for b in xs)
            if far < self.dup_dist:
                continue
            win = max(xs, key=lambda y: (y.id_conf(), y.total))           # 득표율·관측 수 높은 쪽만 등록표에 씀
            info = {"positions": [[round(float(v), 4) for v in x.pos] for x in xs],
                    "id_conf": [x.id_conf() for x in xs], "time": time.strftime("%H:%M:%S"),
                    "winner_track": win.id}
            if tid not in self.conflicts:
                self.get_logger().error(f"태그 {tid} 충돌: {len(xs)}곳 {info['positions']} → 등록표 conflict 표시")
            self.conflicts[tid] = info
            for x in xs:                                    # 득표율 낮은 쪽은 ID 보류 (다시 읽힐 때까지)
                if x is not win:
                    x.tag_id = -1

    def _backup_registry(self):
        try:
            if os.path.exists(self.registry_path):
                os.replace(self.registry_path, self.registry_path.replace(".json", "_prev.json"))
        except OSError:
            pass
        self._reg_dirty = True

    def _log_change(self, new, old):
        try:
            os.makedirs(os.path.dirname(self.registry_path), exist_ok=True)
            with open(os.path.join(os.path.dirname(self.registry_path), "puck_registry_log.jsonl"), "a") as f:
                f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "new": new,
                                    "old": {k: old.get(k) for k in ("color", "shape", "posture")}},
                                   ensure_ascii=False) + "\n")
        except OSError as e:
            self.get_logger().warning(f"등록 기록 실패: {e}", throttle_duration_sec=10.0)

    def _load_registry(self):
        try:
            with open(self.registry_path) as f:
                return json.load(f).get("pucks", {})
        except (OSError, ValueError):
            return {}

    def _save_registry(self, force=False):
        if not (self._reg_dirty or force):
            return
        self._reg_dirty = False
        doc = {"saved": time.strftime("%Y-%m-%d %H:%M:%S"), "count": len(self.registry),
               "pucks": dict(sorted(self.registry.items(), key=lambda kv: int(kv[0])))}
        txt = json.dumps(doc, ensure_ascii=False, indent=1)
        try:
            os.makedirs(os.path.dirname(self.registry_path), exist_ok=True)
            tmp = self.registry_path + ".tmp"
            with open(tmp, "w") as f:
                f.write(txt)
            os.replace(tmp, self.registry_path)
        except OSError as e:
            self.get_logger().warning(f"등록표 저장 실패: {e}", throttle_duration_sec=10.0)
        if rclpy.ok():
            self.reg_pub.publish(String(data=txt))


def main():
    rclpy.init()
    node = PuckRegistry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._save_registry(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
