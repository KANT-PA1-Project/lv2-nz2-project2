#!/usr/bin/env python3
# =====================================================================
# dice_judge  (NZ2 인지) — 장소 L1~L3 주사위 눈 배치 판정 + 증거 기록
#   역할: 장소 태그(12~14, 48 mm)로 3×3 격자 위치를 계산하고, 확정된 퍽 중심이 정해진 칸에
#         10 mm 안으로, 2초 이상 움직임 없이 놓였는지 판정. 빈 칸에 퍽이 있으면 실패.
#         성공·재검증 때 영상(격자·퍽 표시)과 관측한 태그 ID 를 파일로 남김 (재검증 8점 근거)
#   입력: /nz2/tags                (vision_msgs/Detection3DArray, 카메라 광학 프레임) — 장소 태그
#         /perception/puck_tracks   (nz2_interfaces/PuckArray, puck_registry) — 확정 퍽
#         /nz2/dice_plan            (std_msgs/String JSON {"L1":3,"L2":5,"L3":4}, transient_local) 또는 파라미터 plan
#         /gripper/color/image_raw, /gripper/color/camera_info (선택: 증거 사진)
#         /tf (선택: 퍽 프레임 ≠ 태그 프레임일 때, base_frame 으로 '위' 방향)
#   출력: /nz2/dice_result          (std_msgs/String JSON) — 보이는 장소마다 판정
#         /nz2/dice_summary         (std_msgs/String JSON, transient_local) — 장소별 마지막 상태·성공 여부
#         ~/nz2_logs/dice/<장소>_<시각>.jpg / .json, ~/nz2_logs/dice_log.jsonl
#   서비스: ~/snapshot (std_srvs/Trigger) — 지금 보이는 장소를 판정과 함께 바로 기록 (재검증 단계용)
#
#   기하 (미션 v5 p2·p4·p7): 격자 = 장소 중심 기준 X,Y 각 -60, 0, +60 mm. 지도 위쪽 +Y
#     장소 태그는 접근 방향 면, 중심 높이 35 mm. 박스 윗면 70 mm
#     L1 (3670,950)  태그 12 서쪽 면(-x), 면→격자 중심 102.5 mm
#     L2 (2202,3700) 태그 13 받침대 북쪽 면(+y), 면→격자 중심 205 mm  [받침대 820×820, 북면 y=3905]
#     L3 (3670,5354) 태그 14 남쪽 면(-y), 면→격자 중심 102.5 mm
#   주사위 눈: 2·3 = 왼쪽 위 → 오른쪽 아래 대각선, 6 = 좌우 두 열 (회전 대체 불가)
# =====================================================================
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from nz2_interfaces.msg import Puck, PuckArray

try:
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.time import Time
    HAVE_TF = True
except ImportError:                     # pragma: no cover
    HAVE_TF = False
try:
    import cv2
    HAVE_CV = True
except ImportError:                     # pragma: no cover
    HAVE_CV = False

DICE = {
    0: [],
    1: [(0, 0)],
    2: [(-1, 1), (1, -1)],
    3: [(-1, 1), (0, 0), (1, -1)],
    4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
    5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)],
    6: [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)],
}
CNAME = {Puck.COLOR_RED: "red", Puck.COLOR_GREEN: "green"}
SNAME = {Puck.SHAPE_CYLINDER: "cyl", Puck.SHAPE_BOX: "box"}


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class PlaceState:
    def __init__(self, name):
        self.name = name
        self.ok_since = None        # ok_now 가 이어진 시작 시각
        self.ref = {}               # 안정 판정 기준 위치 {퍽키: (x,y)}
        self.success = False
        self.success_time = None
        self.last = None            # 마지막 판정 요약 dict
        self.full = None            # 마지막 판정 전체 (결과, 격자 프레임) — 재검증 기록용
        self.fr = None              # 평활한 격자 프레임 (c, X, Y)
        self.seen = 0.0             # 마지막으로 장소 태그를 본 시각
        self.pp = {}                # 퍽키 → 평활한 격자 좌표 (x, y)
        self.bad_since = None       # ok 가 아니게 된 시각 (grace_s 판단)


class DiceJudge(Node):
    def __init__(self):
        super().__init__("dice_judge")
        d = self.declare_parameter
        self.tag_topic = d("tags_topic", "/nz2/tags").value
        self.puck_topic = d("pucks_topic", "/perception/puck_tracks").value
        self.plan_topic = d("plan_topic", "/nz2/dice_plan").value
        self.image_topic = d("image_topic", "/gripper/color/image_raw").value
        self.info_topic = d("info_topic", "/gripper/color/camera_info").value
        self.names = list(d("place_names", ["L1", "L2", "L3"]).value)
        self.tag_ids = list(d("place_tag_ids", [12, 13, 14]).value)
        self.faces = list(d("place_faces", ["-x", "+y", "-y"]).value)
        self.inward = list(d("place_inward_m", [0.1025, 0.205, 0.1025]).value)
        self.tag_to_top = d("tag_to_top_m", 0.035).value        # 태그 중심(35 mm) → 박스 윗면(70 mm)
        self.grid = d("grid_m", 0.060).value
        self.tol = d("center_tol_m", 0.010).value                # 칸 중심 오차 허용 (미션 10 mm)
        self.stable_s = d("stable_s", 2.0).value                 # 미션 2초 안정
        self.stable_tol = d("stable_tol_m", 0.006).value         # 안정 판정: 평활 위치가 이만큼 넘게 움직이면 다시 (태그 잡음 ~2 mm)
        self.alpha = d("smooth_alpha", 0.3).value                 # 격자·퍽 위치 지수평활 (1 = 평활 안 함)
        self.grace = d("grace_s", 0.4).value                      # 판정이 잠깐(이 시간 안) 틀어져도 안정 시간 유지 (가림·잡음)
        self.region = d("region_m", 0.115).value                 # 격자 중심에서 이 안에 있는 퍽 = 이 장소 위 (90 + 25 mm)
        self.max_age = d("max_age_s", 0.5).value                 # 태그·퍽 메시지 최대 나이 (0 = 검사 안 함)
        self.base_frame = d("base_frame", "base_link").value     # '위' 방향 (tf 없으면 태그 y 축 = 위)
        self.plan_str = d("plan", "").value                      # 예 "L1:3,L2:5,L3:4" (토픽이 오면 토픽 우선)
        self.log_dir = os.path.expanduser(d("log_dir", "~/nz2_logs").value)
        self.hz = d("process_hz", 10.0).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Detection3DArray, self.tag_topic, self.on_tags, 10)
        self.create_subscription(PuckArray, self.puck_topic, self.on_pucks, 10)
        self.create_subscription(String, self.plan_topic, self.on_plan, latched)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.res_pub = self.create_publisher(String, "/nz2/dice_result", 10)
        self.sum_pub = self.create_publisher(String, "/nz2/dice_summary", latched)
        if HAVE_TF:
            self.tf_buf = tf2_ros.Buffer()
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        try:
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "~/snapshot", self.on_snapshot)
            self.create_service(Trigger, "~/reset", self.on_reset)
        except ImportError:
            pass

        self.plan = self._parse_plan_str(self.plan_str)
        self.tags = None
        self.pucks = None
        self.tags_rx = self.pucks_rx = 0.0      # 받은 시각 (단조 시계) — 나이 검사는 받은 시각 기준
        self.image = None
        self.K = None
        self.state = {n: PlaceState(n) for n in self.names}
        os.makedirs(os.path.join(self.log_dir, "dice"), exist_ok=True)
        self.create_timer(1.0 / self.hz, self.tick)
        self.get_logger().info(f"주사위 판정 시작: 장소 {dict(zip(self.names, self.tag_ids))}, 계획 {self.plan or '(없음)'}")

    # ---------------------------------------------------------------- 입력
    def on_tags(self, m):
        self.tags, self.tags_rx = m, time.monotonic()

    def on_pucks(self, m):
        self.pucks, self.pucks_rx = m, time.monotonic()

    def on_image(self, m):
        self.image = m

    def on_info(self, m):
        if m.k[0] > 0:
            self.K = np.array(m.k, dtype=float).reshape(3, 3)

    def on_plan(self, m):
        try:
            p = {k: int(v) for k, v in json.loads(m.data).items() if k in self.names}
        except (ValueError, TypeError, AttributeError):
            self.get_logger().error(f"dice_plan 형식 오류: {m.data!r}")
            return
        if any(not 0 <= v <= 6 for v in p.values()):
            self.get_logger().error(f"dice_plan 눈 수는 0~6: {p}")
            return
        if p != self.plan:
            self.plan = p
            for s in self.state.values():
                s.success, s.ok_since, s.ref = False, None, {}
            self.get_logger().info(f"주사위 계획 = {p}")

    def _parse_plan_str(self, s):
        out = {}
        for part in s.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                if k.strip() in self.names:
                    out[k.strip()] = int(v)
        return out

    def on_reset(self, req, res):
        for s in self.state.values():
            s.success, s.ok_since, s.ref, s.last = False, None, {}, None
        res.success, res.message = True, "판정 상태 초기화"
        return res

    # ---------------------------------------------------------------- 기하
    def _tf(self, target, source, stamp):
        if target == source or not source:
            return np.eye(4)
        if not HAVE_TF or not target:
            return None
        try:
            tr = self.tf_buf.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.02))
        except Exception:
            try:
                tr = self.tf_buf.lookup_transform(target, source, Time())
            except Exception:
                return None
        q, t = tr.transform.rotation, tr.transform.translation
        M = np.eye(4)
        M[:3, :3] = quat_to_R(q)
        M[:3, 3] = [t.x, t.y, t.z]
        return M

    def place_frame(self, i, p_tag, R_tag, up):
        """장소 격자 중심 c 와 지도 X·Y 축 (모두 태그 프레임=카메라 좌표)"""
        n = R_tag[:, 2]                               # 태그 바깥 법선
        n_h = unit(n - (n @ up) * up)
        if n_h is None:
            return None
        face = self.faces[i]
        if face in ("-x", "+x"):
            X = -n_h if face == "-x" else n_h
            Y = np.cross(up, X)
        else:
            Y = -n_h if face == "-y" else n_h
            X = np.cross(Y, up)
        c = p_tag - n_h * self.inward[i] + up * self.tag_to_top
        return c, X, Y

    # ---------------------------------------------------------------- 판정
    def tick(self):
        now = time.monotonic()
        tm, pm = self.tags, self.pucks
        if tm is None:
            return
        if self.max_age > 0 and now - self.tags_rx > self.max_age:
            return                                    # 태그 메시지가 최근에 안 옴
        frame = tm.header.frame_id
        # 위 방향 (카메라 좌표)
        up_tf = None
        if self.base_frame:
            M = self._tf(self.base_frame, frame, tm.header.stamp)
            if M is not None and self.base_frame != frame:
                up_tf = M[:3, :3].T @ np.array([0, 0, 1.0])
        # 퍽 → 태그 프레임 (퍽 정보가 없거나 오래됐으면 이번 판정은 건너뜀 — 빈 칸으로 오판하지 않게)
        pucks = None
        if pm is not None and not (self.max_age > 0 and now - self.pucks_rx > self.max_age):
            M = self._tf(frame, pm.header.frame_id, tm.header.stamp)
            if M is None:
                self.get_logger().warning(f"tf {pm.header.frame_id}→{frame} 없음 → 퍽 위치 못 씀", throttle_duration_sec=5.0)
            else:
                pucks = []
                for pk in pm.pucks:
                    q = (M @ np.array([pk.position.x, pk.position.y, pk.position.z, 1.0]))[:3]
                    pucks.append({"p": q, "key": f"t{pk.tag_id}" if pk.tag_id >= 0 else f"k{pk.track_id}",
                                  "tag_id": int(pk.tag_id), "color": CNAME.get(pk.color, "?"),
                                  "shape": SNAME.get(pk.shape, "?"),
                                  "posture": {Puck.POSTURE_UP: "up", Puck.POSTURE_DOWN: "down"}.get(pk.posture, "?")})
        if pucks is None:
            self.get_logger().warning("퍽 추적 결과가 없거나 오래됨 → 판정 보류", throttle_duration_sec=5.0)
            return
        for det in tm.detections:
            try:
                tid = int(det.id)
            except ValueError:
                continue
            if tid not in self.tag_ids or not det.results or det.results[0].hypothesis.score < 0.99:
                continue
            i = self.tag_ids.index(tid)
            pp = det.results[0].pose.pose
            R = quat_to_R(pp.orientation)
            up = up_tf if up_tf is not None else unit(R[:, 1])      # 태그를 바로 세워 붙였다고 가정
            fr = self.place_frame(i, np.array([pp.position.x, pp.position.y, pp.position.z]), R, up)
            if fr is None:
                continue
            res = self.judge(i, fr, pucks, now)
            res["place_tag_id"] = tid
            res["frame"] = frame
            self.res_pub.publish(String(data=json.dumps(res, ensure_ascii=False)))
        self.sum_pub.publish(String(data=json.dumps(
            {n: (s.last or {}) | {"success": s.success} for n, s in self.state.items()}, ensure_ascii=False)))

    def judge(self, i, fr, pucks, now):
        name = self.names[i]
        st = self.state[name]
        a = self.alpha
        if st.fr is None or now - st.seen > 1.0:            # 처음이거나 1초 넘게 못 봤으면 새로 시작
            st.fr, st.pp = fr, {}
        else:                                               # 장소 태그 잡음 평활
            c0, X0, Y0 = st.fr
            X1 = unit((1 - a) * X0 + a * fr[1])
            Y1 = unit((1 - a) * Y0 + a * fr[2])
            st.fr = ((1 - a) * c0 + a * fr[0], X1 if X1 is not None else fr[1], Y1 if Y1 is not None else fr[2])
        st.seen = now
        fr = st.fr
        c, X, Y = fr
        g = self.grid
        expected = self.plan.get(name)
        want = set(DICE.get(expected, [])) if expected is not None else None
        on = []
        for pk in pucks:
            d = pk["p"] - c
            x, y = float(d @ X), float(d @ Y)
            if abs(x) > self.region or abs(y) > self.region:
                continue
            if pk["key"] in st.pp:                          # 퍽 위치 잡음 평활
                px, py = st.pp[pk["key"]]
                x, y = (1 - a) * px + a * x, (1 - a) * py + a * y
            st.pp[pk["key"]] = (x, y)
            cell = (int(max(-1, min(1, round(x / g)))), int(max(-1, min(1, round(y / g)))))
            err = math.hypot(x - cell[0] * g, y - cell[1] * g)
            on.append({**{k: pk[k] for k in ("key", "tag_id", "color", "shape", "posture")},
                       "x_mm": round(x * 1000, 1), "y_mm": round(y * 1000, 1), "cell": list(cell),
                       "err_mm": round(err * 1000, 1)})
        cells = {}
        for o in on:
            cells.setdefault(tuple(o["cell"]), []).append(o)
        problems = []
        if want is not None:
            for cl in sorted(want):
                n = len(cells.get(cl, []))
                if n == 0:
                    problems.append(f"칸 {cl} 비었음")
                elif n > 1:
                    problems.append(f"칸 {cl} 에 퍽 {n}개")
                elif cells[cl][0]["err_mm"] > self.tol * 1000:
                    problems.append(f"칸 {cl} 오차 {cells[cl][0]['err_mm']} mm")
            for cl, lst in cells.items():
                if cl not in want:
                    problems.append(f"빈 칸이어야 할 {cl} 에 퍽 {len(lst)}개")
        ok_now = want is not None and not problems
        # 2초 안정: ok 가 이어지고 퍽이 stable_tol 이상 안 움직임
        pos = {o["key"]: (o["x_mm"] / 1000, o["y_mm"] / 1000) for o in on}
        if ok_now:
            moved = any(k not in pos or math.hypot(pos[k][0] - v[0], pos[k][1] - v[1]) > self.stable_tol
                        for k, v in st.ref.items()) or set(pos) != set(st.ref)
            if st.ok_since is None or moved:
                st.ok_since, st.ref = now, pos
            st.bad_since = None
        else:
            if st.bad_since is None:
                st.bad_since = now
            if now - st.bad_since > self.grace:           # 잠깐 틀어진 건 봐줌 (가림·한 프레임 잡음)
                st.ok_since, st.ref = None, {}
        stable = (now - st.ok_since) if (st.ok_since is not None and ok_now) else 0.0
        res = {"place": name, "expected": expected, "detected": len(on), "pucks": on,
               "ok_now": ok_now, "stable_s": round(stable, 2), "success": False, "problems": problems,
               "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        success_now = ok_now and stable >= self.stable_s          # 지금 성공 상태 (깨지면 다시 False)
        res["success"] = success_now
        if success_now and not st.success:
            st.success_time = now
            self.get_logger().info(f"[{name}] 주사위 {expected} 성공 (2초 안정) — 퍽 태그 {[o['tag_id'] for o in on]}")
            self._record(res, fr, "success")
        elif st.success and not success_now and st.ok_since is None:   # grace 지나서 확실히 깨졌을 때만
            self.get_logger().warning(f"[{name}] 성공 상태가 깨짐: {problems or '퍽 움직임'}")
            self._record(res, fr, "broken")
        st.success = success_now or (st.success and st.ok_since is not None)
        st.full = (res, fr)
        st.last = {k: res[k] for k in ("expected", "detected", "ok_now", "stable_s", "problems", "time")} | \
            {"tag_ids": sorted(o["tag_id"] for o in on)}
        return res

    # ---------------------------------------------------------------- 증거 기록
    def _record(self, res, fr, why):
        base = os.path.join(self.log_dir, "dice", f"{res['place']}_{time.strftime('%Y%m%d_%H%M%S')}_{why}")
        try:
            with open(base + ".json", "w") as f:
                json.dump(res, f, ensure_ascii=False, indent=1)
            with open(os.path.join(self.log_dir, "dice_log.jsonl"), "a") as f:
                f.write(json.dumps({"why": why, **res}, ensure_ascii=False) + "\n")
        except OSError as e:
            self.get_logger().warning(f"판정 기록 실패: {e}")
        img = self._image_bgr()
        if img is None or not HAVE_CV:
            return
        if self.K is not None and fr is not None:
            c, X, Y = fr
            want = set(DICE.get(res["expected"], [])) if res["expected"] is not None else set()
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    P = c + X * i * self.grid + Y * j * self.grid
                    uv = self.K @ P
                    if uv[2] > 0:
                        col = (0, 200, 0) if (i, j) in want else (160, 160, 160)
                        cv2.circle(img, (int(uv[0] / uv[2]), int(uv[1] / uv[2])), 5, col, 2)
        cv2.putText(img, f"{res['place']} exp={res['expected']} det={res['detected']} "
                         f"{'OK' if res['ok_now'] else 'NG'} {res['stable_s']}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(img, "tags " + ",".join(str(o["tag_id"]) for o in res["pucks"]) + f" | {res['time']}",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(base + ".jpg", img)

    def _image_bgr(self):
        m = self.image
        if m is None:
            return None
        try:
            a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
            a = a.reshape(m.height, m.width, 3).copy()
        except ValueError:
            return None
        return a[..., ::-1].copy() if m.encoding == "rgb8" else a

    def on_snapshot(self, req, res):
        """재검증: 지금 보이는 장소의 마지막 판정을 사진·JSON 으로 기록"""
        n = 0
        for s in self.state.values():
            if s.full is not None:
                self._record(s.full[0], s.full[1], "snapshot")
                n += 1
        res.success, res.message = n > 0, f"{n}개 장소 기록"
        return res


def main():
    rclpy.init()
    node = DiceJudge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
