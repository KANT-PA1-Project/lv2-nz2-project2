"""puck_registry + dice_judge 시험용 가짜 센서 (카메라·YOLO·tag_reader 없이 노드 동작 확인)

L1 앞에서 본 장면을 흉내: 퍽 3개를 주사위 3 칸에 놓고 (누운 직육면체는 YOLO 가 cyl_green_down 으로 틀리게 냄),
태그·YOLO 박스·camera_info·tf(odom→base_link→gripper_color_optical_frame)·주사위 계획 {"L1":3} 을 발행.
2~9 s: 신호등 박스와 겹치는 가짜 초록 퍽 (버려져야 함) / 5 s 후: 빈 칸 (1,1) 에 퍽 추가 (실패로 바뀌어야 함)

실행 (터미널 3개, 모두 source install/setup.bash):
  ros2 run perception puck_registry.py --ros-args --params-file <perception.yaml> -r __node:=puck_registry
  ros2 run perception dice_judge.py    --ros-args --params-file <perception.yaml> -r __node:=dice_judge
  python3 src/perception/test/sim_dice.py [seed] [--depth] [--image]
  (--image: 단순 그림 컬러 영상 → /perception/puck_debug_image 확인, rqt_image_view 로 보기)
기대: "등록: 태그 3 = red cyl up / 7 = green box down / 5 = red box up", "[L1] 주사위 3 성공", 5 s 뒤 "성공 상태가 깨짐"
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

W, H, HFOV = 848, 480, 69.0
FX = W / (2 * math.tan(math.radians(HFOV) / 2))
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
L1 = np.array([3.670, 0.950])
BOX_H, G = 0.07, 0.06
RNG = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
DEPTH = "--depth" in sys.argv
IMAGE = "--image" in sys.argv     # 단순 그림 컬러 영상도 발행 (puck_registry 디버그 영상 확인용)


def R_to_q(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# 카메라 (월드 = odom = 경기장 좌표). 로봇은 L1 서쪽, +X 를 봄
BASE = np.array([L1[0] - 0.1025 - 0.42, L1[1], 0.0])
EYE = BASE + np.array([0.26, 0.0, 0.30])
TGT = np.array([L1[0], L1[1], BOX_H + 0.02])
f = (TGT - EYE) / np.linalg.norm(TGT - EYE)
right = np.cross(f, [0, 0, 1.0])
right /= np.linalg.norm(right)
down = np.cross(f, right)
R_WC = np.stack([right, down, f], axis=1)        # 카메라 → 월드 회전


class Puck:
    def __init__(self, cell, shape, color, posture, tag, yaw=0.0, off=(0, 0), yolo=None):
        self.c = np.array([L1[0] + cell[0] * G + off[0], L1[1] + cell[1] * G + off[1],
                           BOX_H + (0.03 if posture == "up" else 0.015)])
        self.shape, self.color, self.posture, self.tag, self.yaw = shape, color, posture, tag, yaw
        self.yolo = yolo or f"{shape}_{color}_{posture}"
        if posture == "up":
            self.R = rotz(yaw)                                     # 길이축 = z
        else:
            self.R = rotz(yaw) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])   # Rx(90): 길이축 → -y
        self.half = np.array([0.015, 0.015, 0.03])

    def corners(self):
        return [self.c + self.R @ (self.half * np.array(s)) for s in
                [(a, b, c) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]]

    def top_tag(self):
        """위를 보는 면의 태그: 서있으면 윗면(30 mm 위), 누운 직육면체면 위쪽 옆면(15 mm 위)"""
        h = 0.03 if self.posture == "up" else 0.015
        p = self.c + np.array([0, 0, h + 0.0004])
        z = np.array([0, 0, 1.0])
        x = rotz(self.yaw + 0.3) @ np.array([1.0, 0, 0])
        return p, np.stack([x, np.cross(z, x), z], axis=1)


def place_tag():
    p = np.array([L1[0] - 0.1025 - 0.0004, L1[1], 0.035])
    z = np.array([-1.0, 0, 0])
    y = np.array([0, 0, 1.0])
    return p, np.stack([np.cross(y, z), y, z], axis=1)


def scene(t):
    ps = [Puck((-1, 1), "cyl", "red", "up", 3, 0.2, (0.003, -0.002)),
          Puck((0, 0), "box", "green", "down", 7, 0.05, (0.002, 0.004), yolo="cyl_green_down"),   # YOLO 오분류 흉내
          Puck((1, -1), "box", "red", "up", 5, 0.4, (-0.004, 0.001))]
    if t > 5.0:
        ps.append(Puck((1, 1), "cyl", "green", "up", 9, 0.0, (0.0, 0.0)))       # 빈 칸에 추가 → 실패해야 함
    return ps


class Sim(Node):
    def __init__(self):
        super().__init__("sim_dice")
        self.det_pub = self.create_publisher(Detection2DArray, "/perception/detections", 10)
        self.tag_pub = self.create_publisher(Detection3DArray, "/nz2/tags", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/gripper/color/camera_info", 10)
        self.dep_pub = self.create_publisher(Image, "/gripper/aligned_depth_to_color/image_raw", 10)
        self.img_pub = self.create_publisher(Image, "/gripper/color/image_raw", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.plan_pub = self.create_publisher(String, "/nz2/dice_plan", latched)
        self.plan_pub.publish(String(data='{"L1": 3}'))
        self.st = StaticTransformBroadcaster(self)
        tfs = []
        for parent, child, pos, R in [("odom", "base_link", BASE, np.eye(3)),
                                      ("base_link", "gripper_color_optical_frame", EYE - BASE, R_WC)]:
            m = TransformStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id, m.child_frame_id = parent, child
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, pos)
            q = R_to_q(R)
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
            tfs.append(m)
        self.st.sendTransform(tfs)
        self.t0 = time.monotonic()
        self.create_timer(1 / 15, self.tick)

    def to_cam(self, p):
        return R_WC.T @ (p - EYE)

    def tick(self):
        t = time.monotonic() - self.t0
        now = self.get_clock().now().to_msg()
        hdr_frame = "gripper_color_optical_frame"
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = now, hdr_frame
        info.width, info.height = W, H
        info.k = [float(v) for v in K.ravel()]
        self.info_pub.publish(info)
        dets = Detection2DArray()
        dets.header.stamp, dets.header.frame_id = now, hdr_frame
        tags = Detection3DArray()
        tags.header = dets.header
        depth = np.zeros((H, W), np.uint16)
        for pk in scene(t):
            uv = [K @ self.to_cam(c) for c in pk.corners()]
            us = [u[0] / u[2] for u in uv]
            vs = [u[1] / u[2] for u in uv]
            d = Detection2D()
            d.header = dets.header
            d.bbox.center.position.x = (min(us) + max(us)) / 2
            d.bbox.center.position.y = (min(vs) + max(vs)) / 2
            d.bbox.size_x, d.bbox.size_y = max(us) - min(us), max(vs) - min(vs)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = pk.yolo, 0.8
            d.results.append(h)
            dets.detections.append(d)
            if DEPTH:      # 보이는 표면 깊이 흉내: 박스 영역에 퍽 중심 깊이 - 15 mm
                zc = self.to_cam(pk.c)[2] - 0.015
                depth[int(min(vs)):int(max(vs)), int(min(us)):int(max(us))] = int(zc * 1000)
            p, R = pk.top_tag()
            tags.detections.append(self.tag3d(pk.tag, p + RNG.normal(0, 0.0015, 3), R, 0.016))
        if 2.0 < t < 9.0:      # 신호등 근처 초록 퍽 오검출 흉내 (신호등 박스와 겹침)
            for cls, box in [("tower_light", (100.0, 100.0, 140.0, 200.0)), ("cyl_green_up", (102.0, 110.0, 138.0, 160.0))]:
                d = Detection2D()
                d.header = dets.header
                d.bbox.center.position.x, d.bbox.center.position.y = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
                d.bbox.size_x, d.bbox.size_y = float(box[2] - box[0]), float(box[3] - box[1])   # int 를 넣으면 0 이 됨
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id, h.hypothesis.score = cls, 0.9
                d.results.append(h)
                dets.detections.append(d)
        p, R = place_tag()
        tags.detections.append(self.tag3d(12, p + RNG.normal(0, 0.001, 3), R, 0.048))
        if IMAGE:
            self.pub_image(dets.header, scene(t))
        self.tag_pub.publish(tags)
        self.det_pub.publish(dets)
        if DEPTH:
            im = Image()
            im.header = dets.header
            im.height, im.width, im.encoding, im.step = H, W, "16UC1", W * 2
            im.data = depth.tobytes()
            self.dep_pub.publish(im)
        if t > 10.5:
            raise SystemExit

    def proj(self, pw):
        u = K @ self.to_cam(pw)
        return [u[0] / u[2], u[1] / u[2]]

    def pub_image(self, header, pucks):
        """단순 그림: 박스 윗면·앞면, 퍽(볼록 껍질), 위 태그. 실제 렌더가 아님 (오버레이 위치 확인용)"""
        import cv2
        img = np.full((H, W, 3), (70, 60, 50), np.uint8)
        h = 0.1025
        top = [self.proj(np.array([L1[0] + sx * h, L1[1] + sy * h, BOX_H])) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        front = [self.proj(np.array([L1[0] - h, L1[1] + sy * h, z])) for sy, z in ((-1, 0), (1, 0), (1, BOX_H), (-1, BOX_H))]
        cv2.fillPoly(img, [np.int32(front)], (120, 150, 170))
        cv2.fillPoly(img, [np.int32(top)], (170, 200, 215))
        pc, Rc = place_tag()
        cv2.fillPoly(img, [np.int32([self.proj(pc + Rc @ np.array([sx * 0.024, sy * 0.024, 0]))
                                     for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])], (20, 20, 20))
        for pk in sorted(pucks, key=lambda q: -self.to_cam(q.c)[2]):
            hull = cv2.convexHull(np.float32([self.proj(c) for c in pk.corners()]))
            cv2.fillPoly(img, [np.int32(hull)], (40, 40, 200) if pk.color == "red" else (40, 170, 40))
            tp, tR = pk.top_tag()
            cv2.fillPoly(img, [np.int32([self.proj(tp + tR @ np.array([sx * 0.008, sy * 0.008, 0]))
                                         for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])], (20, 20, 20))
        im = Image()
        im.header = header
        im.height, im.width, im.encoding, im.step = H, W, "bgr8", W * 3
        im.data = img.tobytes()
        self.img_pub.publish(im)

    def tag3d(self, tid, p_w, R_wt, size):
        d = Detection3D()
        d.id = str(tid)
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id, h.hypothesis.score = "tag36h11", 1.0
        pc = self.to_cam(p_w)
        Rc = R_WC.T @ R_wt
        h.pose.pose.position.x, h.pose.pose.position.y, h.pose.pose.position.z = map(float, pc)
        q = R_to_q(Rc)
        h.pose.pose.orientation.x, h.pose.pose.orientation.y, h.pose.pose.orientation.z, h.pose.pose.orientation.w = q
        d.results.append(h)
        d.bbox.size.x = d.bbox.size.y = size
        return d


def main():
    rclpy.init()
    n = Sim()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
