#!/usr/bin/env python3
"""yolo_detector.py — YOLO 검출 노드 (Python, ultralytics .pt 직접 추론)

C++ yolo_detector(TensorRT .engine / OpenCV .onnx, 디코드·NMS 구현됨 — yolo_engine.cpp)와 입출력이 똑같은 PC 테스트용 대체 노드.
  입력 : /front/color/image_raw, /gripper/color/image_raw   (sensor_msgs/Image)
  출력 : /perception/detections                             (vision_msgs/Detection2DArray)
         → traffic_light_filter, puck_registry, puck_localizer 가 구독 (전면 카메라는 2026-10-28 삭제)

실행 (yolo_env 활성화 상태에서. 첫 줄이 '#!/usr/bin/env python3' 이라 yolo_env 의 python 으로 실행됨):
    ros2 launch perception perception.launch.py yolo:=py          # 다른 인지 노드와 함께
    ros2 run perception yolo_detector.py                           # 단독 실행
    ros2 run perception yolo_detector.py --ros-args -p model_path:=~/다른/경로/best.pt

주의
  - C++ yolo_detector 와 동시에 띄우지 말 것(같은 토픽에 빈 결과가 섞여 신호등 확정이 깨짐).
    perception.launch.py 는 yolo:=py 일 때 C++ 쪽을 자동으로 끈다.
  - cv_bridge 는 쓰지 않는다. ROS cv_bridge(NumPy 1.x 빌드)와 yolo_env(NumPy 2.x)가 충돌하기 때문.
    영상 변환은 numpy 로 직접 한다(아래 imgmsg_to_bgr).
  - 본선 Jetson 은 C++ yolo_detector + .engine 을 쓴다. 이 노드는 노트북 테스트용.
"""
import os
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# 기본 모델: 배포 모델 (pipeline.py release 가 교체). 예전 sc_real_v1(7클래스)은 보관 처리됨
DEFAULT_MODEL = '~/nz2_models/current.pt'

# ROS 영상 인코딩 → (채널 수, numpy 타입)
_ENC = {
    'bgr8': (3, np.uint8), 'rgb8': (3, np.uint8),
    'bgra8': (4, np.uint8), 'rgba8': (4, np.uint8),
    'mono8': (1, np.uint8), '8UC1': (1, np.uint8), '8UC3': (3, np.uint8),
}


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image → HxWx3 uint8 BGR numpy 배열 (cv_bridge 없이).

    RealSense 컬러는 보통 rgb8 이고 YOLO(ultralytics)는 BGR 을 기대하므로 순서를 맞춘다.
    지원하지 않는 인코딩이면 ValueError.
    """
    if msg.encoding not in _ENC:
        raise ValueError(f'지원하지 않는 인코딩: {msg.encoding}')
    ch, dt = _ENC[msg.encoding]
    itemsize = np.dtype(dt).itemsize
    if msg.step < msg.width * ch * itemsize:
        raise ValueError(f'step({msg.step})이 width*채널보다 작음')
    # 행 끝에 패딩이 있을 수 있어 step 기준으로 자른 뒤 유효 폭만 사용
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * ch * itemsize].view(dt).reshape(msg.height, msg.width, ch)
    enc = msg.encoding
    if enc in ('bgr8', '8UC3'):
        return np.ascontiguousarray(img)
    if enc == 'rgb8':
        return np.ascontiguousarray(img[:, :, ::-1])
    if enc == 'bgra8':
        return np.ascontiguousarray(img[:, :, :3])
    if enc == 'rgba8':
        return np.ascontiguousarray(img[:, :, 2::-1])
    return np.ascontiguousarray(np.repeat(img, 3, axis=2))   # 흑백 → 3채널


class YoloDetectorPy(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        ro = ParameterDescriptor(read_only=True)
        raw_path = self.declare_parameter('model_path', DEFAULT_MODEL, ro).value
        # ~ 를 홈 폴더로 펼치고 절대경로로 (상대경로는 '명령 실행 폴더' 기준이라 헷갈림)
        self.model_path = os.path.abspath(os.path.expanduser(raw_path))
        self.conf = float(self.declare_parameter('conf_threshold', 0.25, ro).value)
        self.iou = float(self.declare_parameter('iou_threshold', 0.45, ro).value)
        self.imgsz = int(self.declare_parameter('imgsz', 640, ro).value)
        self.device = self.declare_parameter('device', '', ro).value   # '' = 자동(GPU 있으면 GPU)
        enable_front = bool(self.declare_parameter('enable_front', False, ro).value)   # [2026-10-28] 전면 카메라 삭제 → 기본 끔
        enable_gripper = bool(self.declare_parameter('enable_gripper', True, ro).value)
        # 디버그 영상: 박스 그린 영상 → /perception/<front|gripper>/debug_image (RViz2 Image / rqt_image_view)
        self.debug_image = bool(self.declare_parameter('publish_debug_image', True, ro).value)

        # --- 파라미터 검증: 잘못된 설정이면 시작하지 않는다 ---
        if not 0.0 <= self.conf <= 1.0:
            raise ValueError('conf_threshold must be in [0, 1]')
        if not 0.0 <= self.iou <= 1.0:
            raise ValueError('iou_threshold must be in [0, 1]')
        if self.imgsz <= 0:
            raise ValueError('imgsz must be positive')
        if not os.path.isfile(self.model_path):
            raise ValueError(
                f"모델 파일이 없습니다: '{self.model_path}' (입력값 '{raw_path}'). "
                '-p model_path:=<절대경로> 로 지정하세요.')

        # ultralytics 는 yolo_env 에만 있으므로 여기서 import 해서 원인을 분명히 알린다
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise ValueError(
                'ultralytics 를 불러오지 못했습니다. yolo_env 를 활성화한 터미널에서 실행하세요. '
                f'({e})') from e
        self.model = YOLO(self.model_path)
        self.names = self.model.names   # {클래스id: 이름} — 학습 config 순서 그대로

        det_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Detection2DArray, '/perception/detections', det_qos)
        self.dbg_pubs = {}
        self._tstat = {}
        if self.debug_image:
            dbg_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
            cams = [c for c, on in (('front', enable_front), ('gripper', enable_gripper)) if on]   # [2026-10-30] 켠 카메라만
            self.dbg_pubs = {c: self.create_publisher(Image, f'/perception/{c}/debug_image', dbg_qos)
                             for c in cams}

        # 영상 구독: 최근 1장만 유지. 추론이 느려 밀리면 오래된 프레임은 버려진다(지연 누적 방지).
        # 영상 구독 QoS: 기본 RELIABLE. BEST_EFFORT 로 큰 영상(848x480 = 1.2MB)을 받으면 DDS 가 조각(fragment)을
        # 하나만 잃어도 프레임 전체를 버려서 15Hz 중 1Hz 만 들어오는 문제가 있었음 (2026-10-26 노트북 실측)
        reliable = bool(self.declare_parameter('image_reliable', True, ro).value)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE if reliable
                                else ReliabilityPolicy.BEST_EFFORT)
        self.subs = []
        if enable_front:
            self.subs.append(self.create_subscription(
                Image, '/front/color/image_raw',
                lambda m: self.on_image(m, 'front'), sensor_qos))
        if enable_gripper:
            self.subs.append(self.create_subscription(
                Image, '/gripper/color/image_raw',
                lambda m: self.on_image(m, 'gripper'), sensor_qos))
        if not self.subs:
            self.get_logger().warning('enable_front/enable_gripper 가 모두 false — 아무 영상도 구독하지 않음')

        self.get_logger().info(
            f"yolo_detector(py) 시작: model='{self.model_path}', imgsz {self.imgsz}, "
            f'conf {self.conf:.2f}, iou {self.iou:.2f}, 클래스 {len(self.names)}개 {list(self.names.values())}, '
            f"front={'on' if enable_front else 'off'} gripper={'on' if enable_gripper else 'off'} "
            '→ /perception/detections')

    def _timing(self, camera, msg, bgr, r, t0, t1):
        """5초마다 속도 로그: 처리 횟수/초, 단계별 ms, 영상 지연(카메라 시각 → 지금)"""
        st = self._tstat.setdefault(camera, {'n': 0, 'cb': 0.0, 'since': time.perf_counter()})
        st['n'] += 1
        st['cb'] += (time.perf_counter() - t0) * 1000
        el = time.perf_counter() - st['since']
        if el >= 5.0:
            sp = r.speed   # ultralytics: preprocess / inference / postprocess (ms)
            self.get_logger().info(
                f"[{camera}] {st['n'] / el:.1f} Hz | 콜백 평균 {st['cb'] / st['n']:.0f} ms "
                f"(변환+추론 {(t1 - t0) * 1000:.0f}: 전처리 {sp['preprocess']:.0f} / 추론 {sp['inference']:.0f} / "
                f"후처리 {sp['postprocess']:.0f}) | 영상 {bgr.shape[1]}x{bgr.shape[0]} {msg.encoding} | "
                f"지연 {_stamp_age(self, msg):.0f} ms | device {getattr(getattr(self.model, 'predictor', None), 'device', '?')}")
            self._tstat[camera] = {'n': 0, 'cb': 0.0, 'since': time.perf_counter()}

    def on_image(self, msg, camera):
        t0 = time.perf_counter()
        try:
            bgr = imgmsg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warning(f'[{camera}] 영상 변환 실패: {e}', throttle_duration_sec=5.0)
            return
        if bgr.size == 0:
            return
        try:
            r = self.model.predict(
                bgr, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                device=(self.device or None), verbose=False)[0]
        except Exception as e:   # 추론 한 번 실패로 노드가 죽지 않도록
            self.get_logger().error(f'[{camera}] 추론 실패: {e}', throttle_duration_sec=5.0)
            return
        t1 = time.perf_counter()

        out = Detection2DArray()
        out.header = msg.header   # 스탬프/frame_id 유지 → 하위 노드가 전면/그리퍼 구분
        for box in r.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            cls_id = int(box.cls[0].item())
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) * 0.5
            det.bbox.center.position.y = (y1 + y2) * 0.5
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names.get(cls_id, str(cls_id))
            hyp.hypothesis.score = float(box.conf[0].item())
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)
        self._timing(camera, msg, bgr, r, t0, t1)
        pub = self.dbg_pubs.get(camera)
        if pub is not None and pub.get_subscription_count() > 0:   # 보는 사람이 있을 때만 그림
            try:
                img = np.ascontiguousarray(r.plot())               # ultralytics 가 박스·클래스·점수를 그린 BGR
                m = Image()
                m.header = msg.header
                m.height, m.width = img.shape[:2]
                m.encoding = 'bgr8'
                m.is_bigendian = 0
                m.step = img.shape[1] * 3
                m.data = img.tobytes()
                pub.publish(m)
                self._dbg_sent = getattr(self, '_dbg_sent', 0) + 1
                if self._dbg_sent == 1:
                    self.get_logger().info(f'[{camera}] 디버그 영상 첫 발행 → /perception/{camera}/debug_image')
            except Exception as e:   # 디버그 실패로 검출 발행이 멈추지 않게
                self.get_logger().error(f'[{camera}] 디버그 영상 실패: {e}', throttle_duration_sec=5.0)


def _stamp_age(node, msg):
    now = node.get_clock().now().nanoseconds
    st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return (now - st) / 1e6


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = YoloDetectorPy()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as e:
        rclpy.logging.get_logger('yolo_detector').error(f'시작 실패: {e}')
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
