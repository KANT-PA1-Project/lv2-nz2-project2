# =============================================================
# 패키지 : nz2_control
# 파일   : opencr_bridge_node.py
# 레이어 : control
# 역할   : Jetson ↔ OpenCR #1(스월브 + IMU) / #2(팔) 통신 브리지
#
# [입력]  /nz2/swerve_command (nz2_interfaces/SwerveCommand)
#         팔 관절 명령 (arm_controller_node)
#         params: 포트 이름, 통신 속도, 타임아웃 (config/swerve_params.yaml, arm_params.yaml)
# [출력]  IMU (sensor_msgs/Imu)
#         엔코더·관절 상태 (sensor_msgs/JointState)
#         보드 상태·에러 토픽
# [해야 할 일 / TODO]
#   - 두 보드와 연결을 연다
#   - 명령을 보드 형식으로 바꿔 보내고, 상태를 받아 토픽으로 발행한다
#   - 하드웨어 에러 플래그(과전류·과열 등)를 읽어 보고한다
# [확인 필요] 통신 방식: micro-ROS / 커스텀 시리얼 프로토콜 — 결정 후 구현
# [안전]  통신 타임아웃 → 정지 명령
#         하드웨어 에러 플래그 확인 → 이상 시 정지·보고
# =============================================================

import rclpy
from rclpy.node import Node


class OpenCRBridgeNode(Node):
    def __init__(self):
        super().__init__('opencr_bridge_node')
        # TODO(제어): 포트·통신 파라미터 선언
        # TODO(제어): SwerveCommand·팔 명령 구독자 생성
        # TODO(제어): IMU·JointState 퍼블리셔 생성
        # TODO(제어): 통신 감시 타이머 생성
        self.get_logger().info('opencr_bridge_node started (skeleton)')
        # TODO(제어): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = OpenCRBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
