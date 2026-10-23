# =============================================================
# 패키지 : nz2_control
# 파일   : swerve_controller_node.py
# 레이어 : control
# 역할   : /cmd_vel을 받아 스월브 4모듈 명령으로 바꾼다
#
# [입력]  /cmd_vel (geometry_msgs/Twist) — linear.x, linear.y, angular.z 사용
#         params: config/swerve_params.yaml
# [출력]  /nz2/swerve_command (nz2_interfaces/SwerveCommand) → opencr_bridge_node
# [해야 할 일 / TODO]
#   - /cmd_vel 수신 → swerve_kinematics.inverse_kinematics 호출
#   - 결과를 SwerveCommand(FL, FR, RL, RR)로 발행
#   - 최대 속도·가속 제한 적용
# [미정] 역기구학을 여기(Jetson)서 할지 OpenCR #1에서 할지 미정
#         Jetson: 이 노드가 SwerveCommand 발행 / OpenCR: 이 노드는 Twist를 그대로 브리지로 넘김
#         → swerve_params.yaml의 kinematics_on_jetson 값으로 두 경우를 나눈다
# [안전]  최대 속도·가속 제한
#         /cmd_vel이 timeout_s 동안 안 오면 0 속도(정지) 명령 발행
# [ROS2 메모] Twist = 선속도(linear)와 각속도(angular)를 담는 표준 속도 메시지
# =============================================================

import rclpy
from rclpy.node import Node


class SwerveControllerNode(Node):
    def __init__(self):
        super().__init__('swerve_controller_node')
        # TODO(제어): swerve_params.yaml 파라미터 선언
        # TODO(제어): /cmd_vel 구독자 생성
        # TODO(제어): /nz2/swerve_command 퍼블리셔 생성
        # TODO(제어): 명령 타임아웃 감시 타이머 생성
        self.get_logger().info('swerve_controller_node started (skeleton)')
        # TODO(제어): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
