# =============================================================
# 패키지 : nz2_control
# 파일   : arm_controller_node.py
# 레이어 : control
# 역할   : PickPlace 액션 서버. 팔 경로를 받아 OpenCR #2로 관절 명령을 보낸다
#
# [입력]  pick_place 액션 goal (nz2_interfaces/PickPlace) — planning의 arm_client가 보냄
#         params: config/arm_params.yaml
# [출력]  팔 관절 명령 → opencr_bridge_node
#         PickPlace 피드백(진행률)·결과
# [해야 할 일 / TODO]
#   - pick_place 액션 서버 생성
#   - MoveIt 2로 접근 → 파지 → 들기 → 배치 경로를 계획한다
#   - 계획한 관절 궤적을 브리지로 보내고 진행률을 피드백한다
#   - 취소(cancel) 요청 처리
# [안전]  관절 범위·전류 제한 (arm_params.yaml)
#         실패·취소 시 팔 정지, 이상 시 토크 OFF
# [ROS2 메모] 액션 서버 = 오래 걸리는 작업을 받아 진행 상황을 알려 주며 수행하는 쪽
# =============================================================

import rclpy
from rclpy.node import Node


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        # TODO(제어): arm_params.yaml 파라미터 선언
        # TODO(제어): pick_place 액션 서버 생성
        # TODO(제어): 관절 명령 퍼블리셔 생성
        self.get_logger().info('arm_controller_node started (skeleton)')
        # TODO(제어): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
