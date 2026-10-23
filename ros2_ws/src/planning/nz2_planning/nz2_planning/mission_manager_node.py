# =============================================================
# 패키지 : nz2_planning
# 파일   : mission_manager_node.py
# 레이어 : planning
# 역할   : 미션 전체 상태기계(FSM). 퍽 순서대로 "탐지 → 이동 → 파지 → 이동 → 배치"를 반복한다
#
# [입력]  /nz2/pucks (nz2_interfaces/PuckArray)
#         /nz2/tags
#         /nz2/traffic_light (nz2_interfaces/TrafficLight)
#         params: config/mission.yaml
# [출력]  Nav2 NavigateToPose 액션 호출 (navigation_client)
#         PickPlace 액션 호출 (arm_client)
#         상태 로그
# [해야 할 일 / TODO]
#   - 상태 정의: IDLE / SCAN / NAV / PICK / PLACE / DONE / ESTOP
#   - 상태 전이 조건 작성 (신호등·퍽 인식·액션 결과)
#   - 퍽 처리 순서는 puck_sequencer로 정한다
#   - 실격 회피 체크: 시작·종료 400×400mm 수납, 벽·선반 접촉 금지 [가정: 세부 규칙 확정 후 갱신]
# [안전]  접촉·위치 오차 감지 시 ESTOP 상태로 가서 주행·팔 액션을 모두 취소한다
# [ROS2 메모] FSM(상태기계) = 지금 상태와 조건에 따라 다음 상태를 정하는 구조
# =============================================================

import rclpy
from rclpy.node import Node


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')
        # TODO(판단): 파라미터(mission.yaml) 선언
        # TODO(판단): /nz2/pucks·/nz2/tags·/nz2/traffic_light 구독자 생성
        # TODO(판단): NavigationClient·ArmClient 생성
        # TODO(판단): 상태기계 주기 타이머 생성
        self.get_logger().info('mission_manager_node started (skeleton)')
        # TODO(판단): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
