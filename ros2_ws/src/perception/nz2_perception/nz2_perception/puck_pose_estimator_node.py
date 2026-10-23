# =============================================================
# 패키지 : nz2_perception
# 파일   : puck_pose_estimator_node.py
# 레이어 : perception
# 역할   : 퍽 모양(원통/직육면체)과 자세(세움/누움)를 판별하고 파지점을 계산한다
#
# [입력]  /nz2/pucks (nz2_interfaces/PuckArray) — puck_detector_node 결과
#         깊이 영상 또는 포인트클라우드 (토픽은 config에서 지정)
# [출력]  /nz2/pucks (nz2_interfaces/PuckArray) — shape·orientation·pose가 보강된 결과
#         ※ 같은 토픽 이름을 쓸지, /nz2/pucks_refined 처럼 나눌지 구현 때 결정
# [해야 할 일 / TODO]
#   - 퍽 주변 깊이 점들로 높이·폭을 재서 세움/누움을 판별한다
#   - 윗면 모양(원/사각형)으로 원통/직육면체를 판별한다
#   - 그리퍼가 잡을 위치·방향(파지점)을 pose에 넣는다
#   - 퍼블리셔/서브스크라이버 연결
# [ROS2 메모] 한 노드의 출력을 다른 노드가 받아 덧붙이는 "파이프라인" 구조
# =============================================================

import rclpy
from rclpy.node import Node


class PuckPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('puck_pose_estimator_node')
        # TODO(인지): /nz2/pucks 구독자 생성
        # TODO(인지): 깊이 구독자 생성
        # TODO(인지): 보강된 PuckArray 퍼블리셔 생성
        self.get_logger().info('puck_pose_estimator_node started (skeleton)')
        # TODO(인지): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = PuckPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
