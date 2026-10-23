# =============================================================
# 패키지 : nz2_perception
# 파일   : puck_detector_node.py
# 레이어 : perception
# 역할   : RealSense 컬러·깊이 영상에서 빨강·초록·파랑 퍽을 찾는다
#
# [입력]  /camera/color/image_raw (sensor_msgs/Image)
#         /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
#         params: 색별 HSV 임계값, 최소 면적, 신뢰도 임계값 (config/perception_params.yaml)
# [출력]  /nz2/pucks (nz2_interfaces/PuckArray) — 색·3D 위치·신뢰도
# [해야 할 일 / TODO]
#   - 컬러 영상을 HSV로 바꿔 색별로 분할(threshold)한다
#   - 윤곽(contour)을 찾아 면적·모양으로 퍽 후보를 고른다
#   - 같은 위치의 깊이 값으로 카메라 좌표 3D 위치를 구한다
#   - PuckArray로 묶어 /nz2/pucks에 발행한다 (모양·자세는 puck_pose_estimator가 채움)
# [ROS2 메모] 노드 = 실행 중인 프로그램 1개. 토픽 = 노드끼리 데이터를 계속 흘려보내는 통로
# =============================================================

import rclpy
from rclpy.node import Node


class PuckDetectorNode(Node):
    def __init__(self):
        super().__init__('puck_detector_node')
        # TODO(인지): 파라미터 선언 (토픽 이름, HSV 임계값)
        # TODO(인지): 컬러·깊이 구독자 생성 (두 영상 시각 맞추기: message_filters)
        # TODO(인지): /nz2/pucks 퍼블리셔 생성
        self.get_logger().info('puck_detector_node started (skeleton)')
        # TODO(인지): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = PuckDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
