# =============================================================
# 패키지 : nz2_perception
# 파일   : traffic_light_node.py
# 레이어 : perception
# 역할   : 신호등 색을 인식해 출발/정지 판단 근거를 만든다
#
# [입력]  /camera/color/image_raw (sensor_msgs/Image)
#         params: 신호등 ROI(영상 영역), 색별 HSV 임계값
# [출력]  /nz2/traffic_light (nz2_interfaces/TrafficLight)
# [해야 할 일 / TODO]
#   - ROI 안에서 빨강/노랑/초록 영역을 색 분할로 찾는다
#   - 여러 프레임 연속으로 같은 색일 때만 상태를 바꾼다 (깜빡임 방지)
#   - 색과 동작(출발/정지)의 대응은 대회 규칙 확정 후 [가정]
# =============================================================

import rclpy
from rclpy.node import Node


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        # TODO(인지): 컬러 영상 구독자 생성
        # TODO(인지): /nz2/traffic_light 퍼블리셔 생성
        self.get_logger().info('traffic_light_node started (skeleton)')
        # TODO(인지): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
