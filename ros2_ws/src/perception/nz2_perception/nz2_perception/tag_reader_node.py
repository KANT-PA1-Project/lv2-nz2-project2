# =============================================================
# 패키지 : nz2_perception
# 파일   : tag_reader_node.py
# 레이어 : perception
# 역할   : AprilTag·바코드를 읽어 과제 순서와 위치를 알아낸다
#
# [입력]  /camera/color/image_raw (sensor_msgs/Image)
#         params: 태그 family, 바코드 종류
# [출력]  /nz2/tags (vision_msgs 또는 간단한 msg — 구현 때 결정)
# [해야 할 일 / TODO]
#   - AprilTag 라이브러리(apriltag)로 태그 ID·자세를 검출한다
#   - zbar(pyzbar)로 바코드 문자열을 읽는다
#   - 읽은 값이 무엇을 뜻하는지(순서/위치)는 대회 규칙 확정 후 mission.yaml과 맞춘다 [가정]
# =============================================================

import rclpy
from rclpy.node import Node


class TagReaderNode(Node):
    def __init__(self):
        super().__init__('tag_reader_node')
        # TODO(인지): 컬러 영상 구독자 생성
        # TODO(인지): /nz2/tags 퍼블리셔 생성
        self.get_logger().info('tag_reader_node started (skeleton)')
        # TODO(인지): 실제 로직은 이후 이슈에서 구현


def main(args=None):
    rclpy.init(args=args)
    node = TagReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
