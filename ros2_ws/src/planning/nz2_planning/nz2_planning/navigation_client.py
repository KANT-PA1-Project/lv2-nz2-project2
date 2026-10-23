# =============================================================
# 패키지 : nz2_planning
# 파일   : navigation_client.py
# 레이어 : planning
# 역할   : (라이브러리) Nav2 NavigateToPose 액션 클라이언트 래퍼
#
# [입력]  목표 자세 (geometry_msgs/PoseStamped)
# [출력]  주행 결과 (성공/실패), 진행 피드백
# [해야 할 일 / TODO]
#   - navigate_to_pose 액션 클라이언트 생성
#   - goal 전송, 피드백 수신, 결과 처리
#   - 취소(cancel) 지원 — ESTOP 때 사용
# [안전]  ESTOP이면 진행 중인 goal을 즉시 취소한다
# [ROS2 메모] Nav2 = ROS 2 표준 자율주행 스택. 목표 위치만 주면 경로 계획·추종을 해 준다
# =============================================================


class NavigationClient:
    """Nav2 NavigateToPose 액션을 감싸는 클래스."""

    def __init__(self, node):
        # TODO(판단): rclpy.action.ActionClient(node, NavigateToPose, 'navigate_to_pose') 생성
        self._node = node

    def go_to(self, pose_stamped):
        """목표 자세로 이동을 요청한다."""
        raise NotImplementedError('TODO(판단): goal 전송')

    def cancel(self):
        """진행 중인 이동을 취소한다."""
        raise NotImplementedError('TODO(판단): goal 취소')
