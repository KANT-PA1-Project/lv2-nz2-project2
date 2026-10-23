# =============================================================
# 패키지 : nz2_planning
# 파일   : arm_client.py
# 레이어 : planning
# 역할   : (라이브러리) PickPlace 액션 클라이언트 래퍼 (control의 arm_controller_node 호출)
#
# [입력]  집을 퍽 (nz2_interfaces/Puck), 놓을 선반 ID
# [출력]  파지·배치 결과 (성공/실패), 진행 피드백
# [해야 할 일 / TODO]
#   - pick_place 액션 클라이언트 생성
#   - goal 전송, 피드백 수신, 결과 처리
#   - 취소 지원
# [안전]  ESTOP이면 진행 중인 goal을 즉시 취소한다
# =============================================================


class ArmClient:
    """PickPlace 액션을 감싸는 클래스."""

    def __init__(self, node):
        # TODO(판단): rclpy.action.ActionClient(node, PickPlace, 'pick_place') 생성
        self._node = node

    def pick_place(self, puck, shelf_id):
        """퍽을 집어 선반에 놓도록 요청한다."""
        raise NotImplementedError('TODO(판단): goal 전송')

    def cancel(self):
        """진행 중인 동작을 취소한다."""
        raise NotImplementedError('TODO(판단): goal 취소')
