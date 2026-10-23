# =============================================================
# 패키지 : nz2_planning
# 파일   : puck_sequencer.py
# 레이어 : planning
# 역할   : (라이브러리) 인식된 퍽을 어떤 순서로 옮길지 정한다
#
# [입력]  인식된 퍽 목록 (Puck 리스트)
#         규칙: config/mission.yaml의 색 순서
# [출력]  처리 순서대로 정렬한 퍽 리스트
# [해야 할 일 / TODO]
#   - mission.yaml의 색 순서대로 정렬한다
#   - 같은 색이 여러 개면 가까운 것부터 등 기준을 정한다
#   - 이미 옮긴 퍽은 제외한다
# [ROS2 메모] 노드가 아닌 일반 Python 모듈. 노드(mission_manager_node)가 import 해서 쓴다
# =============================================================


def sequence_pucks(pucks, color_order):
    """퍽 목록을 처리 순서대로 정렬해 돌려준다.

    pucks: nz2_interfaces/Puck 리스트
    color_order: 예) ['red', 'green', 'blue']
    """
    raise NotImplementedError('TODO(판단): 퍽 처리 순서 결정')
