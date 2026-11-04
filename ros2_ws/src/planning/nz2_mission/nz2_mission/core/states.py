"""미션 상태 정의와 크래시 재시작(RESUME) 재진입 규칙.

상위: IDLE → INIT → OBSERVE → DECODE → SURVEY ⇄ VALIDATE/PLAN → EXECUTE → VERIFY
      → RETURN → HOLD → DONE   (전역: SAFE_STOP, ESTOP / 재시작: RESUME)
"""
from enum import Enum


class Phase(str, Enum):
    IDLE = "IDLE"            # 시작 명령 대기. 신호 카메라 구독 금지(사전 관측 금지)
    INIT = "INIT"            # 자체 점검, 이동자세·출발 위치 확인
    OBSERVE = "OBSERVE"      # LOCATE → LOCK → SYNC → CAPTURE → VOTE (+바코드)
    DECODE = "DECODE"        # 횟수·검증·정렬·L1~L3 배정
    SURVEY = "SURVEY"        # 박스 탐색(퍽 ID·색·모양·신뢰도)
    VALIDATE = "VALIDATE"    # ID 중복·누락 → 실패 시 SURVEY 재탐색
    PLAN = "PLAN"            # 할당 + 운반 계획 → 해 없음 시 SURVEY 재탐색
    EXECUTE = "EXECUTE"      # 이동/줍기/싣기/놓기 (ExecSub)
    VERIFY = "VERIFY"        # 3곳 새 촬영 판정, 장소당 보정 1회
    RETURN = "RETURN"        # 출발 구역 복귀 (이동자세)
    HOLD = "HOLD"            # 3초 정지
    DONE = "DONE"            # 회차 종료 (재개 불가)
    SAFE_STOP = "SAFE_STOP"  # 치명 오류·중단 명령
    ESTOP = "ESTOP"          # 하드웨어 비상정지
    RESUME = "RESUME"        # 재시작 직후 재진입점 계산


class ObserveSub(str, Enum):
    LOCATE = "LOCATE"
    LOCK = "LOCK"
    SYNC = "SYNC"
    CAPTURE = "CAPTURE"
    VOTE = "VOTE"


class ExecSub(str, Enum):
    TRAVEL_POSE = "TRAVEL_POSE"      # 이동자세 확인(이동 가드)
    MOVE_TO_PICK = "MOVE_TO_PICK"    # 이동
    ALIGN_PICK = "ALIGN_PICK"
    PICK = "PICK"                    # 줍기
    LOAD = "LOAD"                    # 싣기(트레이)
    MOVE_TO_PLACE = "MOVE_TO_PLACE"  # 이동
    ALIGN_PLACE = "ALIGN_PLACE"
    UNLOAD = "UNLOAD"                # 놓기: 트레이에서 꺼내기
    PLACE = "PLACE"                  # 놓기: 격자점에 내려놓기
    CHECK_PLACE = "CHECK_PLACE"      # 놓기: 오차·이웃 밀림 확인
    RECOVER = "RECOVER"              # 떨어진 퍽 회수


TERMINAL = {Phase.DONE, Phase.SAFE_STOP, Phase.ESTOP}

# 하위 상태 → 묶음(이동/줍기/싣기/놓기) : 관제 표시·로그용
EXEC_GROUP = {
    "TRAVEL_POSE": "이동", "MOVE_TO_PICK": "이동", "ALIGN_PICK": "이동",
    "MOVE_TO_PLACE": "이동", "ALIGN_PLACE": "이동",
    "PICK": "줍기", "LOAD": "싣기", "UNLOAD": "놓기", "PLACE": "놓기", "CHECK_PLACE": "놓기",
    "RECOVER": "줍기",
}


def resume_entry(phase: str, signal_confirmed: bool, barcode_confirmed: bool) -> Phase:
    """크래시 직전 상태 → 안전한 재진입 상태. 물리 상태(그리퍼·위치)는 재진입 후 다시 확인한다."""
    p = Phase(phase)
    if p in TERMINAL:
        return p
    if p in (Phase.IDLE,):
        return Phase.IDLE
    if p in (Phase.INIT, Phase.OBSERVE):
        return Phase.DECODE if (signal_confirmed and barcode_confirmed) else Phase.OBSERVE
    if p == Phase.DECODE:
        return Phase.DECODE
    if p in (Phase.SURVEY, Phase.VALIDATE):
        return Phase.VALIDATE
    if p in (Phase.PLAN, Phase.EXECUTE):
        return Phase.PLAN          # 그리퍼·트레이 확인 후 재계획(놓인 퍽 고정)
    if p == Phase.VERIFY:
        return Phase.VERIFY
    if p in (Phase.RETURN, Phase.HOLD):
        return Phase.RETURN
    return Phase.SAFE_STOP
