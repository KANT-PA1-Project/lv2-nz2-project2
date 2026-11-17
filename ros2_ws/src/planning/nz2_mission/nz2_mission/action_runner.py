"""비차단 액션 실행기. FSM 타이머(10Hz)에서 poll()로 상태를 확인한다.

콜백 안에서 결과를 블로킹 대기하면 노드 전체가 멈추므로 금지 → Future를 매 틱 확인.
"""
import time
from typing import Any, Optional, Tuple

from action_msgs.msg import GoalStatus

Outcome = Tuple[bool, Any, str]   # (성공, 결과 메시지, 오류 코드)


class ActionRunner:
    def __init__(self, client, on_feedback=None):
        self.client = client
        self.on_feedback = on_feedback
        self._reset()

    def _reset(self):
        self.phase = "IDLE"
        self._send_fut = self._res_fut = self._gh = None
        self._t0 = 0.0
        self._timeout = 0.0
        self.outcome: Optional[Outcome] = None

    def busy(self) -> bool:
        return self.phase in ("SENDING", "RUNNING")

    def start(self, goal, timeout_s: float) -> None:
        self._reset()
        self._t0, self._timeout = time.monotonic(), timeout_s
        if not self.client.server_is_ready():
            self._finish(False, None, "SERVER_UNAVAILABLE")
            return
        self._send_fut = self.client.send_goal_async(goal, feedback_callback=self._fb)
        self.phase = "SENDING"

    def _fb(self, msg):
        if self.on_feedback:
            self.on_feedback(msg.feedback)

    def _finish(self, ok, res, err):
        self.phase, self.outcome = "DONE", (ok, res, err)

    def poll(self) -> Optional[Outcome]:
        if self.phase == "SENDING" and self._send_fut.done():
            gh = self._send_fut.result()
            if gh is None or not gh.accepted:
                self._finish(False, None, "REJECTED")
            else:
                self._gh, self._res_fut, self.phase = gh, gh.get_result_async(), "RUNNING"
        if self.phase == "RUNNING" and self._res_fut.done():
            r = self._res_fut.result()
            ok = r.status == GoalStatus.STATUS_SUCCEEDED
            self._finish(ok, r.result, "" if ok else f"STATUS_{r.status}")
        if self.busy() and time.monotonic() - self._t0 > self._timeout:
            self.cancel()
            self._finish(False, None, "TIMEOUT")
        return self.outcome if self.phase == "DONE" else None

    def cancel(self) -> None:
        if self._gh is not None and self.phase == "RUNNING":
            self._gh.cancel_goal_async()


class WaitRunner:
    """로컬 대기 단계(WAIT)를 액션과 같은 인터페이스로."""

    def __init__(self, seconds: float):
        self._t_end = time.monotonic() + seconds

    def poll(self) -> Optional[Outcome]:
        return (True, None, "") if time.monotonic() >= self._t_end else None

    def cancel(self):
        pass
