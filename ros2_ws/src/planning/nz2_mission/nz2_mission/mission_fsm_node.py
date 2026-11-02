"""미션 FSM 노드 (Jetson, 판단 계층). 설계: docs/mission_fsm_design.md

IDLE → INIT → OBSERVE → DECODE → SURVEY ⇄ VALIDATE/PLAN → EXECUTE → VERIFY → RETURN → HOLD → DONE
전역: SAFE_STOP(치명 오류·중단 명령) / ESTOP(하드웨어) / RESUME(크래시 재시작 재진입)

원칙
- 월드 모델(퍽 12개)의 주인은 이 노드다. 인지 노드는 관측만 보낸다.
- 이동(Nav2) 직전에는 반드시 이동자세(travel_pose) 성공이 있어야 한다(가드).
- 퍽을 실은 채로 SURVEY에 가지 않는다. 재계획은 트레이 내용을 시작 조건으로 쓴다.
- 경기 중 관제 PC 명령은 시작·중단뿐(설정 변경은 IDLE에서만). Wi-Fi가 끊겨도 미션은 계속.
- 상태 전이·단계 완료마다 체크포인트 저장 → respawn 후 RESUME으로 이어서 진행.
"""
import json
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger

from nz2_interfaces.action import ArmSkill
from nz2_interfaces.msg import PuckObservationArray
from nz2_interfaces.srv import SetMode

from .action_runner import ActionRunner, WaitRunner
from .core import checkpoint
from .core.allocator import Allocation, allocate, allocate_partial, diagnose
from .core.dice_layout import cell_world_mm
from .core.mission_spec import GroupAssignment, SpecError, assign_locations
from .core.planner import SiteInfo, Step, plan_steps
from .core.states import EXEC_GROUP, TERMINAL, Phase, resume_entry
from .core.world_model import PuckStatus, WorldModel

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

MODES = ("competition", "rehearsal", "dry_signal", "dry_run")
BEEP_PHASE, BEEP_MILESTONE, BEEP_RECOVERABLE, BEEP_SAFE_STOP = 1, 2, 3, 4
NAV_SUBS = ("MOVE_TO_PICK", "MOVE_TO_PLACE", "MOVE")
SKILL_OF = {"TRAVEL_POSE": "travel_pose", "ALIGN_PICK": "align", "ALIGN_PLACE": "align", "PICK": "pick",
            "LOAD": "load", "UNLOAD": "unload", "PLACE": "place", "CHECK_PLACE": "check_place",
            "RECOVER": "recover_dropped", "OBSERVE_BOX": "observe_box", "VERIFY_LOC": "verify_location",
            "FIX_PLACE": "fix_location", "FIX_LOC": "fix_location", "HOLD_ARM": "hold"}


class MissionFsm(Node):
    def __init__(self):
        super().__init__("mission_fsm")
        d = self.declare_parameter
        d("mode", "competition")
        d("run_root", os.path.expanduser("~/nz2_runs"))
        d("resume_enabled", True)
        d("locations_file", "")
        d("tick_hz", 10.0)
        d("barcode_map", ["P2-01:asc", "P2-03:asc", "P2-02:desc", "P2-04:desc"])
        d("expected_ids", list(range(12)))
        d("tray_capacity", 4)
        d("min_conf", 0.7)
        d("survey_max_rounds", 3)
        d("max_plans", 20)
        d("verify_fix_attempts", 1)
        d("place_tol_mm", 10.0)
        d("hold_s", 3.0)
        d("observe_settle_s", 1.0)
        d("no_progress_s", 30.0)                    # 심판 "동작 불능" 기준보다 짧게 [확인]
        d("observe_timeout_s", 300.0)
        d("observe_call_retry_s", 2.0)
        d("barcode_retries", 2)
        d("timeout_nav_s", 60.0)
        d("timeout_arm_s", 20.0)
        d("timeout_init_s", 30.0)
        d("retries_nav", 2)
        d("retries_arm", 2)
        d("retries_return", 5)
        d("place_surface_z_m", 0.070)
        d("arena_to_map", [0.0, 0.0, 0.0])          # [tx_mm, ty_mm, yaw_rad] 경기장 좌표 → map [확인]
        d("map_frame", "map")
        d("publish_initialpose", True)
        d("gpu_lifecycle_nodes", [""])              # OBSERVE 동안 deactivate할 lifecycle 노드 이름
        d("require_arm_button", False)
        d("dry_signal_sequence", "RGRRGRGGYGYG")
        d("dry_barcode", "P2-01")
        d("signal_start_srv", "/signal_decoder/start")
        d("barcode_start_srv", "/barcode_reader/start")

        self.mode = self.p("mode")
        if self.mode not in MODES:
            raise ValueError(f"알 수 없는 mode: {self.mode}")
        self._load_sites()

        # 통신
        self.pub_phase = self.create_publisher(String, "/state/mission_phase", LATCHED)
        self.pub_detail = self.create_publisher(String, "/state/mission_detail", BEST_EFFORT)
        self.pub_hb = self.create_publisher(String, "/mission/heartbeat", BEST_EFFORT)
        self.pub_log = self.create_publisher(String, "/mission/log", 50)
        self.pub_beep = self.create_publisher(UInt8, "/ui/beep", 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/abort", self._srv_abort)
        self.create_service(SetMode, "~/set_mode", self._srv_set_mode)
        self.create_subscription(String, "/mission/signal_result", self._on_signal, LATCHED)
        self.create_subscription(String, "/mission/barcode_result", self._on_barcode, LATCHED)
        self.create_subscription(PuckObservationArray, "/mission/puck_observations", self._on_obs, 20)
        self.create_subscription(Bool, "/state/estop", self._on_estop, 10)
        self.create_subscription(String, "/state/hw_error", self._on_hw_error, 10)
        self.create_subscription(Bool, "/ui/arm_button", self._on_arm_button, 10)
        self.nav = ActionRunner(ActionClient(self, NavigateToPose, "navigate_to_pose"), self._on_feedback)
        self.arm = ActionRunner(ActionClient(self, ArmSkill, "arm_skill"), self._on_feedback)
        self.cli_sig = self.create_client(Trigger, self.p("signal_start_srv"))
        self.cli_bar = self.create_client(Trigger, self.p("barcode_start_srv"))

        self._new_state()
        self.armed = not self.p("require_arm_button")
        self._try_resume()
        self.create_timer(1.0 / self.p("tick_hz"), self._tick)
        self.create_timer(0.5, self._publish_detail)
        self.create_timer(1.0, lambda: self.pub_hb.publish(String(data=json.dumps(
            {"t": time.time(), "phase": self.phase.value, "mode": self.mode}))))
        self._publish_phase()
        self.get_logger().info(f"mission_fsm 준비 (mode={self.mode}, phase={self.phase.value})")

    # ================================================================== 기본
    def p(self, n):
        return self.get_parameter(n).value

    def _new_state(self):
        self.phase = Phase.IDLE
        self.entry = True
        self.run_id = ""
        self.run_dir = ""
        self.t_start_cmd = None
        self.signal = None           # 확정 신호 dict
        self.barcode = None          # 확정 바코드 dict
        self.groups: List[GroupAssignment] = []
        self.wm = WorldModel(list(self.p("expected_ids")), self.p("tray_capacity"), self.p("min_conf"))
        self.alloc: Optional[Allocation] = None
        self.steps: List[Step] = []
        self.si = 0
        self.retry: Dict[int, int] = {}
        self.survey_round = 0
        self.survey_targets: List[int] = []
        self.plan_count = 0
        self.partial = False
        self.verify_results: Dict = {}
        self.fail_reason = ""
        self.last_error = ""
        self.op = None
        self.travel_ok = False
        self.replan_pending = False
        self.robot_mm = list(self.sites["start_pose"][:2])
        self.t_progress = time.monotonic()
        self.obs_requested = False
        self.sig_called = False
        self.bar_called = False
        self.t_obs = 0.0
        self.t_obs_call = 0.0
        self.barcode_tries = 0
        self.initpose_sent = False
        self.estop = False

    def _load_sites(self):
        path = self.p("locations_file")
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"locations_file 없음: {path}")
        with open(path, encoding="utf-8") as f:
            y = yaml.safe_load(f)
        self.sites = {"start_pose": y["start_pose"]}
        self.boxes = {int(k): SiteInfo(tuple(v["center"]), tuple(v["approach"])) for k, v in y["boxes"].items()}
        self.locs = {k: SiteInfo(tuple(v["center"]), tuple(v["approach"]), v.get("place_order"))
                     for k, v in y["locations"].items()}

    def _log(self, event: str, **kw):
        rec = {"t": time.time(), "phase": self.phase.value, "event": event, **kw}
        self.pub_log.publish(String(data=json.dumps(rec, ensure_ascii=False)))
        if self.run_dir:
            with open(os.path.join(self.run_dir, "events.jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.get_logger().info(f"[{self.phase.value}] {event} {kw if kw else ''}")

    def _beep(self, pattern: int):
        self.pub_beep.publish(UInt8(data=pattern))

    def _enter(self, phase: Phase, reason: str = ""):
        if self.phase in TERMINAL and phase != self.phase:
            return
        prev = self.phase
        self.phase, self.entry = phase, True
        self.t_progress = time.monotonic()
        if phase == Phase.SAFE_STOP:
            self.fail_reason = reason
            self._cancel_ops()
            self._beep(BEEP_SAFE_STOP)
        elif phase == Phase.ESTOP:
            self.fail_reason = reason
            self._cancel_ops()
            self._beep(BEEP_SAFE_STOP)
        else:
            self._beep(BEEP_PHASE)
        self._log("phase", prev=prev.value, to=phase.value, reason=reason)
        self._save()
        self._publish_phase()

    def _publish_phase(self):
        self.pub_phase.publish(String(data=self.phase.value))

    def _cancel_ops(self):
        for r in (self.nav, self.arm):
            r.cancel()
        self.op = None

    # ================================================================== 체크포인트
    def _cp_path(self):
        return os.path.join(self.run_dir, "checkpoint.json")

    def _save(self):
        if not self.run_dir:
            return
        checkpoint.save_atomic(self._cp_path(), {
            "run_id": self.run_id, "mode": self.mode, "phase": self.phase.value, "t_start_cmd": self.t_start_cmd,
            "signal": self.signal, "barcode": self.barcode, "groups": [g.to_dict() for g in self.groups],
            "world": self.wm.to_dict(), "alloc": self.alloc.to_dict() if self.alloc else None,
            "steps": [s.to_dict() for s in self.steps], "si": self.si, "survey_round": self.survey_round,
            "survey_targets": self.survey_targets, "plan_count": self.plan_count, "partial": self.partial,
            "verify_results": self.verify_results, "fail_reason": self.fail_reason,
            "robot_mm": self.robot_mm, "barcode_tries": self.barcode_tries})

    def _try_resume(self):
        if not self.p("resume_enabled"):
            return
        ptr = checkpoint.load(os.path.join(self.p("run_root"), "current.json"))
        if not ptr:
            return
        self.run_dir = os.path.join(self.p("run_root"), ptr["run_id"])
        cp = checkpoint.load(self._cp_path())
        if not cp or Phase(cp["phase"]) == Phase.IDLE:
            self.run_dir = ""
            return
        self.run_id, self.mode, self.t_start_cmd = cp["run_id"], cp["mode"], cp["t_start_cmd"]
        self.signal, self.barcode = cp["signal"], cp["barcode"]
        self.groups = [GroupAssignment(**g) for g in cp["groups"]]
        self.wm = WorldModel.from_dict(cp["world"])
        self.alloc = Allocation.from_dict(cp["alloc"]) if cp["alloc"] else None
        self.survey_round, self.survey_targets = cp["survey_round"], cp["survey_targets"]
        self.plan_count, self.partial = cp["plan_count"], cp["partial"]
        self.verify_results, self.fail_reason = cp["verify_results"], cp["fail_reason"]
        self.robot_mm, self.barcode_tries = cp["robot_mm"], cp["barcode_tries"]
        target = resume_entry(cp["phase"], self.signal is not None, self.barcode is not None)
        self.phase = Phase(cp["phase"])
        if target in TERMINAL:
            self.phase, self.entry = target, False
            self.get_logger().warn(f"이전 회차 종료 상태({target.value}) — 재개하지 않음")
            return
        self._log("resume", from_phase=cp["phase"], to=target.value)
        self._enter(target, "resume")

    # ================================================================== 서비스·구독
    def _srv_start(self, req, res):
        if self.phase != Phase.IDLE:
            res.success, res.message = True, f"이미 시작됨({self.phase.value})"      # 멱등
            return res
        if not self.armed:
            res.success, res.message = False, "ARM(준비 확인) 전"
            return res
        self.run_id = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.p("run_root"), self.run_id)
        os.makedirs(self.run_dir, exist_ok=True)
        checkpoint.save_atomic(os.path.join(self.p("run_root"), "current.json"), {"run_id": self.run_id})
        self.t_start_cmd = time.time()
        self._log("start_command", mode=self.mode)
        self._enter(Phase.INIT, "start")
        res.success, res.message = True, self.run_id
        return res

    def _srv_abort(self, req, res):
        if self.phase in TERMINAL or self.phase == Phase.IDLE:
            res.success, res.message = True, f"중단할 것 없음({self.phase.value})"
            return res
        self._enter(Phase.SAFE_STOP, "operator_abort")
        res.success, res.message = True, "aborted"
        return res

    def _srv_set_mode(self, req, res):
        if self.phase != Phase.IDLE:
            res.success, res.message = False, "IDLE에서만 모드 변경 가능"
        elif req.mode not in MODES:
            res.success, res.message = False, f"모드는 {MODES}"
        else:
            self.mode = req.mode
            res.success, res.message = True, f"mode={self.mode}"
            self._log("set_mode", mode=self.mode)
        return res

    def _on_signal(self, msg: String):
        if self.phase != Phase.OBSERVE or not self.obs_requested:
            return   # 시작 전·이전 회차 래치 메시지 무시
        r = json.loads(msg.data)
        if r.get("status") == "confirmed":
            self.signal = r
            self._log("signal_confirmed", sequence=r["sequence"], counts=r["counts"],
                      first_image_stamp=r.get("first_image_stamp"), start_cmd=self.t_start_cmd)
            self._beep(BEEP_MILESTONE)
        else:
            self._enter(Phase.SAFE_STOP, f"signal_failed:{r.get('reason')}")

    def _on_barcode(self, msg: String):
        if self.phase != Phase.OBSERVE or not self.obs_requested or self.barcode is not None:
            return
        r = json.loads(msg.data)
        if r.get("status") == "confirmed":
            self.barcode = r
            self._log("barcode_confirmed", code=r["code"])
        elif r.get("status") == "unregistered":
            self._enter(Phase.SAFE_STOP, f"unregistered_barcode:{r.get('code')}")
        else:
            self.barcode_tries += 1
            if self.barcode_tries > self.p("barcode_retries"):
                self._enter(Phase.SAFE_STOP, "barcode_failed")
            else:
                self._log("barcode_retry", n=self.barcode_tries)
                self._call(self.cli_bar)

    def _on_obs(self, msg: PuckObservationArray):
        if self.phase not in (Phase.SURVEY, Phase.VALIDATE):
            return
        for o in msg.observations:
            self.wm.observe(o.puck_id, o.color or None, o.shape or None, o.conf_color, o.conf_shape,
                            o.box_id if o.box_id >= 0 else None, (o.x_mm, o.y_mm))

    def _on_estop(self, msg: Bool):
        if msg.data and not self.estop:
            self.estop = True
            self._enter(Phase.ESTOP, "hardware_estop")

    def _on_hw_error(self, msg: String):
        if msg.data and self.phase not in TERMINAL and self.phase != Phase.IDLE:
            self._enter(Phase.SAFE_STOP, f"hw_error:{msg.data}")

    def _on_arm_button(self, msg: Bool):
        if msg.data and self.phase == Phase.IDLE:
            self.armed = True
            self._log("armed")
            self._beep(BEEP_MILESTONE)

    def _on_feedback(self, fb):
        self.t_progress = time.monotonic()

    def _call(self, cli):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())
            return True
        self._log("service_unavailable", srv=cli.srv_name)
        return False

    def _lifecycle(self, activate: bool):
        for name in self.p("gpu_lifecycle_nodes"):
            if not name:
                continue
            cli = self.create_client(ChangeState, f"{name}/change_state")
            if cli.service_is_ready():
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE if activate else Transition.TRANSITION_DEACTIVATE
                cli.call_async(req)
            self._log("gpu_node", node=name, activate=activate, ready=cli.service_is_ready())

    # ================================================================== 좌표
    def _to_map(self, x_mm, y_mm, yaw=0.0):
        tx, ty, th = self.p("arena_to_map")
        c, s = math.cos(th), math.sin(th)
        return ((c * x_mm - s * y_mm + tx) / 1000.0, (s * x_mm + c * y_mm + ty) / 1000.0, yaw + th)

    def _pose(self, x_mm, y_mm, yaw=0.0, z=0.0) -> Pose:
        x, y, th = self._to_map(x_mm, y_mm, yaw)
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.z, p.orientation.w = math.sin(th / 2), math.cos(th / 2)
        return p

    # ================================================================== 메인 루프
    def _tick(self):
        if self.phase in TERMINAL:
            return
        if self.op is not None and time.monotonic() - self.t_progress > self.p("no_progress_s"):
            self._log("no_progress", step=self._cur_step_name())
            self.op.cancel()
            self.op = None
            self._handle_failure(self.steps[self.si] if self.si < len(self.steps) else Step("?"), None, "NO_PROGRESS")
            return
        getattr(self, f"_ph_{self.phase.value.lower()}")()

    def _first(self) -> bool:
        if self.entry:
            self.entry = False
            return True
        return False

    # ---------------------------------------------------------------- 상태별
    def _ph_idle(self):
        pass

    def _ph_resume(self):
        self._enter(Phase.PLAN, "resume")

    def _ph_init(self):
        if self._first():
            self.t_init = time.monotonic()
            self.steps, self.si, self.retry = [Step("TRAVEL_POSE")], 0, {}
        if time.monotonic() - self.t_init > self.p("timeout_init_s"):
            self._enter(Phase.SAFE_STOP, "init_timeout")
            return
        if self._run_steps() == "done":
            self._enter(Phase.OBSERVE)

    def _ph_observe(self):
        if self._first():
            self._lifecycle(False)       # OBSERVE 동안 GPU 작업 정지
            if self.mode in ("dry_signal", "dry_run"):
                seq = self.p("dry_signal_sequence")
                self.signal = {"status": "confirmed", "sequence": seq, "injected": True}
                self.barcode = {"status": "confirmed", "code": self.p("dry_barcode"), "injected": True}
                self._log("signal_injected", sequence=seq, barcode=self.barcode["code"])
            else:
                self.obs_requested, self.t_obs, self.t_obs_call = True, time.monotonic(), 0.0
                self.sig_called = self.signal is not None
                self.bar_called = self.barcode is not None
                self._log("observe_requested")
        if self.signal and self.barcode:
            self._enter(Phase.DECODE)
            return
        now = time.monotonic()
        if self.obs_requested and now - self.t_obs_call >= self.p("observe_call_retry_s"):
            self.t_obs_call = now           # 서비스가 아직 안 떠 있으면 주기적으로 재호출
            if not self.bar_called:
                self.bar_called = self._call(self.cli_bar)
            if not self.sig_called:
                self.sig_called = self._call(self.cli_sig)
        if self.obs_requested and now - self.t_obs > self.p("observe_timeout_s"):
            self._enter(Phase.SAFE_STOP, "observe_timeout")

    def _ph_decode(self):
        bmap = dict(s.split(":") for s in self.p("barcode_map"))
        try:
            self.groups = assign_locations(self.signal["sequence"], self.barcode["code"], bmap)
        except (SpecError, KeyError, TypeError) as e:
            self._enter(Phase.SAFE_STOP, f"decode:{e}")
            return
        self._log("assignment", groups=[g.to_dict() for g in self.groups])
        self.survey_targets = sorted(self.boxes)
        self._enter(Phase.SURVEY)

    def _ph_survey(self):
        if self._first():
            self._lifecycle(True)
            if self.p("publish_initialpose") and not self.initpose_sent:
                self._send_initialpose()
            self.steps, self.si, self.retry = self._survey_steps(self.survey_targets), 0, {}
            self._log("survey", round=self.survey_round, boxes=self.survey_targets)
        if self._run_steps() == "done":
            self._enter(Phase.VALIDATE)

    def _ph_validate(self):
        issues = self.wm.validate()
        if not issues:
            self._enter(Phase.PLAN)
            return
        self._log("validate_issues", issues=[i.__dict__ for i in issues])
        if self.survey_round >= self.p("survey_max_rounds"):
            self.partial = True
            self._enter(Phase.PLAN, "survey_limit_partial")
            return
        self._resurvey([i.pid for i in issues if i.pid is not None],
                       self.wm.boxes_to_resurvey(issues, sorted(self.boxes)))

    def _resurvey(self, pids, boxes):
        for pid in pids:      # 의심 퍽은 관측을 비우고 다시 본다(같은 오판 반복 방지는 인지 쪽 시점 변경)
            p = self.wm.pucks.get(pid)
            if p is not None and p.status == PuckStatus.IN_BOX:
                del self.wm.pucks[pid]
        self.wm.unexpected_ids = []
        self.survey_round += 1
        self.survey_targets = boxes
        self._enter(Phase.SURVEY, f"resurvey_{self.survey_round}")

    def _ph_plan(self):
        self.plan_count += 1
        if self.plan_count > self.p("max_plans"):
            self._enter(Phase.VERIFY, "plan_limit")
            return
        cost = lambda p, l: math.hypot(p.pos_mm[0] - self.locs[l].center_mm[0], p.pos_mm[1] - self.locs[l].center_mm[1]) \
            if p.pos_mm else 1e6
        alloc = None if self.partial else allocate(self.groups, self.wm, cost)
        if alloc is None:
            diag = diagnose(self.groups, self.wm)
            self._log("plan_no_solution", reasons=diag.reasons, suspects=diag.suspect_pids)
            if not self.partial and self.survey_round < self.p("survey_max_rounds"):
                boxes = sorted({self.wm.pucks[p].box for p in diag.suspect_pids
                                if p in self.wm.pucks and self.wm.pucks[p].box is not None}) or sorted(self.boxes)
                if not self.wm.carried():
                    self._resurvey(diag.suspect_pids, boxes)
                    return
            self.partial = True
            alloc = allocate_partial(self.groups, self.wm, cost)
            if alloc is None:
                self._enter(Phase.VERIFY, "no_feasible_location")
                return
        self.alloc = alloc
        try:
            self.steps = plan_steps(self.groups, alloc, self.wm, self.boxes, self.locs, self.robot_mm)
        except (RuntimeError, IndexError, StopIteration) as e:
            self._log("plan_error", error=str(e))
            self._enter(Phase.VERIFY, "plan_error")
            return
        self.si, self.retry, self.replan_pending = 0, {}, False
        self._log("plan", alloc=alloc.to_dict(), n_steps=len(self.steps), partial=self.partial)
        self._enter(Phase.EXECUTE)

    def _ph_execute(self):
        if self._first():
            pass
        if self._run_steps() == "done":
            self._enter(Phase.VERIFY)

    def _ph_verify(self):
        if self._first():
            self.steps, self.si, self.retry = self._verify_steps(), 0, {}
        if self._run_steps() == "done":
            self._log("verify_done", results=self.verify_results)
            self._enter(Phase.RETURN)

    def _ph_return(self):
        if self._first():
            sp = self.sites["start_pose"]
            self.steps = [Step("TRAVEL_POSE"), Step("MOVE", {"pose": list(sp), "kind": "return"})]
            self.si, self.retry = 0, {}
        if self._run_steps() == "done":
            self._enter(Phase.HOLD)

    def _ph_hold(self):
        if self._first():
            self.t_hold = time.monotonic()
        if time.monotonic() - self.t_hold >= self.p("hold_s"):
            self._log("finished", verify=self.verify_results, partial=self.partial)
            self._beep(BEEP_MILESTONE)
            self._enter(Phase.DONE, "returned")

    # ================================================================== 단계 생성
    def _survey_steps(self, targets: List[int]) -> List[Step]:
        steps, pos, left = [], tuple(self.robot_mm), list(targets)
        while left:   # 최근접 순서
            b = min(left, key=lambda k: math.hypot(self.boxes[k].approach[0] - pos[0], self.boxes[k].approach[1] - pos[1]))
            left.remove(b)
            steps += [Step("TRAVEL_POSE"), Step("MOVE", {"pose": list(self.boxes[b].approach), "kind": "survey", "box": b}),
                      Step("OBSERVE_BOX", {"box": b}), Step("WAIT", {"s": self.p("observe_settle_s")})]
            pos = self.boxes[b].approach[:2]
        return steps

    def _verify_steps(self) -> List[Step]:
        steps = []
        for g in self.groups:
            site = self.locs[g.location]
            steps += [Step("TRAVEL_POSE"), Step("MOVE", {"pose": list(site.approach), "kind": "verify", "loc": g.location}),
                      Step("VERIFY_LOC", {"loc": g.location, "pips": g.count, "fix_left": self.p("verify_fix_attempts")})]
        return steps

    # ================================================================== 단계 실행기
    def _cur_step_name(self):
        return self.steps[self.si].sub if self.si < len(self.steps) else "-"

    def _run_steps(self) -> str:
        if self.si >= len(self.steps):
            return "done"
        step = self.steps[self.si]
        if self.op is None:
            self._start_step(step)
            return "running"
        out = self.op.poll()
        if out is None:
            return "running"
        self.op = None
        ok, res, err = out
        if res is not None and hasattr(res, "success"):     # ArmSkill 결과
            ok = ok and res.success
            err = err or res.error_code
        if ok:
            self._on_step_ok(step, res)
        else:
            self._handle_failure(step, res, err or "FAILED")
        return "running"

    def _start_step(self, step: Step):
        sub, a = step.sub, step.args
        self.t_progress = time.monotonic()
        if sub == "REPLAN":
            self.op = WaitRunner(0.0)
            return
        if sub == "WAIT":
            self.op = WaitRunner(float(a.get("s", 1.0)))
            return
        if sub in NAV_SUBS:
            if not self.travel_ok:                       # 이동자세 가드
                self.steps.insert(self.si, Step("TRAVEL_POSE"))
                self._start_step(self.steps[self.si])
                return
            x, y, yaw = a["pose"]
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = self.p("map_frame")
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose = self._pose(x, y, yaw)
            self.nav.start(goal, self.p("timeout_nav_s"))
            self.op = self.nav
            return
        g = ArmSkill.Goal()
        g.skill = SKILL_OF[sub]
        g.target_frame = self.p("map_frame")
        g.puck_id = int(a.get("pid", -1))
        g.tray_slot = int(a.get("slot", -1))
        g.box_id = int(a.get("box", -1))
        g.location = a.get("loc", "")
        g.cell = [int(c) for c in a.get("cell", [])]
        g.expected_pips = int(a.get("pips", 0))
        g.timeout_s = float(self.p("timeout_arm_s"))
        if sub in ("PLACE", "FIX_PLACE") and a.get("cell"):
            wx, wy = cell_world_mm(self.locs[a["loc"]].center_mm, tuple(a["cell"]))
            g.target = self._pose(wx, wy, 0.0, self.p("place_surface_z_m"))
        if sub != "TRAVEL_POSE":
            self.travel_ok = False
        self.arm.start(g, self.p("timeout_arm_s"))
        self.op = self.arm

    def _advance(self):
        self.si += 1
        self.t_progress = time.monotonic()
        self._save()

    def _on_step_ok(self, step: Step, res):
        sub, a = step.sub, step.args
        if sub == "REPLAN":
            self._enter(Phase.PLAN, "replan")
            return
        if sub == "TRAVEL_POSE":
            self.travel_ok = True
        elif sub in NAV_SUBS:
            self.robot_mm = list(a["pose"][:2])
        elif sub == "PICK":
            pid = a["pid"]
            det = res.detected_puck_id if res is not None else -1
            if det >= 0 and det != pid and det in self.wm.pucks:
                self._log("pick_id_mismatch", planned=pid, detected=det)
                pid = det
                self.replan_pending = True
                for s in self.steps[self.si + 1:]:
                    if s.sub == "LOAD" and s.args.get("pid") == a["pid"]:
                        s.args["pid"] = det
                        break
            self.wm.mark_picked(pid)
            p = self.wm.pucks[pid]
            for attr in ("color", "shape"):
                v = getattr(res, f"detected_{attr}", "") if res is not None else ""
                if v and v != getattr(p, attr):
                    self._log("pick_attr_mismatch", pid=pid, attr=attr, before=getattr(p, attr), after=v)
                    setattr(p, attr, v)
                    self.replan_pending = True
        elif sub == "LOAD":
            self.wm.mark_loaded(a["pid"], a["slot"])
            if self.replan_pending:
                self._advance()
                self._enter(Phase.PLAN, "attr_mismatch")
                return
        elif sub == "UNLOAD":
            self.wm.mark_picked(a["pid"])
        elif sub == "PLACE":
            self.wm.mark_placed(a["pid"], a["loc"], tuple(a["cell"]))
            if len(self.wm.placed().get(a["loc"], [])) == next(g.count for g in self.groups if g.location == a["loc"]):
                self._beep(BEEP_MILESTONE)
        elif sub == "CHECK_PLACE":
            err_mm = getattr(res, "place_error_mm", 0.0) if res is not None else 0.0
            if err_mm > self.p("place_tol_mm") and not a.get("fixed"):
                self._log("place_error", pid=a["pid"], error_mm=err_mm)
                self.steps[self.si + 1:self.si + 1] = [Step("FIX_PLACE", dict(a)), Step("CHECK_PLACE", {**a, "fixed": True})]
        elif sub == "RECOVER":
            self.wm.mark_picked(a["pid"])
        elif sub == "VERIFY_LOC":
            self.verify_results[a["loc"]] = {"ok": True, "detail": getattr(res, "result_json", "")}
        self._advance()

    def _handle_failure(self, step: Step, res, err: str):
        sub, a = step.sub, step.args
        self.last_error = f"{sub}:{err}"
        self._log("step_fail", step=sub, args=a, error=err, retry=self.retry.get(self.si, 0))
        self._beep(BEEP_RECOVERABLE)
        if err == "HW_ERROR":
            self._enter(Phase.SAFE_STOP, f"hw_error_in_{sub}")
            return
        n = self.retry.get(self.si, 0)
        limit = self.p("retries_return") if (self.phase == Phase.RETURN) else \
            (self.p("retries_nav") if sub in NAV_SUBS else self.p("retries_arm"))
        if err not in ("DROPPED", "MISMATCH") and n < limit:
            self.retry[self.si] = n + 1
            return                                   # 같은 단계 재시도
        pid = a.get("pid")
        # ---- 상태별 실패 정책 (SAFE_STOP 아님: 끝까지 가서 재검증·복귀가 점수상 유리)
        if self.phase == Phase.INIT:
            self._enter(Phase.SAFE_STOP, f"init_failed:{err}")
        elif self.phase == Phase.SURVEY:
            self._log("survey_skip", step=sub, box=a.get("box"))
            self._skip_until(("TRAVEL_POSE",))
        elif self.phase == Phase.VERIFY:
            if sub == "VERIFY_LOC" and a.get("fix_left", 0) > 0:
                a["fix_left"] -= 1
                self.retry.pop(self.si, None)
                self.steps.insert(self.si, Step("FIX_LOC", {"loc": a["loc"], "pips": a["pips"]}))
                return
            if sub in ("VERIFY_LOC", "FIX_LOC"):
                self.verify_results[a["loc"]] = {"ok": False, "error": f"{sub}:{err}"}
            self._skip_until(("TRAVEL_POSE",))
        elif self.phase == Phase.RETURN:
            self._enter(Phase.SAFE_STOP, f"return_failed:{err}")
        elif self.phase == Phase.EXECUTE:
            if pid is not None and (err == "DROPPED" or sub in ("LOAD", "UNLOAD", "PLACE", "RECOVER")):
                if sub == "RECOVER":
                    self.wm.mark_lost(pid)
                    self._enter(Phase.PLAN, "recover_failed")
                    return
                self.wm.mark_dropped(pid)
                slot = self.wm.free_slot()
                tail = [Step("RECOVER", {"pid": pid}), Step("LOAD", {"pid": pid, "slot": slot}), Step("REPLAN")] \
                    if slot is not None else [Step("REPLAN")]
                self.steps = self.steps[:self.si] + tail
                self.retry.pop(self.si, None)
                self._log("recover_start", pid=pid)
            elif sub == "PICK" and pid is not None:
                self.wm.mark_unusable(pid)
                self._enter(Phase.PLAN, "pick_failed")
            elif sub == "CHECK_PLACE" or sub == "FIX_PLACE":
                self._advance()                       # 기록만 하고 진행
            elif sub in NAV_SUBS or sub.startswith("ALIGN"):
                box = a.get("box")
                if box is not None and sub in ("MOVE_TO_PICK", "ALIGN_PICK"):
                    for p in self.wm.pucks.values():
                        if p.box == box and p.status == PuckStatus.IN_BOX:
                            self.wm.mark_unusable(p.pid)
                self._enter(Phase.PLAN, f"{sub}_failed")
            else:
                self._enter(Phase.PLAN, f"{sub}_failed")
        else:
            self._enter(Phase.SAFE_STOP, f"unhandled:{sub}:{err}")

    def _skip_until(self, subs):
        self.si += 1
        while self.si < len(self.steps) and self.steps[self.si].sub not in subs:
            self.si += 1
        self._save()

    def _send_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p("map_frame")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self._pose(*self.sites["start_pose"])
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.0025
        msg.pose.covariance[35] = 0.003
        self.pub_init.publish(msg)
        self.initpose_sent = True
        self._log("initialpose", pose=list(self.sites["start_pose"]))

    # ================================================================== 관제 표시
    def _publish_detail(self):
        sub = self._cur_step_name() if self.phase in (Phase.EXECUTE, Phase.SURVEY, Phase.VERIFY, Phase.RETURN, Phase.INIT) else ""
        d = {"phase": self.phase.value, "sub": sub, "group": EXEC_GROUP.get(sub, ""), "mode": self.mode,
             "run_id": self.run_id, "armed": self.armed, "step": [self.si, len(self.steps)],
             "signal": (self.signal or {}).get("sequence"), "barcode": (self.barcode or {}).get("code"),
             "groups": [g.to_dict() for g in self.groups], "tray": self.wm.tray, "gripper": self.wm.gripper,
             "pucks": {k: [v.color, v.shape, v.status.value] for k, v in self.wm.pucks.items()},
             "survey_round": self.survey_round, "plan_count": self.plan_count, "partial": self.partial,
             "verify": self.verify_results, "last_error": self.last_error, "fail_reason": self.fail_reason}
        self.pub_detail.publish(String(data=json.dumps(d, ensure_ascii=False)))


def main():
    rclpy.init()
    node = MissionFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
