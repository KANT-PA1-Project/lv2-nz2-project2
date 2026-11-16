#!/usr/bin/env python3
"""
signal_recorder  (NZ2 인지) — 미션 v5 "신호 순서 기억" (신호 순서 12점 + 색상별 횟수 9점)

역할: /perception/traffic_light 의 프레임별 판정(raw_state, visible)을 받아
      12회 점멸을 하나씩 분리하고, 색 순서와 R·Y·G 횟수를 기록 → signal.json 저장 + 결과 토픽 발행

미션 규칙 (mission.pdf v5, 02 신호와 바코드 규칙)
  - 1회 점멸 = 한 색 1 s 켜짐 + 1 s 소등. 연속 같은 색도 소등으로 구별
  - 12번째 점멸 후 3 s 연속 소등 = 종료 (별도 출발 신호 없음)
  - R/Y/G 각 1~6회, 합계 12. 소등은 횟수에 포함하지 않음
  - 누락·불명 색은 임의 추정 금지 → 12회 미만/초과, 불명 색이면 오류 기록 (미션 노드가 안전 정지)

판정 방법
  - 켜짐 구간: raw_state 가 R/Y/G 인 연속 프레임. 구간 안 다수결 색이 그 점멸의 색
      (다수결 비율 < min_color_ratio 면 "불명" → 오류. 추정하지 않음)
  - 켜짐 사이 OFF 가 min_off_s 미만이면 같은 점멸로 봄 (LED 깜빡임). 이상이면 점멸 1회 확정
  - 켜짐이 min_on_s 미만이면 잡음으로 버림 (경고 기록), max_on_s 초과면 "두 점멸이 붙었을 수 있음" 오류
  - visible=false(경광등 안 보임)는 OFF 로 세지 않음. 기록 중 안 보인 구간이
      warn_invisible_s 넘으면 경고, max_invisible_s(0.9 s) 넘으면 "관측 누락" 오류
      (켜짐 1 s 짜리 점멸 하나가 통째로 가려질 수 있는 길이 → 횟수를 믿을 수 없음)
  - 첫 켜짐부터 기록 시작, OFF 가 end_off_s 이상 이어지면 종료 (미션은 3 s, 여유를 두고 2 s)

입력: /perception/traffic_light (nz2_interfaces/TrafficLight, visible·raw_state 필드 필요 — 2026-10-28 추가)
출력: /perception/signal_result (std_msgs/String, JSON, transient_local — 늦게 붙은 노드도 받음)
      <log_dir>/signal_<시각>.json 과 <log_dir>/signal.json (최신)
서비스: /signal_recorder/reset (std_srvs/Trigger) — 다시 대기 상태로 (운영자가 시퀀스를 다시 보여줄 때)

실행 (Jetson 또는 노트북, ros_ws 를 source 한 터미널):
    ros2 run perception signal_recorder.py --ros-args -p log_dir:=$HOME/nz2_logs
"""
import json
import os
import time
from collections import Counter

STATE_UNKNOWN, STATE_RED, STATE_YELLOW, STATE_GREEN, STATE_OFF = 0, 1, 2, 3, 4
COLOR_NAME = {STATE_RED: 'R', STATE_YELLOW: 'Y', STATE_GREEN: 'G'}


class SignalSequencer:
    """ROS 와 무관한 순수 로직 (단위 시험 가능). feed() 로 프레임을 넣고 tick() 으로 시간 경과를 알린다."""

    def __init__(self, expected=12, min_on_s=0.3, max_on_s=1.8, min_off_s=0.4, end_off_s=2.0,
                 max_invisible_s=0.9, warn_invisible_s=0.3, min_color_ratio=0.7, min_count=1, max_count=6):
        self.p = dict(expected=expected, min_on_s=min_on_s, max_on_s=max_on_s, min_off_s=min_off_s,
                      end_off_s=end_off_s, max_invisible_s=max_invisible_s, warn_invisible_s=warn_invisible_s,
                      min_color_ratio=min_color_ratio,
                      min_count=min_count, max_count=max_count)
        self.reset()

    def reset(self):
        self.state = 'WAIT'        # WAIT(첫 켜짐 대기) → RUNNING → DONE
        self.blinks = []           # 확정된 점멸 [{color, t_on, t_off, on_s, color_ratio}]
        self.errors, self.warnings = [], []
        self.cur = None            # 지금 켜져 있는 구간 {t_on, t_last_on, colors[]}
        self.pending = None        # 꺼졌지만 아직 min_off_s 가 안 지난 구간 (다시 켜지면 이어붙임)
        self.off_since = None      # 마지막 OFF 시작 시각 (보이는 OFF 만)
        self.invis_since = None
        self.invis_flag = None
        self.t_start = self.t_end = None
        self.result = None

    # ── 입력 ──────────────────────────────────────────────────────────
    def feed(self, t, visible, raw):
        """t: 영상 시각(s), visible: 경광등 보임, raw: STATE_* (프레임별 판정)"""
        if self.state == 'DONE':
            return
        if not visible or raw == STATE_UNKNOWN:
            if self.state == 'RUNNING':
                if self.invis_since is None:
                    self.invis_since, self.invis_flag = t, None
                gap = t - self.invis_since
                if gap > self.p['max_invisible_s'] and self.invis_flag != 'error':
                    self._error(f'관측 누락: {gap:.2f}s 이상 경광등 안 보임 (t={self.invis_since:.2f}) — 점멸이 가려졌을 수 있음')
                    self.invis_flag = 'error'
                elif gap > self.p['warn_invisible_s'] and self.invis_flag is None:
                    self.warnings.append(f'잠깐 안 보임 {gap:.2f}s+ (t={self.invis_since:.2f})')
                    self.invis_flag = 'warn'
            return
        self.invis_since = None

        if raw in COLOR_NAME:
            if self.state == 'WAIT':
                self.state, self.t_start = 'RUNNING', t
            if self.cur is None:
                if self.pending is not None:      # 짧은 OFF 뒤 다시 켜짐 → 같은 점멸 (LED 깜빡임)
                    self.cur, self.pending = self.pending, None
                else:
                    self.cur = {'t_on': t, 't_last_on': t, 'colors': []}
            self.cur['colors'].append(COLOR_NAME[raw])
            self.cur['t_last_on'] = t
            self.off_since = None
        elif raw == STATE_OFF:
            if self.off_since is None:
                self.off_since = t
            if self.cur is not None:              # 켜짐 → 꺼짐: 일단 보류
                self.cur['t_off'] = t
                self.pending, self.cur = self.cur, None
        self.tick(t)

    def tick(self, t):
        """시간 경과 처리: 보류 구간 확정, 종료 판정 (영상이 안 와도 노드 타이머가 부름)"""
        if self.state != 'RUNNING':
            return
        if self.pending is not None and self.off_since is not None and t - self.off_since >= self.p['min_off_s']:
            self._commit(self.pending)
            self.pending = None
        if (self.cur is None and self.pending is None and self.off_since is not None
                and t - self.off_since >= self.p['end_off_s'] and self.blinks):
            self.t_end = t
            self._finish()

    def stale(self, t):
        """메시지가 끊겼을 때 (노드 타이머). 기록 중이면 관측 누락"""
        if self.state == 'RUNNING':
            self.feed(t, False, STATE_UNKNOWN)

    # ── 내부 ──────────────────────────────────────────────────────────
    def _commit(self, seg):
        on_s = seg['t_off'] - seg['t_on']
        cnt = Counter(seg['colors'])
        color, n = cnt.most_common(1)[0]
        ratio = n / len(seg['colors'])
        if on_s < self.p['min_on_s']:
            self.warnings.append(f"짧은 켜짐 무시: {color} {on_s:.2f}s (t={seg['t_on']:.2f})")
            return
        b = {'index': len(self.blinks) + 1, 'color': color, 't_on': round(seg['t_on'], 3),
             't_off': round(seg['t_off'], 3), 'on_s': round(on_s, 3), 'color_ratio': round(ratio, 3),
             'frames': len(seg['colors'])}
        if ratio < self.p['min_color_ratio']:
            b['color'] = '?'
            self._error(f"점멸 {b['index']}: 색 불명 ({dict(cnt)}) — 추정하지 않음")
        if on_s > self.p['max_on_s']:
            self._error(f"점멸 {b['index']}: 켜짐 {on_s:.2f}s > {self.p['max_on_s']}s — 같은 색 두 번이 붙었을 수 있음")
        self.blinks.append(b)

    def _error(self, msg):
        self.errors.append(msg)

    def _finish(self):
        self.state = 'DONE'
        seq = [b['color'] for b in self.blinks]
        counts = {c: seq.count(c) for c in 'RYG'}
        if len(seq) != self.p['expected']:
            self._error(f"점멸 {len(seq)}회 (규정 {self.p['expected']}회)")
        for c in 'RYG':
            if not self.p['min_count'] <= counts[c] <= self.p['max_count']:
                self._error(f"{c} {counts[c]}회 — 허용 {self.p['min_count']}~{self.p['max_count']}회 밖")
        self.result = {
            'valid': not self.errors,
            'sequence': seq,
            'sequence_str': ' '.join(seq),
            'counts': counts,
            'n_blinks': len(seq),
            'blinks': self.blinks,
            'errors': self.errors,
            'warnings': self.warnings,
            't_start': self.t_start,
            't_end': self.t_end,
            'params': self.p,
        }


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from nz2_interfaces.msg import TrafficLight
    try:
        from std_srvs.srv import Trigger
    except ImportError:          # std_srvs 가 없으면 reset 서비스만 빠짐 (Jetson ROS 설치엔 기본 포함)
        Trigger = None

    class SignalRecorderNode(Node):
        def __init__(self):
            super().__init__('signal_recorder')
            g = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
            self.log_dir = os.path.expanduser(g('log_dir', '~/nz2_logs'))
            self.stale_s = float(g('stale_s', 0.3))
            self.seq = SignalSequencer(
                expected=int(g('expected_blinks', 12)), min_on_s=float(g('min_on_s', 0.3)),
                max_on_s=float(g('max_on_s', 1.8)), min_off_s=float(g('min_off_s', 0.4)),
                end_off_s=float(g('end_off_s', 2.0)), max_invisible_s=float(g('max_invisible_s', 0.9)),
                warn_invisible_s=float(g('warn_invisible_s', 0.3)),
                min_color_ratio=float(g('min_color_ratio', 0.7)))
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/perception/signal_result', latched)
            self.create_subscription(TrafficLight, '/perception/traffic_light', self.on_light, 10)
            if Trigger is not None:
                self.create_service(Trigger, '~/reset', self.on_reset)
            else:
                self.get_logger().warning('std_srvs 없음 → ~/reset 서비스 없이 실행 (재시작으로 초기화)')
            self.create_timer(0.1, self.on_timer)
            self.last_rx = None          # (메시지 시각, 수신 벽시계)
            self.n_logged = 0
            self.get_logger().info(f'signal_recorder 대기: 첫 켜짐부터 기록 → {self.log_dir}/signal.json')

        def on_light(self, m):
            if not hasattr(m, 'raw_state'):
                self.get_logger().error('TrafficLight 에 raw_state/visible 없음 → nz2_interfaces 다시 빌드')
                return
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.last_rx = (t, time.monotonic())
            before = self.seq.state
            self.seq.feed(t, bool(m.visible), int(m.raw_state))
            self._report(before)

        def on_timer(self):
            if self.last_rx is None:
                return
            t_msg, t_wall = self.last_rx
            gap = time.monotonic() - t_wall
            t_now = t_msg + gap          # 메시지 시각 기준으로 흐른 시간 추정
            if gap > self.stale_s:
                self.seq.stale(t_now)
            before = self.seq.state
            self.seq.tick(t_now)
            self._report(before)

        def _report(self, before):
            s = self.seq
            if before == 'WAIT' and s.state == 'RUNNING':
                self.get_logger().info('신호 시퀀스 시작 (첫 켜짐)')
            while self.n_logged < len(s.blinks):
                b = s.blinks[self.n_logged]
                self.get_logger().info(f"점멸 {b['index']:2d}: {b['color']}  켜짐 {b['on_s']:.2f}s  "
                                       f"색 일치 {b['color_ratio']:.0%}")
                self.n_logged += 1
            if before != 'DONE' and s.state == 'DONE':
                self._save(s.result)

        def _save(self, r):
            r = dict(r, saved_at=time.strftime('%Y-%m-%d %H:%M:%S'))
            os.makedirs(self.log_dir, exist_ok=True)
            txt = json.dumps(r, ensure_ascii=False, indent=2)
            path = os.path.join(self.log_dir, time.strftime('signal_%Y%m%d_%H%M%S.json'))
            for p in (path, os.path.join(self.log_dir, 'signal.json')):
                with open(p, 'w', encoding='utf-8') as f:
                    f.write(txt)
            self.pub.publish(String(data=json.dumps(r, ensure_ascii=False)))
            log = self.get_logger().info if r['valid'] else self.get_logger().error
            log(f"신호 기록 종료: {r['sequence_str']}  R{r['counts']['R']} Y{r['counts']['Y']} "
                f"G{r['counts']['G']}  {'유효' if r['valid'] else '오류: ' + ' / '.join(r['errors'])}  → {path}")

        def on_reset(self, _req, res):
            self.seq.reset()
            self.n_logged = 0
            res.success, res.message = True, '신호 기록 초기화 → 첫 켜짐 대기'
            self.get_logger().info(res.message)
            return res

    rclpy.init()
    node = SignalRecorderNode()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / launch 종료
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
