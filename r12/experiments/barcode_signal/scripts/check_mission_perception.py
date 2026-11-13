"""미션 인식·판단 전체 점검: 신호등(OpenCR sign_tower_v3) 정답 vs 로봇 인식(tower_yolo_signal.py) 결과

한 회차 흐름 (경기 절차와 같은 순서)
  1) 신호등 OpenCR 에 STOP → RAND(또는 --seq) 로 정답 시퀀스를 정함 (아직 소등)
  2) tower_yolo_signal.py 실행: D435 로 신호등 찾기·꺼짐 고정 + 바코드 자동 해독
  3) 로봇 쪽 "자동 고정"(=준비 완료)이 뜨면 신호등에 START  ← 운영자가 시작하는 역할
  4) 12회 + 3 s 소등 후 로봇 기록 완료 → 결과(RESULT) 수집
  5) 채점표(mission.pdf 06)의 인식·판단 항목으로 비교
       신호 순서 12점(시점별 1점) / 색상별 횟수 9점(각 3점) / 바코드 4점 / 장소 그룹 배정 9점(각 3점) = 34점
     정답 배정은 tutor mission.py 의 BARCODES·정렬 규칙(동률 R→Y→G)으로 계산

준비
  - 신호등 OpenCR(sign_tower_v3) USB 연결, 신호등 전원 연결
  - D435 연결, 신호등·바코드가 화면에 보이게 고정 (규정: 관측 위치 (734,950) mm 에서 -X)
  - 실제 놓은 바코드 라벨을 --barcode 로 알려줌 (정답 비교용. 로봇 인식에는 쓰지 않음)

실행 (개발 PC)
  cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
  python scripts/check_mission_perception.py --barcode P2-01 --model ~/git/p2_perception/models/current.pt \
         --exposure 150 --wb 4600 --rounds 3
  # 특정 패턴: --seq RGRRGRGGYGYG   /  화면 끄기: --no-show   /  신호등 포트: --tower-port /dev/ttyACM0
결과: reports/mission_check/<시각>.json + 화면 요약
"""
import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
DETECTOR = HERE / 'scripts/tower_yolo_signal.py'

READY_TIMEOUT_S = 90      # 로봇 준비(꺼짐 고정)까지 최대 대기
RUN_TIMEOUT_S = 60        # START 후 기록 완료까지 최대 대기 (정상 26 s)
BOOT_WAIT_S = 3.0         # 시리얼을 열면 OpenCR 이 재시작될 수 있음 → 부팅 대기


def expected_plan(barcodes, seq, code):
    c = {k: seq.count(k) for k in 'RYG'}
    desc = barcodes[code]
    return [[f'L{i+1}', g, c[g]] for i, g in enumerate(sorted('RYG', key=lambda k: -c[k] if desc else c[k]))]


def score(truth_seq, truth_code, result, barcodes):
    got = result.get('sequence') or ''
    s_order = sum(1 for i in range(12) if i < len(got) and got[i] == truth_seq[i])
    tc = {k: truth_seq.count(k) for k in 'RYG'}
    gc = result.get('counts') or {}
    s_count = sum(3 for k in 'RYG' if result.get('complete') and gc.get(k) == tc[k])
    s_code = 4 if result.get('barcode') == truth_code and not result.get('barcode_error') else 0
    exp = expected_plan(barcodes, truth_seq, truth_code)
    plan = result.get('plan') or []
    s_assign = sum(3 for i in range(3) if i < len(plan) and plan[i][1] == exp[i][1] and plan[i][2] == exp[i][2])
    return dict(order=s_order, counts=s_count, barcode=s_code, assignment=s_assign,
                total=s_order + s_count + s_code + s_assign, expected_plan=exp)


class Tower:
    """sign_tower_v3 시리얼. 읽은 줄은 큐로."""
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.q = queue.Queue()
        self.lines = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b''
        while True:
            try:
                buf += self.s.read(256)
            except Exception:
                return
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                t = line.decode(errors='ignore').strip()
                if t:
                    self.lines.append((time.time(), t)); self.q.put(t)

    def send(self, cmd):
        self.s.write((cmd + '\n').encode())

    def wait_for(self, prefix, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                t = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if t.startswith(prefix):
                return t
        return None


def run_round(a, tower, barcodes, k):
    print(f'\n===== 회차 {k} =====')
    while not tower.q.empty():
        tower.q.get()
    tower.send('STOP')
    time.sleep(0.3)
    tower.send(f'SEQ {a.seq}' if a.seq else 'RAND')
    seq_line = tower.wait_for('SEQ,', 5)
    if seq_line is None:
        return dict(ok=False, error='신호등 응답 없음 (포트·펌웨어 확인)')
    truth_seq = seq_line.split(',')[1]
    print(f'[check] 정답 시퀀스 {truth_seq} (신호등 대기 중, 소등)')

    cmd = a.detector_cmd.split() if a.detector_cmd else [
        sys.executable, '-u', str(DETECTOR), '--model', a.model, '--source', 'realsense',
        '--sequence', '--stop-when-done', '--r12-src', a.r12_src, '--detect-every', str(a.detect_every)]
    if not a.detector_cmd:
        if a.exposure is not None: cmd += ['--exposure', str(a.exposure)]
        if a.wb is not None: cmd += ['--wb', str(a.wb)]
        if a.no_show: cmd += ['--no-show']
    det = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    dq, dlines = queue.Queue(), []

    def reader():
        for line in det.stdout:
            line = line.rstrip('\n'); dlines.append(line); dq.put(line)
    threading.Thread(target=reader, daemon=True).start()

    # 로봇 준비(자동 고정) 대기 → START
    ready, end = False, time.time() + READY_TIMEOUT_S
    while time.time() < end and det.poll() is None:
        try:
            line = dq.get(timeout=0.2)
        except queue.Empty:
            continue
        if line.startswith('[tower]'):
            print('  ' + line)
        if '자동 고정' in line:
            ready = True
            break
    if not ready:
        det.send_signal(signal.SIGINT); det.wait(10)
        return dict(ok=False, truth=truth_seq, error='로봇이 준비 시간 안에 신호등을 고정하지 못함',
                    detector_tail=dlines[-15:])
    tower.send('START')
    t_start = time.time()
    print('[check] 로봇 준비 완료 → 신호 시작')

    # 기록 완료까지
    result = None
    while time.time() < t_start + RUN_TIMEOUT_S:
        try:
            line = dq.get(timeout=0.2)
        except queue.Empty:
            if det.poll() is not None and dq.empty():
                break
            continue
        if line.startswith('[tower]'):
            print('  ' + line)
        if line.startswith('RESULT '):
            result = json.loads(line[7:])
            break
    if result is None and det.poll() is None:
        det.send_signal(signal.SIGINT)                # 중단해도 RESULT 를 출력하게 되어 있음
        try:
            det.wait(15)
        except subprocess.TimeoutExpired:
            det.kill()
        while not dq.empty():
            line = dq.get()
            if line.startswith('RESULT '):
                result = json.loads(line[7:])
    det.wait(15) if det.poll() is None else None
    tower.send('STOP')
    if result is None:
        return dict(ok=False, truth=truth_seq, error='로봇 결과(RESULT) 없음', detector_tail=dlines[-15:])
    sc = score(truth_seq, a.barcode, result, barcodes)
    events = [t for _, t in tower.lines if t.startswith('EVT,') and _ >= t_start - 1]
    return dict(ok=True, truth=truth_seq, truth_barcode=a.barcode, result=result, score=sc,
                tower_events=events[-24:], detector_tail=dlines[-10:])


def _repo_r12():
    """이 파일이 저장소 r12/ 아래에 있으면 그 r12, 아니면 None (상대 경로 우선)."""
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'r12' and (parent / 'src/arm_practice/mission.py').is_file():
            return parent
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--barcode', required=True, help='실제 놓은 바코드 (P2-01~04), 정답 비교용')
    ap.add_argument('--model', default=os.path.expanduser('~/git/p2_perception/models/current.pt'))
    ap.add_argument('--tower-port', default='/dev/ttyACM0')
    ap.add_argument('--seq', default=None, help='정답 시퀀스 고정 (없으면 매 회차 RAND)')
    ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--exposure', type=float, default=None)
    ap.add_argument('--wb', type=float, default=None)
    ap.add_argument('--detect-every', type=int, default=5)
    ap.add_argument('--no-show', action='store_true')
    ap.add_argument('--r12-src', default=str(_repo_r12() / 'src') if _repo_r12() else os.path.expanduser('~/git/lv2-nz2-project2/r12/src'))
    ap.add_argument('--detector-cmd', default=None, help=argparse.SUPPRESS)   # 시험용 대체 명령
    a = ap.parse_args()
    sys.path.insert(0, a.r12_src)
    from arm_practice.mission import BARCODES
    if a.barcode not in BARCODES:
        raise SystemExit(f'미등록 바코드 {a.barcode} (등록: {list(BARCODES)})')

    tower = Tower(a.tower_port)
    time.sleep(BOOT_WAIT_S)
    rounds = [run_round(a, tower, BARCODES, k + 1) for k in range(a.rounds)]

    print('\n===== 요약 (인식·판단 34점 만점) =====')
    for k, r in enumerate(rounds, 1):
        if not r['ok']:
            print(f"회차 {k}: 실패 - {r['error']}  (정답 {r.get('truth')})")
            for t in r.get('detector_tail', [])[-5:]:
                print('    ' + t)
            continue
        s, res = r['score'], r['result']
        print(f"회차 {k}: {s['total']}/34  순서 {s['order']}/12  횟수 {s['counts']}/9  바코드 {s['barcode']}/4  배정 {s['assignment']}/9")
        print(f"   정답 {r['truth']} / 인식 {res['sequence']} {'완료' if res['complete'] else '미완료'} {res.get('error') or ''}")
        print(f"   바코드 정답 {r['truth_barcode']} / 인식 {res.get('barcode')} {res.get('barcode_error') or ''}")
        print(f"   배정 정답 {s['expected_plan']} / 인식 {res.get('plan')}")
    out = HERE / 'reports/mission_check'
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(dict(args=vars(a), rounds=rounds), ensure_ascii=False, indent=1))
    print(f'기록: {path}')


if __name__ == '__main__':
    main()
