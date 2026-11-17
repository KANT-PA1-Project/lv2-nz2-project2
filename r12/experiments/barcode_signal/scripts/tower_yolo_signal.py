"""YOLO(pt/onnx/engine)로 신호등 위치 검출 + 칸별 색 판정 (자세·URDF 불필요)

흐름 (매 프레임)
  1) YOLO 로 tower_light 상자 검출 (가장 높은 점수 1개)
  2) 상자 위쪽 흰 덮개(CAP_FRAC)를 빼고 3등분 → 칸 R(위)·Y·G(아래), 각 칸 가운데 INNER 만 사용
  3) 기준 없는 판정: tower_signal.color_from_boxes(칸)        ← 켜진 상태로 시작해도 바로 색이 나옴
  4) 자동 고정: 기준 없는 판정이 OFF 이고 상자가 흔들리지 않는 상태가 AUTO_FRAMES 연속이면
     locate_body(꺼짐 중앙값, 칸) 로 칸을 렌즈에 맞추고 OFF 기준(make_off_reference)을 만든다.
     locate_body 가 실패하면 YOLO 칸 그대로 기준을 만든다 (화면·기록에 registration=yolo 로 표시)
  5) 고정 후: OFF 기준 비교 판정(color_from_boxes + off_reference). YOLO 상자가 고정 위치에서
     크게 벗어나면 고정 해제(moved) → 다시 4)
  최종 색 = 고정 후에는 기준 비교 판정, 고정 전에는 기준 없는 판정(unreferenced 표시)

판정 규칙·임계값은 tower_signal.PARAMS 그대로 (src/arm_practice/tower_signal.py, tower_signal_search.py 를
수정 없이 import). 후보 규칙, 실사 정답 영상 검증 전.

입력 (--source)
  realsense            D435 컬러 640×480 (pyrealsense2)
  <영상.mp4>            녹화 영상 (capture_d435.py --record 로 찍은 것)
  <사진 폴더>            사진마다 따로 판정 (고정 없음). 파일 이름에 _R_ / _Y_ / _G_ / _OFF_ 가 있으면 정답과 비교
  <숫자>                일반 웹캠 번호

실행 예 (개발 PC 또는 Jetson, ultralytics 설치된 환경)
  cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
  python scripts/tower_yolo_signal.py --model ~/git/p2_perception/models/current.pt --source realsense --exposure 150 --wb 4600
  python scripts/tower_yolo_signal.py --model ~/git/p2_perception/models/current.onnx --source realsense --exposure 150
  python scripts/tower_yolo_signal.py --model ~/git/p2_perception/models/current.pt --source ~/git/p2_perception/data/real/raw/1111_check/tower/real_xxx.mp4 --sequence
  python scripts/tower_yolo_signal.py --model ~/git/p2_perception/models/current.pt \
         --source "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal/validation_set_20261110/images" --no-show

바코드: 같은 영상에서 매 BARCODE_EVERY 프레임마다 화면 전체를 자동 해독 (scripts/barcode_support.py 의
  decode_rgb + StableCode, zxing-cpp Code 128). 등록 코드(P2-01~04)가 3프레임 연속 같으면 확정.
  확정 뒤 다른 코드가 확정되면 오류로 표시 (임의 선택 안 함). 바코드를 따로 지정하는 옵션은 없음

--sequence: tutor mission.SignalRecorder 로 12회 기록 (r12/src 필요, --r12-src).
  기록은 신호등 자동 고정(=준비 완료) 이후 프레임만 넣는다. 고정 해제(moved)되면 UNKNOWN 이 들어가 오류 정지
  12회 완료 + 바코드 확정이면 tutor mission.plan() 42줄과 같은 정렬로 L1~L3 그룹·퍽 개수 표시

키 (화면 켰을 때): r 고정 해제 / q·ESC 종료
"""
import argparse
import collections
import importlib.util
import json
import os
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]                        # r12/experiments/barcode_signal

# ---------------- 설정값 (한곳에서 변경) ----------------
WIDTH, HEIGHT, FPS = 640, 480, 30
TOWER_CLASS = 'tower_light'
CAP_FRAC = 0.09          # 상자 위쪽에서 뺄 흰 덮개 비율 = 덮개 11 mm / (덮개 11 + 렌즈 3단 108 mm) (실측값 기반)
INNER = 0.6              # 칸 가운데만 사용 (tower_signal.PARAMS['inner'] 와 같은 뜻)
AUTO_FRAMES = 30         # 자동 고정: 기준 없는 판정 OFF + 상자 안정이 이만큼 연속 (30 fps 에서 1초)
REF_MAX_FRAMES = 10      # OFF 기준 중앙값에 쓰는 최대 프레임 수
JITTER_PX = 4            # 고정 전 수집 중 허용 상자 흔들림 (px)
LOCATE_RADIUS_PX = 20    # locate_body 검색 반경. YOLO 상자가 이미 정확하므로 작게
MOVED_FRAC = 0.35        # 고정 후 YOLO 상자 중심이 (상자 폭 × 이 값) 이상 움직이면 고정 해제
FRAME_TIMEOUT_MS = 2000
BARCODE_EVERY = 3        # 바코드 해독 주기 (프레임). 확정 뒤에도 계속 확인
BARCODE_UPSCALE = 2      # 못 읽으면 2배 확대해 한 번 더 (barcode_support.decode_rgb)
COLOR_BGR = {'R': (0, 0, 255), 'Y': (0, 220, 255), 'G': (0, 200, 0),
             'OFF': (200, 200, 200), 'UNKNOWN': (255, 0, 255)}


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ts = load('tower_signal', HERE / 'src/arm_practice/tower_signal.py')
search = load('tower_signal_search', HERE / 'src/arm_practice/tower_signal_search.py')
barcode = load('barcode_support', HERE / 'scripts/barcode_support.py')      # zxing-cpp (vendor/)


def tiers_from_box(box, shape, cap_frac=CAP_FRAC, inner=INNER):
    """YOLO 상자 → (boxes, polygons). boxes = {색: (x0,y0,x1,y1,0.0)}, 화면 밖이면 (None, None)."""
    x1, y1, x2, y2 = map(float, box)
    y1 += (y2 - y1) * cap_frac
    w, h = x2 - x1, (y2 - y1) / 3
    boxes, polys = {}, {}
    for k, c in enumerate('RYG'):
        a0, a1 = x1 + w * (1 - inner) / 2, x2 - w * (1 - inner) / 2
        b0, b1 = y1 + k * h + h * (1 - inner) / 2, y1 + (k + 1) * h - h * (1 - inner) / 2
        bx = (int(np.floor(a0)), int(np.floor(b0)), int(np.ceil(a1)), int(np.ceil(b1)))
        if bx[0] < 0 or bx[1] < 0 or bx[2] > shape[1] or bx[3] > shape[0] or bx[2] - bx[0] < 2 or bx[3] - bx[1] < 2:
            return None, None
        boxes[c] = (*bx, 0.0)
        polys[c] = [[a0, b0], [a1, b0], [a1, b1], [a0, b1]]
    return boxes, polys


class TowerDetector:
    def __init__(self, model_path, imgsz=640, conf=0.25, device=None):
        from ultralytics import YOLO
        self.model = YOLO(str(Path(model_path).expanduser()), task='detect')
        names = self.model.names
        ids = [i for i, n in names.items() if n == TOWER_CLASS]
        if not ids:
            raise SystemExit(f'모델에 {TOWER_CLASS} 클래스 없음: {names}')
        self.cls, self.imgsz, self.conf, self.device = ids[0], imgsz, conf, device

    def __call__(self, bgr):
        """반환 (상자 xyxy, 점수) 또는 (None, 0)."""
        kw = dict(imgsz=self.imgsz, conf=self.conf, classes=[self.cls], verbose=False)
        if self.device is not None:
            kw['device'] = self.device
        r = self.model.predict(bgr, **kw)[0]
        if not len(r.boxes):
            return None, 0.0
        i = int(r.boxes.conf.argmax())
        return [float(v) for v in r.boxes.xyxy[i].tolist()], float(r.boxes.conf[i])


class SignalReader:
    """YOLO 상자 → 칸 → 색. 자동 고정 상태 관리. 카메라와 무관 (영상·사진으로 시험 가능)."""

    def __init__(self, auto_frames=AUTO_FRAMES, exposure_token=''):
        self.auto_frames, self.exposure_token = auto_frames, exposure_token
        self.reset()

    def reset(self):
        self.lock = None                 # dict(boxes, polygons, ref, key, box, how)
        self.frames, self.lens = [], []

    def _collect_reset(self):
        self.frames, self.lens = [], []

    def step(self, rgb, box):
        """한 프레임. 반환 dict(color, referenced, raw, res, state)."""
        shape = rgb.shape
        raw = None
        if box is not None:
            tb, tp = tiers_from_box(box, shape)
            raw = ts.color_from_boxes(rgb, tb, roi_polygons=tp) if tb else None
        if self.lock is not None:
            if box is not None:
                cx0 = (self.lock['box'][0] + self.lock['box'][2]) / 2; cy0 = (self.lock['box'][1] + self.lock['box'][3]) / 2
                cx1, cy1 = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                if np.hypot(cx1 - cx0, cy1 - cy0) > MOVED_FRAC * (self.lock['box'][2] - self.lock['box'][0]):
                    self.reset()
                    return dict(color='UNKNOWN', referenced=False, raw=raw, res=None, state='moved_unlocked')
            res = ts.color_from_boxes(rgb, self.lock['boxes'], roi_polygons=self.lock['polygons'],
                                      off_reference=self.lock['ref'], context_key=self.lock['key'])
            return dict(color=res['color'], referenced=True, raw=raw, res=res, state='locked:' + self.lock['how'])
        # 고정 전: 기준 없는 판정을 내보내고, OFF 가 이어지면 고정
        if box is None or raw is None:
            self._collect_reset()
            return dict(color='UNKNOWN', referenced=False, raw=raw, res=None,
                        state='no_tower' if box is None else 'box_outside_image')
        if raw['color'] != 'OFF':
            self._collect_reset()
        else:
            if self.lens and np.abs(np.subtract(box, self.lens[0])).max() > JITTER_PX:
                self._collect_reset()
            self.frames.append(np.ascontiguousarray(rgb)); self.lens.append(box)
            if len(self.frames) >= self.auto_frames:
                self._lock(shape)
        state = f'collect_off {len(self.frames)}/{self.auto_frames}'
        return dict(color=raw['color'], referenced=False, raw=raw, res=raw, state=state)

    def _lock(self, shape):
        box = np.median(np.array(self.lens), axis=0).tolist()
        tb, tp = tiers_from_box(box, shape)
        idx = np.linspace(0, len(self.frames) - 1, min(REF_MAX_FRAMES, len(self.frames))).round().astype(int)
        frames = [self.frames[i] for i in idx]
        how, boxes, polys = 'yolo', tb, tp
        reg = search.locate_body(np.median(frames, axis=0).astype(np.uint8), tb, tp, confirmed_off=True,
                                 params={'radius_px': LOCATE_RADIUS_PX})
        if reg['found']:
            how, boxes, polys = 'locate_body', reg['boxes'], reg['polygons']
        key = f"yolo:{self.exposure_token}:{boxes}"
        ref = ts.make_off_reference(frames, boxes, roi_polygons=polys, confirmed_off=True, context_key=key)
        self.lock = dict(boxes=boxes, polygons=polys, ref=ref, key=key, box=box, how=how)
        print(f"[tower] 자동 고정 ({how}; locate_body {reg.get('reason') or 'ok'} shift {reg.get('shift_px')}): {boxes}")
        self._collect_reset()


def draw(view, box, out, lock):
    if box is not None:
        cv2.rectangle(view, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 255), 1)
    polys = lock['polygons'] if lock else (tiers_from_box(box, view.shape)[1] if box is not None else None)
    for c, p in (polys or {}).items():
        cv2.polylines(view, [np.rint(np.asarray(p)).astype(np.int32)], True, COLOR_BGR[c], 2 if lock else 1)
    col = out['color']
    txt = col + ('' if out['referenced'] else '?')
    cv2.putText(view, txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 0, 0), 8)
    cv2.putText(view, txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.8, COLOR_BGR[col], 3)


def put_lines(view, lines):
    h = view.shape[0]
    for i, t in enumerate(lines):
        y = h - 12 - 22 * (len(lines) - 1 - i)
        cv2.putText(view, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 4)
        cv2.putText(view, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)


def frames_from(source, a):
    """(bgr, 시각 s, 이름) 생성기. 사진 폴더면 이름에 파일명."""
    if source == 'realsense':
        import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        prof = pipe.start(cfg)
        sensor = next(s for s in prof.get_device().query_sensors() if s.get_info(rs.camera_info.name) == 'RGB Camera')
        for opt, val, auto in ((rs.option.exposure, a.exposure, rs.option.enable_auto_exposure),
                               (rs.option.white_balance, a.wb, rs.option.enable_auto_white_balance)):
            if val is not None and sensor.supports(opt):
                sensor.set_option(auto, 0); sensor.set_option(opt, val)
        if a.exposure is None:
            print('[tower] 경고: 자동 노출. OFF 기준 비교는 노출 고정에서만 의미 있음 (--exposure 권장)')
        try:
            for _ in range(30):
                pipe.wait_for_frames(FRAME_TIMEOUT_MS)
            while True:
                try:
                    fr = pipe.wait_for_frames(FRAME_TIMEOUT_MS)
                except RuntimeError:
                    print('[tower] 카메라 프레임 타임아웃 → 종료'); return
                cf = fr.get_color_frame()
                if cf:
                    yield np.asanyarray(cf.get_data()).copy(), cf.get_timestamp() / 1000., None
        finally:
            pipe.stop()
    p = Path(os.path.expanduser(source))
    if p.is_dir():
        for f in sorted(x for x in p.iterdir() if x.suffix.lower() in ('.jpg', '.jpeg', '.png')):
            im = cv2.imread(str(f))
            if im is not None:
                yield im, None, f.name
        return
    cap = cv2.VideoCapture(int(source) if source.isdigit() else str(p))
    if not cap.isOpened():
        raise SystemExit(f'입력을 열 수 없음: {source}')
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    n = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            return
        n += 1
        yield bgr, (n / fps if not source.isdigit() else time.monotonic()), None


def _repo_r12():
    """이 파일이 저장소 r12/ 아래에 있으면 그 r12, 아니면 None (상대 경로 우선)."""
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'r12' and (parent / 'src/arm_practice/mission.py').is_file():
            return parent
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='.pt / .onnx / .engine (tower_light 클래스 포함)')
    ap.add_argument('--source', default='realsense')
    ap.add_argument('--imgsz', type=int, default=640, help='onnx/engine 은 내보낸 크기와 같게')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--device', default=None, help='예: 0, cpu')
    ap.add_argument('--detect-every', type=int, default=1, help='N 프레임마다 YOLO (사이 프레임은 직전 상자)')
    ap.add_argument('--auto-frames', type=int, default=AUTO_FRAMES)
    ap.add_argument('--exposure', type=float, default=None)
    ap.add_argument('--wb', type=float, default=None)
    ap.add_argument('--no-show', action='store_true')
    ap.add_argument('--log', default=None, help='프레임별 결과 jsonl 경로')
    ap.add_argument('--sequence', action='store_true', help='tutor SignalRecorder 로 12회 기록')
    ap.add_argument('--r12-src', default=str(_repo_r12() / 'src') if _repo_r12() else os.path.expanduser('~/git/lv2-nz2-project2/r12/src'))
    ap.add_argument('--stop-when-done', action='store_true',
                    help='--sequence 기록이 완료되거나 오류가 나면 종료 (자동 점검용)')
    a = ap.parse_args()
    if a.auto_frames < 3:
        raise SystemExit('--auto-frames 는 3 이상')

    det = TowerDetector(a.model, a.imgsz, a.conf, a.device)
    is_dir = Path(os.path.expanduser(a.source)).is_dir()
    reader = SignalReader(a.auto_frames, f'exp={a.exposure},wb={a.wb}')
    mission = rec = rec_err = None
    if a.sequence:
        sys.path.insert(0, a.r12_src)
        from arm_practice import mission
        rec = mission.SignalRecorder()
    stable, code, code_err = barcode.StableCode(), None, None
    log_f = open(os.path.expanduser(a.log), 'w') if a.log else None
    stats, confusion, det_ms, n, box, conf = collections.Counter(), collections.Counter(), [], 0, None, 0.0
    try:
        for bgr, t_s, name in frames_from(a.source, a):
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            if is_dir or n % a.detect_every == 0:
                t0 = time.perf_counter(); box, conf = det(bgr); det_ms.append((time.perf_counter() - t0) * 1000)
            n += 1
            if is_dir:                                        # 사진: 매장 독립, 기준 없는 판정만
                reader.reset()
                tb, tp = tiers_from_box(box, rgb.shape) if box is not None else (None, None)
                res = ts.color_from_boxes(rgb, tb, roi_polygons=tp) if tb else None
                out = dict(color=res['color'] if res else 'UNKNOWN', referenced=False, raw=res, res=res,
                           state='photo' if box is not None else 'no_tower')
                m = re.search(r'_(R|Y|G|OFF)_', name)
                if m:
                    confusion[f"{m.group(1)}->{out['color']}"] += 1
                print(f"{name}: {out['color']} ({out['state']}, conf {conf:.2f})"
                      + (f"  정답 {m.group(1)}" if m else ''))
            else:
                out = reader.step(rgb, box)
            stats[out['color']] += 1
            if n % BARCODE_EVERY == 0 or is_dir:                  # 바코드 자동 해독
                reads = barcode.decode_rgb(rgb, upscale=BARCODE_UPSCALE)['reads']
                if is_dir:
                    print(f"  바코드: {[r['text'] for r in reads] or '없음'}")
                else:
                    got = stable.observe(reads)
                    if got and code is None:
                        code = got; print(f'[tower] 바코드 확정 {code} (frame {n})')
                    elif got and got != code and code_err is None:
                        code_err = f'바코드 불일치 {code} vs {got}'; print(f'[tower] 오류: {code_err}')
            seq_txt = ''
            if rec is not None and rec_err is None and t_s is not None and out['referenced']:
                try:
                    rec.observe(t_s, out['color'])
                except ValueError as e:
                    rec_err = str(e); print(f'[tower] SignalRecorder 오류 (frame {n}): {rec_err}')
            if rec is not None:
                seq = ''.join(rec.sequence)
                seq_txt = ('' if reader.lock or rec.sequence else 'not ready  ') + f'seq {len(seq)}/12 {seq}' + (' COMPLETE' if rec.complete else '') + (f' ERROR {rec_err}' if rec_err else '')
            if log_f:
                r_ = out.get('res') or {}
                log_f.write(json.dumps(dict(frame=n, t_s=t_s, name=name, box=box, conf=conf, color=out['color'],
                                            referenced=out['referenced'], state=out['state'],
                                            decision=r_.get('decision'), reason=r_.get('reason'),
                                            ratios=r_.get('ratios'), seq=''.join(rec.sequence) if rec else None)) + '\n')
            if not a.no_show:
                view = bgr.copy()
                draw(view, box, out, reader.lock)
                put_lines(view, [f"{out['state']}  conf {conf:.2f}  {np.mean(det_ms[-30:]):.0f} ms",
                                 f"barcode {code or '-'}" + (f'  ERROR {code_err}' if code_err else ''), seq_txt])
                cv2.imshow('tower yolo signal', view)
                k = cv2.waitKey(0 if is_dir else 1) & 0xFF
                if k in (ord('q'), 27):
                    break
                if k == ord('r'):
                    reader.reset(); print('[tower] 고정 해제')
            if a.stop_when_done and rec is not None and (rec.complete or rec_err):
                break
    except KeyboardInterrupt:
        print('[tower] 중단 (Ctrl+C)')
    finally:
        if log_f:
            log_f.close()
        if not a.no_show:
            cv2.destroyAllWindows()
    print(f'[tower] 프레임 {n}, 판정 {dict(stats)}, YOLO 평균 {np.mean(det_ms) if det_ms else 0:.1f} ms')
    if confusion:
        ok = sum(v for k, v in confusion.items() if k.split('->')[0] == k.split('->')[1])
        print(f'[tower] 정답 비교 {ok}/{sum(confusion.values())}: {dict(sorted(confusion.items()))}')
    if rec is not None:
        seq = ''.join(rec.sequence)
        print(f"[tower] 기록 {seq} ({len(seq)}개) {'완료' if rec.complete else '미완료'} {rec_err or ''}")
        if rec.complete and code and not code_err:
            c = {k: seq.count(k) for k in 'RYG'}; d = mission.BARCODES[code]
            print(f'[tower] 배치 ({code}, {"내림" if d else "오름"}차순)',
                  [(f'L{i+1}', g, c[g]) for i, g in enumerate(sorted('RYG', key=lambda k: -c[k] if d else c[k]))])
        elif rec.complete:
            print(f'[tower] 배치 계산 불가: 바코드 {code_err or "미확정"}')
    if not is_dir:
        print(f'[tower] 바코드 {code or "미확정"} {code_err or ""}')
    if rec is not None:                                  # 자동 점검용 한 줄 결과
        seq = ''.join(rec.sequence)
        c = {k: seq.count(k) for k in 'RYG'}
        plan = None
        if rec.complete and code and not code_err:
            d = mission.BARCODES[code]
            plan = [[f'L{i+1}', g, c[g]] for i, g in enumerate(sorted('RYG', key=lambda k: -c[k] if d else c[k]))]
        print('RESULT ' + json.dumps(dict(sequence=seq, complete=rec.complete, error=rec_err, counts=c,
                                          barcode=code, barcode_error=code_err, plan=plan)), flush=True)


if __name__ == '__main__':
    main()
