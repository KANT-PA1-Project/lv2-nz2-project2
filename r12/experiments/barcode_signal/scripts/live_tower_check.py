"""D435 실물 신호등 검출 확인 — tutor 로 옮길 검출 로직을 그대로 호출 (시각화·기록만 추가)

호출하는 코드 (수정 없이 import 만 함):
  src/arm_practice/tower_signal.py         tier_rois (투영) · make_off_reference · camera_context
  src/arm_practice/tower_signal_search.py  locate_body (투영 주변 몸통 찾기) · read_registered (매 프레임 판정)
  tutor r12/src/arm_practice/mission.py    SignalRecorder (12회 기록) · BARCODES (정렬 방향)
호출 순서는 scripts/test_tower_signal_isaac.py 488~538줄(시뮬 시험)과 같다.
tutor 와 다른 점은 카메라 자세 입력 하나뿐:
  tutor = map_pose × 카메라 장착 변환 / 여기 = 실측한 카메라 위치·각도(--cam-*)

순서
  1) D435 를 실제 관측 위치에 둔다. 규정: 로봇 중심 (734, 950) mm 에서 -X 방향 관측, 신호등 (80, 950, 0) mm
     카메라 높이(--cam-z)·숙임 각도(--pitch)를 재서 넣는다. 화면의 회색 칸 = 이 자세로 투영한 예상 위치
  2) 신호등이 꺼진 상태(운영자 시작 전)에서 o → 꺼짐 3장 중앙값으로 locate_body 실행
     성공: 컬러 칸 표시 + 콘솔에 이동(px)·크기·점수. 실패: 이유 표시 (body_not_found, ambiguous_bodies …)
  3) 신호 시작 → 매 프레임 read_registered 판정, SignalRecorder 에 기록. 12회 + 3초 소등이면 완료 표시
     --barcode 를 주면 tutor mission.py 와 같은 정렬로 L1~L3 그룹·퍽 개수 표시

실행 (개발 PC, D435 연결):
  source ~/yolo_env/bin/activate
  cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
  python scripts/live_tower_check.py --cam-z 0.30 --pitch 10 --exposure 150 --wb 4600 --barcode P2-01
  # --cam-x/--cam-y 기본 = 규정 관측 위치(0.734, 0.950), --yaw 기본 180(-X 방향)
  # 자세를 대충 넣었으면 --radius 를 키움(최대 256 px). 이동량(shift_px)이 크게 나오면 실측값을 다시 확인

키: o 꺼짐 확인 → 몸통 찾기 / r 해제·레코더 초기화 / l 기록(jsonl) / v 원본 녹화(mp4) / q 종료

주의
  - 신호등 좌표는 --tower-x/-y/-z 로 넘긴다(기본 규정값 0.080/0.950/0.0). tower_signal.py 45줄 상수는
    (0.12, 0.95) [가정] 그대로이며 이 스크립트는 그 파일을 바꾸지 않는다
  - 카메라 내부값은 RealSense 가 알려주는 실제 값을 쓴다 (시뮬 400 아님)
  - 노출 고정 권장. 자동 노출이면 OFF 기준 비교가 무의미
  - 판정 규칙·임계값은 그대로 (후보 규칙, 실사 정답 영상 검증 전)
"""
import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]                         # r12/experiments/barcode_signal

# ---------------- 설정값 ----------------
WIDTH, HEIGHT, FPS = 640, 480, 30          # 판정 기준 해상도 (tutor uniform640)
OFF_FRAMES = 3                             # 꺼짐 기준 프레임 수 (시뮬 시험과 같음)
FRAME_TIMEOUT_MS = 2000                    # 카메라 프레임 타임아웃 → 종료
SPEC_OBSERVE_XY = (0.734, 0.950)           # mission.pdf v5: 로봇 관측 중심 (m)
SPEC_TOWER_XYZ = (0.080, 0.950, 0.0)       # mission.pdf v5: 신호등 하단 기준 (m)
COLOR_BGR = {'R': (0, 0, 255), 'Y': (0, 220, 255), 'G': (0, 200, 0),
             'OFF': (200, 200, 200), 'UNKNOWN': (255, 0, 255)}


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ts = load('tower_signal', HERE / 'src/arm_practice/tower_signal.py')
search = load('tower_signal_search', HERE / 'src/arm_practice/tower_signal_search.py')


def load_mission(r12_src):
    """tutor mission.py (SignalRecorder, BARCODES) — tutor 는 읽기만."""
    sys.path.insert(0, str(r12_src))
    from arm_practice import mission
    return mission


def camera_pose(x, y, z, yaw_deg, pitch_down_deg, roll_deg=0.0):
    """카메라 광학 프레임(x 오른쪽, y 아래, z 앞)의 월드 자세 4×4. yaw 180 = -X 방향을 봄."""
    yaw, pitch, roll = np.radians([yaw_deg, pitch_down_deg, roll_deg])
    zc = np.array([np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), -np.sin(pitch)])
    xc = np.cross(zc, [0, 0, 1.]); xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    c, s = np.cos(roll), np.sin(roll)
    xc, yc = c * xc + s * yc, -s * xc + c * yc
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = xc, yc, zc, [x, y, z]
    return T


class Detector:
    """tutor 로 옮길 검출 경로를 한 묶음으로 호출 (카메라 없이 사진으로도 시험 가능)."""

    def __init__(self, T, K, shape, tower, radius_px, exposure_token):
        self.T, self.K, self.shape, self.tower = T, np.asarray(K, float), shape, tower
        self.radius, self.exposure_token = radius_px, exposure_token
        self.prior_boxes, self.prior_polys, self.prior_reason = ts.tier_rois(T, self.K, shape, tower)
        self.reset()

    def reset(self):
        self.registration = self.ref = self.context = None

    @property
    def locked(self):
        return self.ref is not None

    def lock(self, off_frames):
        """확인된 꺼짐 프레임들로 몸통 찾기 + OFF 기준 생성. 반환: locate_body 결과 dict."""
        if self.prior_reason:
            return dict(found=False, reason='projection:' + self.prior_reason)
        off_frames = [np.ascontiguousarray(f) for f in off_frames]
        reg = search.locate_body(np.median(off_frames, axis=0).astype(np.uint8),
                                 self.prior_boxes, self.prior_polys, confirmed_off=True,
                                 params={'radius_px': self.radius})
        if reg['found']:
            ctx = ts.camera_context(self.T, self.K, self.shape, self.exposure_token) + reg['key']
            self.ref = ts.make_off_reference(off_frames, reg['boxes'], roi_polygons=reg['polygons'],
                                             confirmed_off=True, context_key=ctx)
            self.registration, self.context = reg, ctx
        return reg

    def read(self, rgb):
        return search.read_registered(ts, np.ascontiguousarray(rgb), self.registration,
                                      context_key=self.context, off_reference=self.ref)


def draw_poly(img, poly, color, thick=1):
    cv2.polylines(img, [np.rint(np.asarray(poly)).astype(np.int32)], True, color, thick)


def location_plan(mission, counts, barcode):
    """tutor mission.plan() 42줄과 같은 정렬 (동률은 R→Y→G 유지). 퍽 할당은 하지 않음."""
    desc = mission.BARCODES[barcode]
    groups = sorted('RYG', key=lambda c: -counts[c] if desc else counts[c])
    return [(f'L{i+1}', g, counts[g]) for i, g in enumerate(groups)]


def _repo_r12():
    """이 파일이 저장소 r12/ 아래에 있으면 그 r12, 아니면 None (상대 경로 우선)."""
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'r12' and (parent / 'src/arm_practice/mission.py').is_file():
            return parent
    return None


def main():
    import pyrealsense2 as rs
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam-x', type=float, default=SPEC_OBSERVE_XY[0], help='카메라 광학 중심 맵 X (m)')
    ap.add_argument('--cam-y', type=float, default=SPEC_OBSERVE_XY[1], help='카메라 광학 중심 맵 Y (m)')
    ap.add_argument('--cam-z', type=float, required=True, help='카메라 광학 중심 높이 (m, 실측)')
    ap.add_argument('--yaw', type=float, default=180., help='카메라 방향 (deg, 180 = -X)')
    ap.add_argument('--pitch', type=float, default=0., help='아래로 숙인 각도 (deg, 실측)')
    ap.add_argument('--roll', type=float, default=0., help='카메라 롤 (deg)')
    ap.add_argument('--tower-x', type=float, default=SPEC_TOWER_XYZ[0])
    ap.add_argument('--tower-y', type=float, default=SPEC_TOWER_XYZ[1])
    ap.add_argument('--tower-z', type=float, default=SPEC_TOWER_XYZ[2], help='신호등 받침 바닥 높이 (m)')
    ap.add_argument('--radius', type=int, default=120, help='locate_body 검색 반경 px (1~256)')
    ap.add_argument('--exposure', type=float, default=None)
    ap.add_argument('--wb', type=float, default=None)
    ap.add_argument('--serial', default=None)
    ap.add_argument('--barcode', default=None, help='P2-01~04 (주면 L1~L3 정렬 표시)')
    ap.add_argument('--r12-src', default=str(_repo_r12() / 'src') if _repo_r12() else os.path.expanduser('~/git/lv2-nz2-project2/r12/src'))
    ap.add_argument('--out', default=str(HERE / 'reports/live'))
    a = ap.parse_args()
    mission = load_mission(a.r12_src)
    if a.barcode is not None and a.barcode not in mission.BARCODES:
        raise SystemExit(f'미등록 바코드 {a.barcode} (등록: {list(mission.BARCODES)})')
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)

    pipe, cfg = rs.pipeline(), rs.config()
    if a.serial:
        cfg.enable_device(a.serial)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    prof = pipe.start(cfg)
    dev = prof.get_device()
    sensor = next(s for s in dev.query_sensors() if s.get_info(rs.camera_info.name) == 'RGB Camera')

    def set_opt(opt, val):
        if sensor.supports(opt):
            try:
                sensor.set_option(opt, val)
            except Exception as e:
                print(f'[live] 옵션 실패 {opt}: {e}')

    if a.exposure is not None:
        set_opt(rs.option.enable_auto_exposure, 0); set_opt(rs.option.exposure, a.exposure)
    else:
        set_opt(rs.option.enable_auto_exposure, 1)
        print('[live] 경고: 자동 노출. OFF 기준 비교 결과는 참고만 (--exposure 권장)')
    if a.wb is not None:
        set_opt(rs.option.enable_auto_white_balance, 0); set_opt(rs.option.white_balance, a.wb)
    it = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[it.fx, 0, it.ppx], [0, it.fy, it.ppy], [0, 0, 1.]])
    print(f'[live] K fx {it.fx:.1f} fy {it.fy:.1f} cx {it.ppx:.1f} cy {it.ppy:.1f} (왜곡 모델 {it.model}, 보정 안 함)')

    T = camera_pose(a.cam_x, a.cam_y, a.cam_z, a.yaw, a.pitch, a.roll)
    tower = dict(ts.TOWER, xy_m=(a.tower_x, a.tower_y), base_z_m=a.tower_z)
    det = Detector(T, K, (HEIGHT, WIDTH, 3), tower, a.radius, f'exp={a.exposure},wb={a.wb}')
    if det.prior_reason:
        print(f'[live] 경고: 투영 실패 ({det.prior_reason}) → 카메라 자세 입력 확인. 몸통 찾기 불가')
    else:
        d = np.linalg.norm(T[:2, 3] - tower['xy_m'])
        print(f'[live] 신호등까지 수평 {d:.3f} m, 예상 칸 {det.prior_boxes}')

    for _ in range(30):
        pipe.wait_for_frames(FRAME_TIMEOUT_MS)

    recorder, rec_err, collecting = mission.SignalRecorder(), None, None
    log_f, writer, last_reg = None, None, None
    stamp = time.strftime('%Y%m%d_%H%M%S')
    try:
        while True:
            try:
                frames = pipe.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError:
                print('[live] 카메라 프레임 타임아웃 → 종료'); break
            cf = frames.get_color_frame()
            if not cf:
                continue
            bgr = np.asanyarray(cf.get_data()).copy()
            rgb = bgr[:, :, ::-1]
            t_s = cf.get_timestamp() / 1000.
            view = bgr.copy()
            if writer is not None:
                writer.write(bgr)
            if det.prior_polys:
                for p in det.prior_polys.values():
                    draw_poly(view, p, (160, 160, 160))                  # 투영 예상 위치 (회색)
            row = dict(t_s=t_s)

            if collecting is not None:                                   # o 이후 꺼짐 프레임 모음
                collecting.append(rgb.copy())
                if len(collecting) >= OFF_FRAMES:
                    last_reg = det.lock(collecting)
                    collecting = None
                    info = {k: last_reg.get(k) for k in ('found', 'reason', 'shift_px', 'scale', 'score', 'runner')}
                    print(f'[live] locate_body: {info}')
                    if last_reg.get('search_bounds'):
                        print(f"[live] 검색 범위 {last_reg['search_bounds']}")
                    row.update(event='locate_body', **{k: v for k, v in info.items()})
            if det.locked:
                res = det.read(rgb)
                col = res['color']
                for c, p in det.registration['polygons'].items():
                    draw_poly(view, p, COLOR_BGR[c], 2)                 # 찾은 칸 (색)
                if rec_err is None:
                    try:
                        recorder.observe(t_s, col)
                    except ValueError as e:
                        rec_err = str(e); print(f'[live] SignalRecorder 오류 → 기록 중단: {rec_err}')
                cv2.putText(view, col, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 0, 0), 8)
                cv2.putText(view, col, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.8, COLOR_BGR[col], 3)
                seq = ''.join(recorder.sequence)
                lines = [f"{res.get('decision') or res.get('reason')}  seq {len(seq)}/12 {seq}",
                         f"ERROR {rec_err}" if rec_err else ('COMPLETE' if recorder.complete else 'recording')]
                if recorder.complete:
                    counts = {c: seq.count(c) for c in 'RYG'}
                    lines.append(f"R{counts['R']} Y{counts['Y']} G{counts['G']}" + (
                        '  ' + ' '.join(f'{l}={g}x{n}' for l, g, n in location_plan(mission, counts, a.barcode))
                        if a.barcode else ''))
                row.update(color=col, decision=res.get('decision'), reason=res.get('reason'),
                           ratios=res.get('ratios'), support=res.get('body_support'), seq=seq, rec_err=rec_err)
            else:
                why = 'collecting OFF...' if collecting is not None else (
                    f"locate failed: {last_reg['reason']}" if last_reg and not last_reg['found'] else 'tower OFF -> press o')
                lines = [why]
            for i, t in enumerate(lines):
                y = HEIGHT - 12 - 22 * (len(lines) - 1 - i)
                cv2.putText(view, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 4)
                cv2.putText(view, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
            if log_f:
                log_f.write(json.dumps(row) + '\n')
            cv2.imshow('tower live check (tutor detection path)', view)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('o') and not det.locked and collecting is None:
                collecting = []
                print('[live] 꺼짐 확인 → 기준 프레임 수집')
            elif k == ord('r'):
                det.reset(); recorder, rec_err, last_reg = mission.SignalRecorder(), None, None
                print('[live] 해제·레코더 초기화')
            elif k == ord('l'):
                if log_f:
                    log_f.close(); log_f = None; print('[live] 기록 끔')
                else:
                    p = out / f'{stamp}_{time.strftime("%H%M%S")}.jsonl'; log_f = p.open('w'); print(f'[live] 기록 → {p}')
            elif k == ord('v'):
                if writer:
                    writer.release(); writer = None; print('[live] 녹화 끔')
                else:
                    p = out / f'{stamp}_{time.strftime("%H%M%S")}.mp4'
                    writer = cv2.VideoWriter(str(p), cv2.VideoWriter_fourcc(*'mp4v'), FPS, (WIDTH, HEIGHT))
                    print(f'[live] 녹화 → {p}')
    finally:
        if log_f:
            log_f.close()
        if writer:
            writer.release()
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
