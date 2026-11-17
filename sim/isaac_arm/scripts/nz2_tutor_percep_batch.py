#!/usr/bin/env python3
"""NZ2 튜터 인식 모델(YOLO 6클래스 + CenterPose) 실물 사진 일괄 평가 (개발 PC)

위치: ~/manipulator-isaac61-lyrical-practice/scripts/nz2_tutor_percep_batch.py
실행 환경: ~/venvs/nz2-percep (torch cu128, ultralytics 8.3.221, opencv-python)
  ~/venvs/nz2-percep/bin/python nz2_tutor_percep_batch.py --images ~/Downloads/puck

입력: 폴더 아래 *.jpg/*.png (하위 폴더 이름 = 정답 색 힌트)
  puck_g → 초록만, puck_r → 빨강만, puck_g_r → 둘 다 (그 밖의 이름은 색 검사 안 함)
출력 (--output, 기본 agent/percep/tutor_batch):
  results.jsonl   사진별 검출·자세 결과
  summary.json    폴더별 집계 (화면에도 표로 출력)
  overlay/        폴더별 앞쪽 N장 + 색 오류·미검출 사진의 표시 이미지

주의
- RGB 사진만 있으므로 [가정] 내부 파라미터(fx)로 계산한 XYZ 는 참고값. 미터 단위 정확도 판정에 쓰지 않음
- 세움/누움은 [추정]: (1) CenterPose 회전의 긴 축이 영상에서 세로인지 (2) YOLO 박스 세로/가로 비
- 로봇 구동 명령 없음 (영상 처리만)
"""
import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

# ───── 설정 상수 (한곳에서 수정) ─────
TUTOR_REPO = Path.home() / 'git/turtlebot-swerve-cuvslam'          # 튜터 저장소 (feature/nvblox-swerve-navigation)
MODEL_DIR = Path.home() / 'manipulator-isaac61-lyrical-practice/models/tutor_1111'
YOLO_PT = MODEL_DIR / 'nz2_yolo6_best.pt'
POSE_PT = MODEL_DIR / 'nz2_centerpose_best.pt'
OUT_DEFAULT = Path.home() / 'manipulator-isaac61-lyrical-practice/agent/percep/tutor_batch'
IMGSZ = 800                    # 튜터 inference.json 과 같음
DET_CONF = 0.25
POSE_VIEWS = 'scale3_median'
PUCK_DIMS_M = (0.03, 0.03, 0.06)
PUCK_CLASSES = {0: 'red_cylinder', 1: 'red_cuboid', 2: 'green_cylinder', 3: 'green_cuboid'}
FX_1280_ASSUMED = 920.0        # [가정] D435 RGB 1280×720 수평 화각 ≈ 69° → fx ≈ 640 / tan(34.5°) ≈ 930
STAND_COS = np.cos(np.deg2rad(30))   # 긴 축이 영상 세로에서 30° 이내면 세움 추정
STAND_ASPECT = 1.3                    # 박스 세로/가로 > 1.3 이면 세움 추정
FOLDER_COLOR = {'puck_g': {'green'}, 'puck_r': {'red'}, 'puck_g_r': {'red', 'green'}}

sys.path.insert(0, str(TUTOR_REPO / 'perception/scripts'))
try:
    from train_centerpose import load_model, infer   # 튜터 코드 (perception/third_party 포함)
    from ultralytics import YOLO
except ImportError as error:
    sys.exit(f'[오류] 모듈 불러오기 실패: {error}\n'
             f'  튜터 저장소 브랜치 확인: git -C {TUTOR_REPO} branch --show-current → feature/nvblox-swerve-navigation\n'
             f'  실행 python 확인: ~/venvs/nz2-percep/bin/python')


def camera_matrix(width, height, fx_1280):
    """사진 크기에 맞춰 [가정] 내부 파라미터 생성 (주점 = 영상 중심)"""
    fx = fx_1280 * width / 1280.0
    return np.array([[fx, 0, width / 2], [0, fx, height / 2], [0, 0, 1.0]])


def standing_guess(pose, box):
    """세움/누움 추정. 회전 후보가 없으면 박스 비율만 사용"""
    x1, y1, x2, y2 = box
    aspect = (y2 - y1) / max(x2 - x1, 1e-6)
    by_box = 'stand' if aspect > STAND_ASPECT else 'lie'
    by_rot = None
    if pose.get('rotation_valid') and 'rotation_matrix' in pose:
        axis = np.asarray(pose['rotation_matrix'])[:, 2]          # 물체 긴 축(z) 의 카메라 좌표 방향
        planar = np.hypot(axis[0], axis[1])
        if planar > 0.2:                                            # 카메라 쪽으로 누워 있으면 판정 보류
            by_rot = 'stand' if abs(axis[1]) / planar > STAND_COS else 'lie'
    return by_box, by_rot, float(aspect)


def draw(image, rows):
    canvas = image.copy()
    for r in rows:
        x1, y1, x2, y2 = map(int, r['bbox_xyxy'])
        color = (60, 60, 230) if r['label'].startswith('red') else (60, 200, 60) if r['label'].startswith('green') else (180, 180, 180)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        text = f"{r['label']} {r['confidence']:.2f}"
        if 'stand_box' in r:
            text += f" {r['stand_box']}/{r.get('stand_rot') or '-'}"
        if r.get('color_error'):
            text += ' COLOR?'
        cv2.putText(canvas, text, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        if r.get('center_px'):
            u, v = map(int, r['center_px'])
            cv2.drawMarker(canvas, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 12, 2)
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', type=Path, required=True)
    ap.add_argument('--output', type=Path, default=OUT_DEFAULT)
    ap.add_argument('--fx1280', type=float, default=FX_1280_ASSUMED, help='1280 폭 기준 fx [가정]')
    ap.add_argument('--overlay-per-folder', type=int, default=15)
    ap.add_argument('--max-images', type=int, default=0, help='0 = 전부 (시험용으로 20 등)')
    ap.add_argument('--cpu', action='store_true')
    a = ap.parse_args()

    files = sorted(p for p in a.images.rglob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if a.max_images:
        files = files[:a.max_images]
    if not files:
        sys.exit(f'[오류] 사진 없음: {a.images}')
    for f in (YOLO_PT, POSE_PT):
        if not f.exists():
            sys.exit(f'[오류] 모델 없음: {f}')

    device = 'cpu' if a.cpu or not torch.cuda.is_available() else 'cuda'
    yolo = YOLO(str(YOLO_PT))
    pose_model = load_model(str(POSE_PT), device)
    (a.output / 'overlay').mkdir(parents=True, exist_ok=True)
    print(f'[NZ2-PERCEP] 사진 {len(files)}장, 장치 {device}, fx1280 {a.fx1280} [가정], 출력 {a.output}')

    stats = defaultdict(lambda: Counter())
    conf_sum = defaultdict(list)
    times = []
    saved = Counter()
    with open(a.output / 'results.jsonl', 'w') as out:
        for i, path in enumerate(files):
            folder = path.parent.name
            image = cv2.imread(str(path))
            if image is None:
                stats[folder]['read_fail'] += 1
                continue
            h, w = image.shape[:2]
            K = camera_matrix(w, h, a.fx1280)
            t0 = time.perf_counter()
            boxes = yolo.predict(image, imgsz=IMGSZ, conf=DET_CONF, device=0 if device == 'cuda' else 'cpu', verbose=False)[0].boxes
            rows, color_error = [], False
            for b in boxes:
                cls = int(b.cls[0])
                box = b.xyxy[0].cpu().numpy()
                row = {'class_id': cls, 'label': yolo.names[cls], 'confidence': float(b.conf[0]), 'bbox_xyxy': box.tolist()}
                if cls in PUCK_CLASSES:
                    stats[folder][row['label']] += 1
                    conf_sum[folder].append(row['confidence'])
                    color = row['label'].split('_')[0]
                    allowed = FOLDER_COLOR.get(folder)
                    if allowed is not None and color not in allowed:
                        row['color_error'] = True
                        color_error = True
                        stats[folder]['color_error'] += 1
                    try:
                        p = infer(pose_model, image, box, K, device=device, dimensions_m=PUCK_DIMS_M, views=POSE_VIEWS)
                        row['translation_valid'] = bool(p.get('translation_valid'))
                        row['rotation_valid'] = bool(p.get('rotation_valid'))
                        row['pose_confidence'] = float(p.get('confidence', 0))
                        row['center_px'] = p.get('center_px')
                        if row['translation_valid']:
                            row['center_camera_m_assumedK'] = p['center_camera_m']
                        if row['rotation_valid']:
                            row['rotation_matrix'] = p['rotation_matrix']
                        stats[folder]['t_valid'] += row['translation_valid']
                        stats[folder]['r_valid'] += row['rotation_valid']
                        sb, sr, asp = standing_guess(p, box)
                        row.update(stand_box=sb, stand_rot=sr, bbox_aspect=asp)
                        stats[folder][f'box_{sb}'] += 1
                        if sr:
                            stats[folder][f'rot_{sr}'] += 1
                            stats[folder]['stand_agree' if sr == sb else 'stand_disagree'] += 1
                    except (cv2.error, ValueError, OverflowError, RuntimeError) as error:
                        row['pose_error'] = str(error)
                        stats[folder]['pose_error'] += 1
                else:
                    stats[folder][row['label']] += 1
                rows.append(row)
            if device == 'cuda':
                torch.cuda.synchronize()
            times.append((time.perf_counter() - t0) * 1000)
            n_puck = sum(r['class_id'] in PUCK_CLASSES for r in rows)
            stats[folder]['images'] += 1
            stats[folder]['puck_dets'] += n_puck
            if n_puck == 0:
                stats[folder]['no_puck_images'] += 1
            out.write(json.dumps({'image': str(path.relative_to(a.images)), 'size': [w, h], 'detections': rows}) + '\n')
            if saved[folder] < a.overlay_per_folder or color_error or n_puck == 0:
                tag = '_COLOR' if color_error else '_NOPUCK' if n_puck == 0 else ''
                cv2.imwrite(str(a.output / 'overlay' / f'{folder}__{path.stem}{tag}.jpg'), draw(image, rows))
                saved[folder] += 1
            if (i + 1) % 50 == 0:
                print(f'  {i + 1}/{len(files)}')

    summary = {'images_root': str(a.images), 'fx1280_assumed': a.fx1280, 'device': device,
               'mean_ms_per_image': float(np.mean(times)) if times else None, 'folders': {}}
    print(f"\n{'폴더':10} {'사진':>5} {'퍽검출':>6} {'무검출':>6} {'색오류':>6} {'위치유효':>8} {'회전유효':>8} {'세움(박스)':>9} {'누움(박스)':>9} {'박스=회전':>9} {'평균conf':>8}")
    for folder, s in sorted(stats.items()):
        n = max(s['puck_dets'], 1)
        summary['folders'][folder] = dict(s, mean_conf=float(np.mean(conf_sum[folder])) if conf_sum[folder] else None)
        agree = s['stand_agree'] + s['stand_disagree']
        print(f"{folder:10} {s['images']:>5} {s['puck_dets']:>6} {s['no_puck_images']:>6} {s['color_error']:>6} "
              f"{s['t_valid'] / n:>8.0%} {s['r_valid'] / n:>8.0%} {s['box_stand']:>9} {s['box_lie']:>9} "
              f"{(s['stand_agree'] / agree if agree else 0):>9.0%} {np.mean(conf_sum[folder]) if conf_sum[folder] else 0:>8.2f}")
        print('           클래스별:', {k: s[k] for k in list(PUCK_CLASSES.values()) + ['box', 'apriltag'] if s[k]})
    print(f"\n평균 처리 {summary['mean_ms_per_image']:.1f} ms/장 ({device})")
    (a.output / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f'[NZ2-PERCEP] 저장: {a.output}/summary.json, results.jsonl, overlay/')


if __name__ == '__main__':
    main()
