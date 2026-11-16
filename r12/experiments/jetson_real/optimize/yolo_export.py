#!/usr/bin/env python3
"""팀원 YOLO(.pt) → ONNX 여러 입력 크기로 내보내고 PyTorch 와 출력이 같은지 확인 [2026-11-04]

대상: drive-download-.../yolo_best.pt = YOLOv8n-P2, 클래스 6개(red/green cylinder·cuboid, box, apriltag), 학습 imgsz 800
목적: Jetson 에서 PyTorch 없이 TensorRT 로 돌리기 위한 첫 단계 (메모리 1 GB 이하·속도 목표)
내보내는 크기 (기본):
    800x800  학습 크기 그대로 (기준)
    448x800  카메라 848x480 비율 (위아래 빈칸 없음 → 계산량 약 44% 감소)
    640x640  일반 크기
    384x640  우리 v8 과 같은 크기 (가장 가벼움)
설정: opset 17, simplify, 입력 크기 고정(dynamic=False), NMS 미포함(후처리는 노드에서), FP32 (FP16 은 TensorRT 가 변환)
검증 (크기마다):
    ① 같은 입력 텐서로 PyTorch vs ONNX Runtime 원시 출력 최대 오차 (1e-3 이하면 OK)
    ② 실제 사진 몇 장을 같은 전처리(letterbox 고정 크기, 회색 114)로 넣어 원시 출력 오차 + NMS 후 검출(클래스·점수) 비교
실행 (개발 PC, yolo_env):
    cd ~/git/p2_perception && source ~/yolo_env/bin/activate
    pip install onnx onnxslim onnxruntime        # 없으면
    python optimize/yolo_export.py
    python optimize/yolo_export.py --sizes 448x800 --images data/real/yolo/1026_tower/images/test
결과: optimize/onnx/yolo_<H>x<W>.onnx + optimize/onnx/yolo_export_report.json
"""
import argparse
import glob
import json
import os
import shutil
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PT = "drive-download-20261104T022039Z-1-001/yolo_best.pt"


def parse_size(s):
    h, w = s.lower().split("x")
    return int(h), int(w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pt", default=DEFAULT_PT)
    ap.add_argument("--sizes", nargs="+", default=["800x800", "448x800", "640x640", "384x640"], help="HxW")
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--out", default="optimize/onnx")
    ap.add_argument("--images", default="", help="비교용 사진 폴더 (기본: data/real/yolo/*/images/test 에서 8장)")
    ap.add_argument("--n_images", type=int, default=8)
    ap.add_argument("--conf", type=float, default=0.25)
    a = ap.parse_args()

    import cv2
    import onnxruntime as ort
    import torch
    from ultralytics import YOLO
    from ultralytics.data.augment import LetterBox
    try:
        from ultralytics.utils.ops import non_max_suppression
    except ImportError:                      # 최신 ultralytics 는 utils.nms 로 옮김
        from ultralytics.utils.nms import non_max_suppression

    pt = a.pt if os.path.isabs(a.pt) else os.path.join(ROOT, a.pt)
    out = a.out if os.path.isabs(a.out) else os.path.join(ROOT, a.out)
    os.makedirs(out, exist_ok=True)
    if not os.path.exists(pt):
        raise SystemExit(f"모델 없음: {pt}")

    base = YOLO(pt)
    print(f"모델: {pt}\n  task {base.task}, 클래스 {base.names}")
    tm = base.model.float().eval()
    try:
        tm = tm.fuse()           # Conv+BN 합치기 (내보낸 ONNX 와 같은 구조로 비교)
    except Exception:
        pass

    if a.images:
        imgs = sorted(glob.glob(os.path.join(a.images, "*.jpg")) + glob.glob(os.path.join(a.images, "*.png")))
    else:
        imgs = sorted(glob.glob(os.path.join(ROOT, "data/real/yolo/*/images/test/*.jpg")))
    imgs = imgs[:: max(1, len(imgs) // max(1, a.n_images))][: a.n_images]
    print(f"비교용 사진 {len(imgs)}장")

    report = {"pt": pt, "names": base.names, "opset": a.opset, "sizes": {}}
    for s in a.sizes:
        h, w = parse_size(s)
        if h % 32 or w % 32:
            print(f"[{s}] 32 의 배수가 아님 → 건너뜀")
            continue
        print(f"\n===== {h}x{w} =====", flush=True)
        t0 = time.time()
        m = YOLO(pt)
        f = m.export(format="onnx", imgsz=[h, w], opset=a.opset, simplify=True, dynamic=False,
                     half=False, batch=1, device="cpu")
        dst = os.path.join(out, f"yolo_{h}x{w}.onnx")
        shutil.move(f, dst)
        mb = os.path.getsize(dst) / 1e6
        print(f"  ONNX {dst} ({mb:.1f} MB, {time.time() - t0:.0f} s)")

        # ① 원시 출력 비교
        sess = ort.InferenceSession(dst, providers=["CPUExecutionProvider"])
        inp = sess.get_inputs()[0]
        x = np.random.default_rng(0).random((1, 3, h, w), dtype=np.float32)
        with torch.no_grad():
            yt = tm(torch.from_numpy(x))
        yt = (yt[0] if isinstance(yt, (list, tuple)) else yt).numpy()
        yo = sess.run(None, {inp.name: x})[0]
        diff = float(np.max(np.abs(yt - yo))) if yt.shape == yo.shape else None
        print(f"  입력 {inp.name}{inp.shape} → 출력 {yo.shape}, PyTorch 대비 최대 오차 {diff}")

        # ② 실제 사진: 같은 전처리 텐서로 PyTorch vs ONNX → 원시 오차 + NMS 후 검출 비교
        #    (ultralytics predict 는 .pt 일 때 최소 패딩(auto) 이라 고정 크기 ONNX 와 입력이 달라짐 → 직접 전처리)
        rows = []
        lb = LetterBox(new_shape=(h, w), auto=False, scaleup=True)
        for p in imgs:
            im = cv2.imread(p)
            if im is None:
                continue
            t = lb(image=im)[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            t = np.ascontiguousarray(t)
            with torch.no_grad():
                rt = tm(torch.from_numpy(t))
            rt = rt[0] if isinstance(rt, (list, tuple)) else rt
            ro = torch.from_numpy(sess.run(None, {inp.name: t})[0])
            raw = float((rt - ro).abs().max())
            dp = non_max_suppression(rt, conf_thres=a.conf, iou_thres=0.7)[0]
            do = non_max_suppression(ro, conf_thres=a.conf, iou_thres=0.7)[0]
            cp = sorted(int(c) for c in dp[:, 5].tolist())
            co = sorted(int(c) for c in do[:, 5].tolist())
            sp = sorted(float(v) for v in dp[:, 4].tolist())
            so = sorted(float(v) for v in do[:, 4].tolist())
            dmax = max([abs(u - v) for u, v in zip(sp, so)] or [0.0])
            rows.append(dict(img=os.path.basename(p), n_pt=len(cp), n_onnx=len(co), same_cls=cp == co,
                             raw_max_abs_diff=round(raw, 6), score_diff_max=round(dmax, 4)))
        same = sum(r["same_cls"] for r in rows)
        print(f"  사진 {len(rows)}장: 검출 수 {[r['n_pt'] for r in rows]}, 클래스 같음 {same}/{len(rows)}, "
              f"원시 오차 최대 {max([r['raw_max_abs_diff'] for r in rows] or [0]):.2e}, "
              f"점수 차이 최대 {max([r['score_diff_max'] for r in rows] or [0]):.4f}")
        report["sizes"][f"{h}x{w}"] = dict(onnx=dst, mb=round(mb, 2), out_shape=list(yo.shape),
                                          raw_max_abs_diff=diff, images=rows)

    rp = os.path.join(out, "yolo_export_report.json")
    json.dump(report, open(rp, "w"), indent=1, ensure_ascii=False)
    print("\n================ 요약 ================")
    print(f"{'크기':>9s} {'MB':>6s} {'출력':>16s} {'원시오차':>10s} {'사진 클래스 같음':>16s}")
    for k, v in report["sizes"].items():
        ok = sum(r["same_cls"] for r in v["images"])
        print(f"{k:>9s} {v['mb']:6.1f} {str(v['out_shape']):>16s} {str(v['raw_max_abs_diff'])[:9]:>10s} "
              f"{ok:>8d}/{len(v['images'])}")
    print(f"저장: {rp}")
    print("판정: 원시 오차 1e-3 이하 + 사진 클래스 전부 같음 → ONNX 정상. 다음 = TensorRT FP16 (메모리·속도 측정)")


if __name__ == "__main__":
    main()
