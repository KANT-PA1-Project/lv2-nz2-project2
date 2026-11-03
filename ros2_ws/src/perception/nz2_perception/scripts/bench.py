"""Jetson 에서 엔진 속도/결과 확인.

  python jetson/bench.py --model models/current.engine                # 더미 입력 FPS
  python jetson/bench.py --model models/current.engine --src imgs/   # 폴더 사진 → bench_out/ 에 결과 저장
  python jetson/bench.py --model models/current.engine --src d435    # D435 실시간 (pyrealsense2 필요)
  python jetson/bench.py --model models/current.pt --compare models/current.engine --src imgs/
      → .pt 와 엔진 결과 박스 수/conf 비교 (FP16 변환 후 성능 떨어졌는지)
"""
import argparse, time
from pathlib import Path
import numpy as np
import cv2
from ultralytics import YOLO

CONF = 0.25


def frames(src, n):
    if src is None:
        for _ in range(n):
            yield "dummy", np.random.randint(0, 255, (480, 848, 3), np.uint8)
    elif src == "d435":
        import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        try:
            for i in range(n):
                f = pipe.wait_for_frames().get_color_frame()
                yield f"d435_{i:04d}", np.asanyarray(f.get_data())
        finally:
            pipe.stop()
    else:
        files = sorted(p for p in Path(src).rglob("*") if p.suffix.lower() in (".jpg", ".png", ".jpeg"))
        for p in files[:n]:
            yield p.stem, cv2.imread(str(p))


def run(model_path, args, save=True):
    m = YOLO(model_path, task="detect")
    half = not str(model_path).endswith(".pt")   # .pt 는 FP32 그대로 → 엔진(FP16)과 비교해야 변환 손실이 보임
    imgsz = args.imgsz if len(args.imgsz) == 2 else args.imgsz[0]
    dummy = np.zeros((480, 848, 3), np.uint8)
    for _ in range(10):  # 워밍업
        m.predict(dummy, imgsz=imgsz, conf=CONF, verbose=False, half=half)
    times, out = [], {}
    outdir = Path("bench_out"); outdir.mkdir(exist_ok=True)
    for name, img in frames(args.src, args.n):
        t = time.perf_counter()
        r = m.predict(img, imgsz=imgsz, conf=CONF, verbose=False, half=half)[0]
        times.append(time.perf_counter() - t)
        out[name] = [(m.names[int(c)], float(s)) for c, s in zip(r.boxes.cls, r.boxes.conf)]
        if save and args.src not in (None,):
            cv2.imwrite(str(outdir / f"{name}.jpg"), r.plot())
    t = np.array(times[2:] if len(times) > 4 else times) * 1000
    print(f"[{model_path}] {len(times)}장  평균 {t.mean():.1f} ms  p95 {np.percentile(t,95):.1f} ms  "
          f"→ {1000/t.mean():.1f} FPS (전처리+추론+후처리)")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--compare", help="두 번째 모델 (보통 .engine)")
    ap.add_argument("--src", default=None, help="이미지 폴더 / d435 / 생략=더미")
    ap.add_argument("--imgsz", type=int, nargs="+", default=[384, 640])
    ap.add_argument("-n", type=int, default=200)
    args = ap.parse_args()

    a = run(args.model, args)
    if args.compare:
        b = run(args.compare, args, save=False)
        diff = [k for k in a if len(a[k]) != len(b.get(k, []))]
        print(f"박스 개수 다른 사진: {len(diff)}/{len(a)}", diff[:20])
        ca = [c for v in a.values() for _, c in v]; cb = [c for v in b.values() for _, c in v]
        if ca and cb:
            print(f"평균 conf  {args.model}: {np.mean(ca):.3f}   {args.compare}: {np.mean(cb):.3f}")


if __name__ == "__main__":
    main()
