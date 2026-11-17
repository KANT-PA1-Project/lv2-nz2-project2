#!/usr/bin/env python3
"""Jetson 용 YOLO + CenterPose TensorRT 파이프라인 — torch·ultralytics 없이 (메모리 1 GB 판정용) [2026-11-04, 11-06 원본 맞춤]

왜: ultralytics/torch 를 불러오면 그것만으로 수백 MB → 실제 배포 메모리와 다름.
    이 스크립트는 TensorRT + numpy + OpenCV + (카메라면) pyrealsense2 만 쓴다.
    CUDA 파이썬 바인딩(cuda-python/pycuda)이 없어서 libcudart 를 ctypes 로 직접 부른다 (설치 불필요).
흐름: 카메라/사진 → letterbox → YOLO 엔진 → NMS → 퍽(cls 0~3)마다 crop 3장 → CenterPose 엔진 1회(batch 3)
      → 장마다 디코딩 + PnP·학습 깊이로 3D → 3D 위치 중앙값 → 화면에 그림
      두 엔진을 한 프로세스에서 불러옴 (CUDA 컨텍스트 1개) = 실제 노드와 같은 조건
CenterPose 전후처리 = 팀 참조 코드 train_centerpose.py 그대로 옮김 (numpy·OpenCV):
      ~/.cache/isaac-fleet-r12/mission_native_perception_v23/work/perception_implementation/scripts/train_centerpose.py
      (체크포인트 저장 형식·scope 문자열이 centerpose_best.pt 와 같음 → 같은 계열. 정확한 버전은 팀원 확인)
      crop_image   : 박스 긴 변 x 배율 정사각, 실수 좌표 affine(warpAffine, 회색 114) → 128, BGR→RGB, /255
      scale3_median: 배율 1.5 / 1.35 / 1.65 (이동 없음) 3장
      decode       : sigmoid 최고점 → 중심 (+reg)·꼭짓점 (+hps) x4 → affine 역변환 → 원본 픽셀
                     PnP(EPNP + LM, 퍽 치수 0.03x0.03x0.06 m) → 회전, 재투영 오차 < 10 px·점수 ≥ 0.25 → 회전 유효
                     scale[0] = log 깊이 → 깊이 = exp(scale0)·fx·max(치수)·(128/변)/128, 위치 = 중심 픽셀 광선 x 깊이
      합치기       : 유효한 3D 위치들의 중앙값. 회전·꼭짓점·점수는 첫 장(1.5배) 결과 그대로
      카메라 K     : realsense 면 스트림 내부 파라미터, 사진이면 --K (없으면 fx=fy=1.28·높이 [가정, D435 컬러 일반값])
메모리 표시: RSS = 이 프로세스, 시스템+ = 시작 전 대비 MemAvailable 감소 (Jetson 은 CPU·GPU 공유 메모리라 이게 실제 사용량에 가까움.
            다른 프로그램이 메모리를 쓰면 같이 잡히므로 다른 노드는 끄고 잴 것). 50 Hz 별도 스레드로 로드·워밍업 포함 최고값 기록
시간 표시: wait = 카메라 프레임 기다린 시간, age = 카메라 시각 → 처리 시작 (시각 영역이 system/global 일 때만),
           total = 처리 시작 → YOLO·NMS·CenterPose·PnP 끝 (화면 표시 제외). 요약에 퍽 개수별 시간
실행 (Jetson, ~/git/jetson-edge-computing-labs 에서):
    sudo jetson_clocks
    tegrastats --interval 1000          # 다른 터미널 (GR3D_FREQ = GPU 사용률, RAM = 전체 메모리)
    python3 optimize/jetson_pipeline.py --source realsense --width 640 --height 480 --headless --frames 1800
    python3 optimize/jetson_pipeline.py --source optimize/acc_ref_43real/images --headless --frames 600   # 사진 반복 (부하 시험)
    python3 optimize/jetson_pipeline.py --check_ref optimize/onnx_cp/ref_128x128_b3_h4         # CenterPose FP16 출력 정합 (난수 입력)
    python3 optimize/jetson_pipeline.py --acc_ref optimize/acc_ref_43real                      # 원본 PyTorch 결과 대비 정확도
옵션: --yolo optimize/engines/yolo_800x800_base_jetson.engine (기본 = 정확도 통과 우선 후보, 모드 잠정 base)
      --cp optimize/engines/centerpose_128x128_b3_h4_base_jetson.engine (기본) --no_cp (YOLO 만)
      --conf 0.25 --iou 0.7 --crop_scales 1.5 1.35 1.65 --K fx fy cx cy --width 640 --height 480 --fps 30
결과: outputs/pipeline/<시각>/timings.csv (프레임별 시간·메모리) + summary.json, s 키 / --save_every 로 사진 저장
주의: 화면 표시(cv2.imshow)는 GUI 라이브러리 때문에 메모리가 더 듦 → 1 GB 판정은 --headless 값으로
"""
import os

os.environ.setdefault("CUDA_MODULE_LOADING", "LAZY")   # CUDA 라이브러리를 실제로 쓸 때만 올림 (메모리 절약). 반드시 import 전에


def mem_available_mb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024.0
    return float("nan")


def rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return float("nan")


BASE_AVAIL = mem_available_mb()   # 무엇도 불러오기 전 기준값

import argparse  # noqa: E402
import ctypes  # noqa: E402
import ctypes.util  # noqa: E402
import csv  # noqa: E402
import glob  # noqa: E402
import json  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

YOLO_NAMES = ["red_cylinder", "red_cuboid", "green_cylinder", "green_cuboid", "box", "apriltag"]   # yolo_best.pt 순서
CP_HEADS = ["hm", "wh", "hps", "reg", "hm_hp", "hp_offset", "scale"]                                  # 헤드 7개 (names.json 없을 때)
CP_NEED = {"hm", "hps", "reg", "scale"}                                                                # 디코딩에 필요한 출력
CP_CLASSES = {0, 1, 2, 3}                                     # tutor: cls < 4 (퍽 4종)만 CenterPose. box·apriltag 는 검출만
CROP_SCALES = [1.5, 1.35, 1.65]                               # 원본 scale3_median: 배율 3개, 이동 없음
PUCK_DIMS = (0.03, 0.03, 0.06)                                # 원본 infer 기본값 (worker 도 cls<4 에 같은 값)
CORNERS = np.array([[x, y, z] for x in (-.015, .015) for y in (-.015, .015) for z in (-.03, .03)], np.float64)  # 원본 순서
EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]          # 육면체 모서리 12개


def _sym_rotations():
    """정사각 기둥의 대칭 회전 8개 (원본 symmetry_permutations 와 같은 군): z축 90° x 4 · (그대로 / x축 180°)"""
    out = []
    for k in range(4):
        a = k * np.pi / 2
        rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        for flip in (np.eye(3), np.diag([1., -1., -1.])):
            out.append(rz @ flip)
    return out


SYM_R = _sym_rotations()


def default_K(w, h):
    """사진에 K 가 없을 때 [가정]: D435 컬러 fx≈fy≈1.28·높이 (640x480·848x480 ≈ 615, 1280x720 ≈ 920), 주점 = 가운데"""
    f = 1.28 * h
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], np.float64)


class MemSampler(threading.Thread):
    """50 Hz 로 시스템 메모리 감소·RSS 최고값 기록 (프레임 사이 순간 피크 포함). phase 별 최고값도"""

    def __init__(self, period=0.02):
        super().__init__(daemon=True)
        self.period, self.phase, self.stop_ev = period, "start", threading.Event()
        self.peak = dict(sys=0.0, rss=0.0)
        self.by_phase = {}

    def run(self):
        while not self.stop_ev.is_set():
            s, r = BASE_AVAIL - mem_available_mb(), rss_mb()
            self.peak["sys"], self.peak["rss"] = max(self.peak["sys"], s), max(self.peak["rss"], r)
            p = self.by_phase.setdefault(self.phase, dict(sys=0.0, rss=0.0))
            p["sys"], p["rss"] = max(p["sys"], s), max(p["rss"], r)
            time.sleep(self.period)

    def stop(self):
        self.stop_ev.set()
        self.join(1.0)


# ---------------------------------------------------------------- CUDA (ctypes)
class Cuda:
    H2D, D2H = 1, 2

    def __init__(self):
        cands = [ctypes.util.find_library("cudart"), "libcudart.so", "libcudart.so.13", "libcudart.so.12",
                 "/usr/local/cuda/lib64/libcudart.so", "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so"]
        self.lib, err = None, None
        for c in cands:
            if not c:
                continue
            try:
                self.lib = ctypes.CDLL(c)
                break
            except OSError as e:
                err = e
        if self.lib is None:
            raise SystemExit(f"libcudart 를 못 찾음 ({err}). ls /usr/local/cuda/lib64/libcudart* 확인")
        L = self.lib
        vp, sz = ctypes.c_void_p, ctypes.c_size_t
        L.cudaMalloc.argtypes = [ctypes.POINTER(vp), sz]
        L.cudaFree.argtypes = [vp]
        L.cudaMemcpyAsync.argtypes = [vp, vp, sz, ctypes.c_int, vp]
        L.cudaStreamCreate.argtypes = [ctypes.POINTER(vp)]
        L.cudaStreamSynchronize.argtypes = [vp]
        L.cudaGetErrorString.argtypes = [ctypes.c_int]
        L.cudaGetErrorString.restype = ctypes.c_char_p
        s = vp()
        self._ck(L.cudaStreamCreate(ctypes.byref(s)), "cudaStreamCreate")
        self.stream = s

    def _ck(self, rc, what):
        if rc != 0:
            raise RuntimeError(f"{what} 실패: {self.lib.cudaGetErrorString(rc).decode()}")

    def malloc(self, nbytes):
        p = ctypes.c_void_p()
        self._ck(self.lib.cudaMalloc(ctypes.byref(p), max(1, nbytes)), "cudaMalloc")
        return p

    def h2d(self, dptr, arr):
        self._ck(self.lib.cudaMemcpyAsync(dptr, arr.ctypes.data, arr.nbytes, self.H2D, self.stream), "H2D")

    def d2h(self, arr, dptr):
        self._ck(self.lib.cudaMemcpyAsync(arr.ctypes.data, dptr, arr.nbytes, self.D2H, self.stream), "D2H")

    def sync(self):
        self._ck(self.lib.cudaStreamSynchronize(self.stream), "cudaStreamSynchronize")


# ---------------------------------------------------------------- TensorRT 엔진
class TrtModel:
    def __init__(self, trt, logger, cuda, path):
        self.cuda = cuda
        self.path = path
        buf = open(path, "rb").read()
        self.file_mb = round(len(buf) / 1e6, 2)
        off = 0                                    # ultralytics 엔진이면 앞에 [4바이트 길이 + JSON] → 건너뜀
        if len(buf) > 8:
            n = int.from_bytes(buf[:4], "little", signed=True)
            if 0 < n < len(buf) - 4 and buf[4:5] == b"{":
                off = 4 + n
        self.runtime = trt.Runtime(logger)
        self.engine = self.runtime.deserialize_cuda_engine(buf[off:])
        del buf
        if self.engine is None:
            raise SystemExit(f"엔진 역직렬화 실패: {path} (다른 기기/TensorRT 버전에서 만든 엔진이면 Jetson 에서 다시 빌드)")
        self.ctx = self.engine.create_execution_context()
        # 엔진이 실행 중 쓰는 GPU 작업 메모리(활성값·scratch). base/lowmem 차이를 잡음 없이 비교하는 값
        dm = None
        for attr in ("device_memory_size_v2", "device_memory_size"):
            try:
                dm = int(getattr(self.engine, attr))
                break
            except Exception:
                continue
        self.device_mem_mb = None if dm is None else round(dm / 1e6, 2)
        self.inputs, self.outputs, self.host, self.dev = [], [], {}, {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            if any(d < 0 for d in shape):
                raise SystemExit(f"{path}: 동적 shape {name}{shape} 은 지원 안 함 (고정 크기로 빌드)")
            dt = trt.nptype(self.engine.get_tensor_dtype(name))
            self.host[name] = np.empty(shape, dtype=dt)
            self.dev[name] = cuda.malloc(self.host[name].nbytes)
            self.ctx.set_tensor_address(name, self.dev[name].value)
            (self.inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else self.outputs).append(name)
        if len(self.inputs) != 1:
            raise SystemExit(f"{path}: 입력이 1개가 아님 {self.inputs}")
        self.in_name = self.inputs[0]
        self.in_shape = self.host[self.in_name].shape

    def __call__(self, x):
        h = self.host[self.in_name]
        np.copyto(h, x.astype(h.dtype, copy=False))
        self.cuda.h2d(self.dev[self.in_name], h)
        if not self.ctx.execute_async_v3(self.cuda.stream.value):
            raise RuntimeError("execute_async_v3 실패")
        for n in self.outputs:
            self.cuda.d2h(self.host[n], self.dev[n])
        self.cuda.sync()
        return {n: self.host[n] for n in self.outputs}


# ---------------------------------------------------------------- YOLO 전후처리 (ultralytics 와 같은 규칙)
def letterbox(img, H, W):
    h0, w0 = img.shape[:2]
    s = min(H / h0, W / w0)
    nw, nh = int(round(w0 * s)), int(round(h0 * s))
    dw, dh = (W - nw) / 2, (H - nh) / 2
    top, left = int(round(dh - 0.1)), int(round(dw - 0.1))
    im = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w0, h0) else img
    im = cv2.copyMakeBorder(im, top, H - nh - top, left, W - nw - left, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    x = im[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0      # BGR→RGB, /255, NCHW
    return np.ascontiguousarray(x), s, left, top


def yolo_decode(out, conf, iou, s, padx, pady, shape0, max_det=300):
    """out [1, 4+nc, N] (cx, cy, w, h, 클래스 점수) → [(x1, y1, x2, y2, score, cls)] 원본 픽셀"""
    p = out[0]
    scores = p[4:].T
    cls = scores.argmax(1)
    sc = scores[np.arange(len(cls)), cls]
    m = sc >= conf
    if not m.any():
        return []
    b, sc, cls = p[:4].T[m], sc[m], cls[m]
    xyxy = np.stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2, b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2], 1)
    off = cls[:, None].astype(np.float32) * 4096.0                                 # 클래스별 NMS (좌표를 클래스마다 떨어뜨림)
    rect = np.concatenate([xyxy[:, :2] + off, xyxy[:, 2:] - xyxy[:, :2]], 1)
    keep = cv2.dnn.NMSBoxes(rect.tolist(), sc.tolist(), conf, iou)
    keep = np.array(keep).reshape(-1)[:max_det]
    h0, w0 = shape0
    res = []
    for k in keep:
        x1, y1, x2, y2 = xyxy[k]
        x1, x2 = np.clip([(x1 - padx) / s, (x2 - padx) / s], 0, w0)
        y1, y2 = np.clip([(y1 - pady) / s, (y2 - pady) / s], 0, h0)
        res.append((float(x1), float(y1), float(x2), float(y2), float(sc[k]), int(cls[k])))
    return res


# ---------------------------------------------------------------- CenterPose 전후처리 (train_centerpose.py 와 같게)
def cp_crop(img, box, crop_scale=1.5, shift=(0.0, 0.0), size=128):
    """원본 crop_image: 긴 변 x 배율 정사각을 실수 좌표 affine 으로 size 에 맞춤 (회색 114), BGR→RGB, /255, CHW"""
    x1, y1, x2, y2 = (float(v) for v in box)
    side = max(max(x2 - x1, y2 - y1) * crop_scale, 1e-3)
    cx, cy = (x1 + x2) / 2 + shift[0] * side, (y1 + y2) / 2 + shift[1] * side
    m = np.array([[size / side, 0, size / 2 - cx * size / side], [0, size / side, size / 2 - cy * size / side]], np.float32)
    crop = cv2.warpAffine(img, m, (size, size), borderValue=(114, 114, 114))
    x = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).transpose(2, 0, 1).astype(np.float32) / 255.0
    return x, m


def cp_decode_view(out, b, m, K, dims=PUCK_DIMS, size=128, metric_depth=True, pnp=True):
    """원본 decode_prediction: batch 의 b 번째 → 점수, 중심·꼭짓점(원본 픽셀), PnP 회전, 학습 깊이 3D 위치, 유효 판정
    pnp=False: PnP 생략 (학습 깊이 모드에서 2·3번째 장은 위치만 쓰이므로 결과 동일, CPU 시간 절약)"""
    grid = out["hm"].shape[-1]
    heat = (1.0 / (1.0 + np.exp(-out["hm"][b, 0].astype(np.float32)))).astype(np.float32)
    y, x = divmod(int(np.argmax(heat)), grid)
    score = float(heat[y, x])
    ij = np.array([x, y], np.float32)
    ct = (ij + out["reg"][b, :, y, x].astype(np.float32)) * 4
    corners = (out["hps"][b, :, y, x].astype(np.float32).reshape(8, 2) + ij) * 4
    inv = cv2.invertAffineTransform(m)
    pixels = corners @ inv[:, :2].T + inv[:, 2]
    center_px = ct @ inv[:, :2].T + inv[:, 2]
    obj = CORNERS * np.asarray(dims) / np.array([.03, .03, .06])
    dist = np.zeros(5)
    r = dict(confidence=score, center_px=center_px, corners_px=pixels, valid=False)
    ok, rvec, tvec = (cv2.solvePnP(obj, pixels.astype(np.float64), K, dist, flags=cv2.SOLVEPNP_EPNP) if pnp
                      else (False, None, None))
    if ok:
        rvec, tvec = cv2.solvePnPRefineLM(obj, pixels.astype(np.float64), K, dist, rvec, tvec)
        reproj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = float(np.linalg.norm(reproj.reshape(-1, 2) - pixels, axis=1).mean())
        valid = bool(tvec[2, 0] > 0 and np.isfinite(tvec).all() and err < 10 and score >= .25)
        r.update(valid=valid, center_camera_m=tvec.reshape(3), rotation_matrix=cv2.Rodrigues(rvec)[0],
                 reprojection_error_px=err)
    if metric_depth:
        depth = float(np.exp(float(out["scale"][b, 0, y, x])) * K[0, 0] * max(dims) * m[0, 0] / size)
        ray = np.linalg.solve(K, np.r_[center_px, 1.0])
        r["pnp_valid"] = r["valid"]
        r["pnp_center_camera_m"] = r.get("center_camera_m")
        r["center_camera_m"] = ray * depth
        r["translation_valid"] = bool(np.isfinite(depth) and depth > 0 and score >= .25)
        r["rotation_valid"] = r["pnp_valid"]
        r["valid"] = r["translation_valid"] and r["rotation_valid"]
    r.setdefault("translation_valid", r["valid"])
    r.setdefault("rotation_valid", r["valid"])
    return r


def cp_pose(run, img, box, K, scales=CROP_SCALES, size=128, dims=PUCK_DIMS, metric_depth=True):
    """원본 infer(views='scale3_median'): crop len(scales)장 → 엔진 1회 → 장별 디코딩 →
    유효한 3D 위치 중앙값, 회전·꼭짓점·점수는 첫 장. 반환 (결과 dict, 장별 dict 목록)"""
    crops = [cp_crop(img, box, s, (0.0, 0.0), size) for s in scales]
    out = run(np.ascontiguousarray(np.stack([c[0] for c in crops])))
    # 원본: 회전·꼭짓점·유효(회전)는 첫 장만, 나머지 장은 학습 깊이 위치만 사용 → 2·3번째 장 PnP 는 결과에 영향 없음 (생략)
    est = [cp_decode_view(out, j, mm, K, dims, size, metric_depth, pnp=(j == 0 or not metric_depth))
           for j, (_, mm) in enumerate(crops)]
    if len(est) == 1:
        return est[0], est
    res = dict(est[0])
    pts = [e["center_camera_m"] for e in est if e["translation_valid"] and np.isfinite(e["center_camera_m"]).all()]
    res["translation_valid"] = bool(pts)
    res["translation_views_used"] = len(pts)
    if pts:
        res["center_camera_m"] = np.median(np.array(pts), axis=0)
    res["valid"] = res["translation_valid"] and res["rotation_valid"]
    return res, est


def rot_err_deg(Ra, Rb, sym=False):
    """두 회전 차이(도). sym=True 면 정사각 기둥 대칭 8개 중 최소 (꼭짓점 순서가 대칭으로 바뀐 경우 무시)"""
    Ra, Rb = np.asarray(Ra, np.float64), np.asarray(Rb, np.float64)

    def ang(R):
        return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    if not sym:
        return ang(Ra.T @ Rb)
    return min(ang((Ra @ S).T @ Rb) for S in SYM_R)


def draw_cuboid(canvas, kps, center, ok):
    col = (0, 255, 255) if ok else (150, 150, 150)
    p = [tuple(int(v) for v in k) for k in kps]
    for i, j in EDGES:
        cv2.line(canvas, p[i], p[j], col, 2 if ok else 1, cv2.LINE_AA)
    cv2.circle(canvas, (int(center[0]), int(center[1])), 4, (0, 0, 255) if ok else (120, 120, 120), -1)


def to_jsonable(o):
    if isinstance(o, dict):
        return {k: to_jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [to_jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return to_jsonable(o.tolist())
    if isinstance(o, (np.floating, float)):
        return float(o) if np.isfinite(o) else None
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


# ---------------------------------------------------------------- 실행
def check_ref(cp, ref_dir):
    x = np.load(os.path.join(ref_dir, "input.npy"))
    if tuple(x.shape) != tuple(cp.in_shape):
        raise SystemExit(f"ref 입력 {x.shape} ≠ 엔진 입력 {cp.in_shape}")
    out = cp(x)
    print(f"CenterPose 엔진 vs PyTorch reference ({ref_dir})")
    print("주의: input.npy 는 0~1 난수 입력 → 엔진 정상 동작 확인용 (정확도는 --acc_ref)")
    worst = 0.0
    nf = os.path.join(ref_dir, "names.json")
    heads = json.load(open(nf)) if os.path.exists(nf) else CP_HEADS
    for i, name in enumerate(heads):
        ref = np.load(os.path.join(ref_dir, f"reference_{i}.npy"))
        got = out[name].astype(np.float32)
        if got.shape != ref.shape:
            print(f"  {name:10s} shape 다름 {got.shape} ≠ {ref.shape}")
            worst = float("inf")
            continue
        d = float(np.max(np.abs(got - ref)))
        rel = d / max(float(np.max(np.abs(ref))), 1e-6)
        worst = max(worst, rel)
        print(f"  {name:10s} {str(list(ref.shape)):18s} 최대 오차 {d:.4f}  (값 크기 대비 {rel * 100:.2f}%)")
    print(f"판정(FP16 기준 값 크기 대비 1% 이하): {'OK' if worst <= 0.01 else '확인 필요'}")


def box_iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


THRESHOLDS = (0.25, 0.5, 0.7)        # 0.25 = 검출 임계값(tutor detector_confidence), 0.7 = FSM 퍽 재탐색 기준
REF_FORMAT = "nz2_acc_ref_v2"        # make_accuracy_ref.py 가 쓰는 기준 형식 (원본 PyTorch 포즈 포함)
ORIG_KEYS = ("confidence", "center_px", "corners_px", "center_camera_m", "rotation_matrix",
             "valid", "translation_valid", "rotation_valid")
NA = "검증 불가"


def check_ref_format(ref, need_pose):
    """기준 파일 형식 확인: 형식 버전 + (CenterPose 비교면) 모든 pose 의 원본 필드. 문제 있으면 이유 목록"""
    errs = []
    if ref.get("format") != REF_FORMAT:
        errs.append(f"format={ref.get('format')!r} (필요 {REF_FORMAT!r}) → 새 make_accuracy_ref.py 로 다시 만들 것")
    for k in ("names", "conf", "iou", "yolo_input", "images"):
        if k not in ref:
            errs.append(f"필수 항목 없음: {k}")
    if need_pose and not errs:
        for k in ("cp_size", "crop_scales"):
            if k not in ref:
                errs.append(f"필수 항목 없음: {k}")
        for rec in ref["images"]:
            for i, ps in enumerate(rec.get("poses", [])):
                o = ps.get("orig")
                miss = [k for k in ("box", "cls") if k not in ps] + ([] if isinstance(o, dict) else ["orig"])
                if isinstance(o, dict):
                    miss += [f"orig.{k}" for k in ORIG_KEYS if k not in o]
                    for k, ok_k in (("translation_valid", "center_camera_m"), ("rotation_valid", "rotation_matrix")):
                        if o.get(k) and o.get(ok_k) is None:
                            miss.append(f"orig.{ok_k} (값 없음, {k}=True)")
                if miss:
                    errs.append(f"{rec.get('file')} pose {i}: {miss}")
                    if len(errs) > 5:
                        return errs
    return errs


def _q(v, p):
    return round(float(np.percentile(v, p)), 4) if len(v) else None


def acc_check(yolo, yout, cp, ref_dir, out_dir):
    """make_accuracy_ref.py 기준과 이 엔진 결과 비교. 같은 사진·같은 YOLO 전후처리
    YOLO: 원본 검출마다 같은 클래스·IoU 0.5 이상 엔진 검출을 짝지음 → 임계값 T 별 '유지'(원본 ≥T 인 물체를 엔진도 ≥T 로 냄),
          점수 하락, 클래스별 손실, 새로 T 를 넘은 검출(추가)
    CenterPose: 원본 박스를 그대로 넣음 (YOLO 차이와 분리). 기준 = 원본 PyTorch infer(scale3_median) 결과
          → 최종 3D 위치(mm)·회전(도, 대칭 고려)·유효 판정 일치, 꼭짓점 px"""
    ref = json.load(open(os.path.join(ref_dir, "ref.json")))
    names, conf, iou = ref["names"], ref["conf"], ref["iou"]
    scales = ref.get("crop_scales", CROP_SCALES)
    dims = tuple(ref.get("dims", PUCK_DIMS))
    _, _, YH, YW = yolo.in_shape
    if [YH, YW] != list(ref["yolo_input"]):
        print(f"주의: YOLO 엔진 입력 {YH}x{YW} ≠ 원본 {ref['yolo_input']} → 크기 변경 + FP16 영향을 함께 보는 비교")
    if cp is not None and (cp.in_shape[0] != len(scales) or cp.in_shape[2] != ref["cp_size"]):
        raise SystemExit(f"CenterPose 엔진 {cp.in_shape} ≠ 원본 batch {len(scales)}·{ref['cp_size']} → 같은 계약 엔진으로")
    errs = check_ref_format(ref, cp is not None)
    if errs:
        raise SystemExit("기준 파일 형식 문제:\n  " + "\n  ".join(errs))
    cps = cp.in_shape[2] if cp is not None else 0
    y_rows, extra, p_rows = [], [], []
    for rec in ref["images"]:
        img = cv2.imread(os.path.join(ref_dir, rec["file"]))
        if img is None:
            print("사진 못 읽음:", rec["file"])
            continue
        K = np.array(rec["K"], np.float64) if rec.get("K") else default_K(img.shape[1], img.shape[0])
        x, s, px, py = letterbox(img, YH, YW)
        dets = yolo_decode(yolo(x)[yout], conf, iou, s, px, py, img.shape[:2])
        used = set()
        for r in sorted(rec["dets"], key=lambda d: -d[4]):
            best, bi = 0.0, -1
            for j, d in enumerate(dets):
                if j in used or d[5] != r[5]:
                    continue
                v = box_iou(r[:4], d[:4])
                if v > best:
                    best, bi = v, j
            if best >= 0.5:
                used.add(bi)
                y_rows.append(dict(img=rec["file"], cls=r[5], ref=r[4], eng=dets[bi][4], iou=best))
            else:
                y_rows.append(dict(img=rec["file"], cls=r[5], ref=r[4], eng=None, iou=best))
        extra += [dict(img=rec["file"], cls=d[5], eng=d[4]) for j, d in enumerate(dets) if j not in used]
        if cp is None:
            continue
        for ps in rec["poses"]:
            o = ps["orig"]
            res, _ = cp_pose(cp, img, ps["box"], K, scales, cps, dims)
            row = dict(img=rec["file"], cls=ps["cls"], conf_ref=o["confidence"], conf_eng=res["confidence"],
                       valid_ref=o["valid"], valid_eng=res["valid"], tv_ref=o["translation_valid"],
                       tv_eng=res["translation_valid"], rv_ref=o["rotation_valid"], rv_eng=res["rotation_valid"],
                       corner_px=float(np.max(np.linalg.norm(res["corners_px"] - np.array(o["corners_px"]), axis=1))))
            if o["translation_valid"] and res["translation_valid"]:
                d = np.asarray(res["center_camera_m"]) - np.array(o["center_camera_m"])
                row.update(trans_mm=float(np.linalg.norm(d) * 1000), depth_mm=float(abs(d[2]) * 1000),
                           dist_m=float(np.linalg.norm(o["center_camera_m"])))
            if o["rotation_valid"] and res["rotation_valid"]:
                row.update(rot_deg=rot_err_deg(o["rotation_matrix"], res["rotation_matrix"]),
                           rot_sym_deg=rot_err_deg(o["rotation_matrix"], res["rotation_matrix"], sym=True))
            p_rows.append(row)

    # ---- YOLO 요약
    def keep(rows, T):
        base = [r for r in rows if r["ref"] >= T]
        k = sum(r["eng"] is not None and r["eng"] >= T for r in base)
        return round(k / max(len(base), 1), 4), len(base)

    ysum = dict(ref_dets=len(y_rows), matched=sum(r["eng"] is not None for r in y_rows))
    for T in THRESHOLDS:
        kv, n = keep(y_rows, T)
        kp, npk = keep([r for r in y_rows if r["cls"] in CP_CLASSES], T)
        new_over = sum(e["eng"] >= T for e in extra) + sum(r["eng"] is not None and r["eng"] >= T > r["ref"] for r in y_rows)
        ysum[f"T{T}"] = dict(keep=kv, n=n, keep_puck=kp, n_puck=npk, new_over=int(new_over))
    ysum["keep_by_class_T0.5"] = {names[c]: keep([r for r in y_rows if r["cls"] == c], 0.5) for c in sorted({r["cls"] for r in y_rows})}
    dd = [r["eng"] - r["ref"] for r in y_rows if r["eng"] is not None]
    ysum["score_diff"] = dict(p5=_q(dd, 5), p50=_q(dd, 50), min=round(min(dd), 4) if dd else None)
    ysum["drop_below_0.7"] = int(sum(r["ref"] >= 0.7 and (r["eng"] is None or r["eng"] < 0.7) for r in y_rows))
    t25, t5, t7 = ysum["T0.25"], ysum["T0.5"], ysum["T0.7"]
    if min(t25["n"], t5["n"], t7["n"], t5["n_puck"]) == 0:                       # 비교 대상 0 → 판정 못 함
        v_y = NA
    else:
        v_y = "OK" if (t25["keep"] >= 0.95 and t5["keep"] >= 0.98 and t7["keep"] >= 0.98 and t5["keep_puck"] >= 0.98
                       and t5["new_over"] <= 0.02 * t5["n"]) else "확인 필요"
    summ = dict(ref_dir=ref_dir, reference=ref.get("reference"), yolo_input=[YH, YW], ref_yolo_input=ref["yolo_input"],
                images=len(ref["images"]), yolo=ysum)

    # ---- CenterPose 요약
    v_p = None if cp is None else NA                                               # CenterPose 비교 안 했으면 None
    if p_rows:
        tm = [r["trans_mm"] for r in p_rows if "trans_mm" in r]
        rs_ = [r["rot_sym_deg"] for r in p_rows if "rot_sym_deg" in r]
        rr = [r["rot_deg"] for r in p_rows if "rot_deg" in r]
        cpx = [r["corner_px"] for r in p_rows]
        flips = dict(valid=sum(r["valid_ref"] != r["valid_eng"] for r in p_rows),
                     translation=sum(r["tv_ref"] != r["tv_eng"] for r in p_rows),
                     rotation=sum(r["rv_ref"] != r["rv_eng"] for r in p_rows))
        summ["centerpose"] = dict(
            pucks=len(p_rows), valid_ref=sum(r["valid_ref"] for r in p_rows), flips=flips,
            trans_mm=dict(n=len(tm), p50=_q(tm, 50), p95=_q(tm, 95), max=round(max(tm), 3) if tm else None),
            depth_mm_p95=_q([r["depth_mm"] for r in p_rows if "depth_mm" in r], 95),
            rot_sym_deg=dict(n=len(rs_), p50=_q(rs_, 50), p95=_q(rs_, 95), max=round(max(rs_), 3) if rs_ else None),
            rot_raw_deg_p95=_q(rr, 95), corner_px=dict(p50=_q(cpx, 50), p95=_q(cpx, 95), max=round(max(cpx), 3)),
            conf_diff_max=round(max(abs(r["conf_eng"] - r["conf_ref"]) for r in p_rows), 4))
        c = summ["centerpose"]
        n_p = len(p_rows)
        if not all(flips[k] <= 0.01 * n_p for k in ("valid", "translation", "rotation")):
            v_p = "확인 필요"                                                     # 유효 판정이 1% 넘게 바뀜 (표본 1개면 1개만 바뀌어도)
        elif not tm or not rs_:                                                    # 위치·회전 비교 표본 0 → 판정 못 함
            v_p = NA
        else:
            v_p = "OK" if (c["trans_mm"]["p95"] <= 5.0 and c["rot_sym_deg"]["p95"] <= 2.0) else "확인 필요"
    summ["verdict"] = dict(yolo=v_y, centerpose=v_p,
                           rule="[제안 기준, 팀 합의 필요] YOLO: 원본 ≥T 물체를 엔진도 ≥T 로 유지 — T0.25 ≥0.95, T0.5 ≥0.98, "
                                "T0.7 ≥0.98, 퍽 T0.5 ≥0.98, 새로 0.5 넘은 검출 ≤2% (어느 하나라도 비교 대상 0 이면 검증 불가) / "
                                "CenterPose: 유효·위치 유효·회전 유효 판정 바뀜 각각 ≤1%, 3D 위치 차이 95% ≤5 mm, "
                                "회전 차이(대칭 고려) 95% ≤2° (위치·회전 비교 표본 0 이면 검증 불가)")
    os.makedirs(out_dir, exist_ok=True)
    json.dump(to_jsonable(dict(summary=summ, yolo_rows=y_rows, extra=extra, pose_rows=p_rows)),
              open(os.path.join(out_dir, "acc_result.json"), "w"), indent=1, ensure_ascii=False)
    print("\n================ 정확도 (원본 FP32 대비) ================")
    print(f"YOLO {YH}x{YW} (원본 {ref['yolo_input']}): 원본 검출 {ysum['ref_dets']} → 짝 {ysum['matched']}")
    print("     유지율 " + " | ".join(f"≥{T}: {ysum[f'T{T}']['keep']} ({ysum[f'T{T}']['n']}개, 퍽 {ysum[f'T{T}']['keep_puck']}"
                                     f"/{ysum[f'T{T}']['n_puck']}, 새로 넘음 {ysum[f'T{T}']['new_over']})" for T in THRESHOLDS))
    print(f"     점수 변화 p5 {ysum['score_diff']['p5']} 최소 {ysum['score_diff']['min']}, 원본≥0.7→0.7 미만 {ysum['drop_below_0.7']}개"
          f" | 클래스별 유지(≥0.5) {ysum['keep_by_class_T0.5']} → {summ['verdict']['yolo']}")
    if not p_rows and cp is not None:
        print(f"CenterPose: 비교할 퍽 0개 → {NA}")
    if p_rows:
        c = summ["centerpose"]
        print(f"CenterPose 퍽 {c['pucks']} (원본 유효 {c['valid_ref']}): 유효 판정 바뀜 {c['flips']} | 3D 위치 차이 mm "
              f"중앙 {c['trans_mm']['p50']} 95% {c['trans_mm']['p95']} 최대 {c['trans_mm']['max']} (깊이 95% {c['depth_mm_p95']})")
        print(f"     회전 차이(대칭 고려) 도 중앙 {c['rot_sym_deg']['p50']} 95% {c['rot_sym_deg']['p95']} 최대 {c['rot_sym_deg']['max']}"
              f" (그대로 95% {c['rot_raw_deg_p95']}) | 꼭짓점 px 95% {c['corner_px']['p95']} 최대 {c['corner_px']['max']}"
              f" | 점수차 최대 {c['conf_diff_max']} → {summ['verdict']['centerpose']}")
    print(f"기준: {summ['verdict']['rule']}")
    print(f"저장: {os.path.join(out_dir, 'acc_result.json')}")


# ---------------------------------------------------------------- 카메라 처리 주기 재현 (tutor perception_schedule.py)
TOP_CAMS = ("r07_camera1", "r07_camera2", "r07_camera3")
# tutor origin/tutor 8b4468e 기본 프로필 r12-camera-calibration-v2-uniform640: 모든 카메라 640x480, K 400·400·319.5·239.5
# 11-06 이전 mixed 프로필(명시 선택 시만): 손목 1920x1440, K 1200·1200·959.5·719.5 → SIM_K_MIXED_WRIST
SIM_K = {"top": (400.0, 400.0, 319.5, 239.5), "wrist": (400.0, 400.0, 319.5, 239.5)}
SIM_K_MIXED_WRIST = (1200.0, 1200.0, 959.5, 719.5)


def select_perception_work(frame_index, wrist_ready=False):
    """tutor r12/src/arm_practice/perception_schedule.py 와 같은 규칙
    survey: 20틱마다 1번 감지, 카메라 r07_camera1 → 2 → 3 → arm_camera 순환 (카메라당 80틱마다)
    wrist_confirmation(손목 준비 완료): arm_camera 를 매 틱 감지"""
    if wrist_ready:
        return dict(camera="arm_camera", detect=True, mode="wrist_confirmation")
    return dict(camera=(TOP_CAMS + ("arm_camera",))[(frame_index // 20) % 4], detect=frame_index % 20 == 0, mode="survey")


def process_frame(img, K, yolo, yout, cp, a, YH, YW, cps):
    """한 장: letterbox → YOLO → NMS → 퍽마다 CenterPose(crop 3장·PnP·깊이) → 시간·개수"""
    tA = time.perf_counter()
    x, s, padx, pady = letterbox(img, YH, YW)
    yo = yolo(x)[yout]
    dets = yolo_decode(yo, a.conf, a.iou, s, padx, pady, img.shape[:2])
    tD = time.perf_counter()
    n_cp = n_valid = 0
    for x1, y1, x2, y2, sc, c in dets:
        if cp is None or c not in CP_CLASSES:
            continue
        res, _ = cp_pose(cp, img, (x1, y1, x2, y2), K, a.crop_scales, cps)
        n_cp += 1
        n_valid += bool(res["valid"])
    tE = time.perf_counter()
    return dict(n_det=len(dets), n_cp=n_cp, n_valid=n_valid, yolo_ms=(tD - tA) * 1000, cp_ms=(tE - tD) * 1000,
                total_ms=(tE - tA) * 1000)


def run_schedule(a, yolo, yout, cp, YH, YW, cps, mem, engines, load_mem):
    """틱(--tick_hz)마다 tutor 규칙으로 카메라를 고르고, 감지 틱에만 처리. 실제 시간에 맞춰 기다림 (GPU 쉬는 시간 포함)
    시간 기록 (모두 실제 경과 시간 기준):
      proc_ms  = 처리만 (letterbox·YOLO·NMS·CenterPose·PnP)
      start_ms = 틱 시각 → 처리 시작 (앞 요청이 밀려서 늦어진 만큼)
      e2e_ms   = 틱 시각 → 처리 끝. 마감 넘김 = e2e > 마감 (survey 20틱, wrist 1틱)
      버림     = 처리 시작 시점에 이미 마감이 지난 틱 (오래된 프레임)
      처리 Hz  = 처리한 요청 수 / 실제 경과 시간
    사진: 기본은 --preload 장을 미리 메모리에 올려 매 요청 디스크 읽기를 없앰 (실제 카메라는 디코딩된 프레임이 옴).
          보관 메모리는 preload_mb 로 따로 기록. --read_each 면 매 요청 디스크에서 읽고, 그 시간도 e2e 에 포함"""
    def files_of(d):
        f = sorted(glob.glob(os.path.join(d, "*.jpg")) + glob.glob(os.path.join(d, "*.png")))
        if not f:
            raise SystemExit(f"사진 없음: {d}")
        return f
    src = {"top": files_of(a.top_dir), "wrist": files_of(a.wrist_dir)}
    pool, preload_mb = {}, {}
    if not a.read_each:
        for kind in ("top", "wrist"):
            pool[kind] = [im for im in (cv2.imread(f) for f in src[kind][:a.preload]) if im is not None]
            preload_mb[kind] = round(sum(im.nbytes for im in pool[kind]) / 1e6, 1)
    Ks = {k: np.array([[v[0], 0, v[2]], [0, v[1], v[3]], [0, 0, 1]], np.float64)
          for k, v in (("top", a.K_top), ("wrist", a.K_wrist))}
    period = 1.0 / a.tick_hz
    out_dir = os.path.join(a.out, time.strftime("%Y%m%d-%H%M%S") + f"_{a.schedule}")
    os.makedirs(out_dir)
    cols = ["tick", "mode", "camera", "kind", "h", "w", "n_det", "n_cp", "n_valid", "read_ms", "start_ms", "yolo_ms",
            "cp_ms", "proc_ms", "e2e_ms", "deadline_ms", "overrun", "rss_mb", "sys_used_mb"]
    rows, cnt, dropped = [], {"top": 0, "wrist": 0}, {"top": 0, "wrist": 0}
    print(f"주기 재현 {a.schedule}: 틱 {a.tick_hz} Hz, {a.ticks}틱 | 상단 {a.top_dir} ({len(src['top'])}장, K {a.K_top}) | "
          f"손목 {a.wrist_dir} ({len(src['wrist'])}장, K {a.K_wrist}) | 사진 "
          + ("매번 디스크에서 읽음 (읽기 시간 e2e 포함)" if a.read_each else f"미리 올림 {preload_mb} MB"))

    def get(kind):
        if a.read_each:
            return cv2.imread(src[kind][cnt[kind] % len(src[kind])])
        return pool[kind][cnt[kind] % len(pool[kind])]
    for kind in ("top", "wrist"):                                   # 워밍업 (통계 제외)
        for _ in range(3):
            process_frame(cv2.imread(src[kind][0]), Ks[kind], yolo, yout, cp, a, YH, YW, cps)
    mem.phase = "run"
    t0 = time.perf_counter() + 0.5
    with open(os.path.join(out_dir, "schedule.csv"), "w", newline="") as fcsv:
        wr = csv.DictWriter(fcsv, fieldnames=cols)
        wr.writeheader()
        for i in range(a.ticks):
            tick_t = t0 + i * period
            now = time.perf_counter()
            if now < tick_t:
                time.sleep(tick_t - now)
            work = select_perception_work(i, wrist_ready=(a.schedule == "wrist"))
            if not work["detect"]:
                continue
            kind = "wrist" if work["camera"] == "arm_camera" else "top"
            deadline = period * (1 if work["mode"] == "wrist_confirmation" else 20)
            tR = time.perf_counter()
            if tR - tick_t > deadline:                                  # 이미 마감 지난 틱 = 실제로는 버려지는 오래된 프레임
                dropped[kind] += 1
                continue
            img = get(kind)
            cnt[kind] += 1
            tS = time.perf_counter()
            r = process_frame(img, Ks[kind], yolo, yout, cp, a, YH, YW, cps)
            tE = time.perf_counter()
            row = dict(tick=i, mode=work["mode"], camera=work["camera"], kind=kind, h=img.shape[0], w=img.shape[1],
                       n_det=r["n_det"], n_cp=r["n_cp"], n_valid=r["n_valid"], read_ms=round((tS - tR) * 1000, 2),
                       start_ms=round((tR - tick_t) * 1000, 2), yolo_ms=round(r["yolo_ms"], 2), cp_ms=round(r["cp_ms"], 2),
                       proc_ms=round(r["total_ms"], 2), e2e_ms=round((tE - tick_t) * 1000, 2),
                       deadline_ms=round(deadline * 1000, 1), overrun=bool(tE - tick_t > deadline),
                       rss_mb=round(rss_mb(), 1), sys_used_mb=round(BASE_AVAIL - mem_available_mb(), 1))
            wr.writerow(row)
            rows.append(row)
    t_end = max(time.perf_counter(), t0 + a.ticks * period)          # 실제 경과 (예정보다 길어졌으면 그만큼)
    elapsed = t_end - t0
    mem.stop()
    if not rows:
        print("처리한 요청 0 → --ticks 를 늘릴 것")
        return

    def pct(v, q):
        return round(float(np.percentile(v, q)), 2) if v else None
    by = {}
    for kind in ("top", "wrist"):
        g = [r for r in rows if r["kind"] == kind]
        if g or dropped[kind]:
            asked = len(g) + dropped[kind]
            by[kind] = dict(requests=asked, processed=len(g), dropped=dropped[kind],
                            overruns=sum(r["overrun"] for r in g), deadline_ms=g[0]["deadline_ms"] if g else None,
                            proc_p50=pct([r["proc_ms"] for r in g], 50), proc_p99=pct([r["proc_ms"] for r in g], 99),
                            e2e_p50=pct([r["e2e_ms"] for r in g], 50), e2e_p99=pct([r["e2e_ms"] for r in g], 99),
                            e2e_max=max([r["e2e_ms"] for r in g], default=None),
                            start_p99=pct([r["start_ms"] for r in g], 99), read_p50=pct([r["read_ms"] for r in g], 50),
                            asked_hz=round(asked / elapsed, 2), done_hz=round(len(g) / elapsed, 2),
                            on_time_ratio=round(sum(not r["overrun"] for r in g) / max(asked, 1), 4),
                            max_pucks=max([r["n_cp"] for r in g], default=0),
                            frame=f"{g[0]['w']}x{g[0]['h']}" if g else None)
    busy = sum(r["e2e_ms"] - r["start_ms"] for r in rows) / (elapsed * 1000)   # 읽기(--read_each)+처리 시간 / 실제 경과
    summ = dict(schedule=a.schedule, tick_hz=a.tick_hz, ticks=a.ticks, elapsed_s=round(elapsed, 2),
                planned_s=round(a.ticks * period, 2), read_each=a.read_each, preload_mb=preload_mb,
                by_camera_kind=by, busy_ratio=round(busy, 3), yolo=a.yolo, cp=a.cp, engines=engines, load_mem=load_mem,
                peak_sys_used_mb=round(mem.peak["sys"], 1), peak_rss_mb=round(mem.peak["rss"], 1),
                peak_by_phase={k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in mem.by_phase.items()},
                K_top=a.K_top, K_wrist=a.K_wrist,
                note="tutor perception_schedule 재현. 마감 판정·처리 Hz 는 실제 경과 시간 기준 (e2e = 틱 → 처리 끝). "
                     "on_time_ratio = 마감 안에 끝난 요청 / (처리 + 버림). 메모리 최고값에는 미리 올린 사진(preload_mb)이 포함됨")
    json.dump(to_jsonable(summ), open(os.path.join(out_dir, "summary.json"), "w"), indent=1, ensure_ascii=False)
    print(f"\n================ 주기 재현 요약 ({a.schedule}, {a.tick_hz} Hz, 실제 {summ['elapsed_s']} s / 예정 {summ['planned_s']} s) ================")
    for k, v in by.items():
        print(f"{'상단' if k == 'top' else '손목'} {v['frame']}: 요청 {v['requests']} → 처리 {v['processed']}, 버림 {v['dropped']}, "
              f"마감 넘김 {v['overruns']} (마감 {v['deadline_ms']} ms) | 제때 끝난 비율 {v['on_time_ratio']} | "
              f"요청 {v['asked_hz']} Hz → 처리 {v['done_hz']} Hz")
        print(f"     처리만 p50 {v['proc_p50']} p99 {v['proc_p99']} ms | 틱→끝 p50 {v['e2e_p50']} p99 {v['e2e_p99']} 최대 {v['e2e_max']} ms"
              f" | 늦은 시작 p99 {v['start_p99']} ms | 읽기 p50 {v['read_p50']} ms | 퍽 최대 {v['max_pucks']}")
    print(f"바쁜 비율 {summ['busy_ratio']} (1 미만이어야 따라감) | 메모리 최대 시스템 +{summ['peak_sys_used_mb']} MB, "
          f"RSS {summ['peak_rss_mb']} MB (미리 올린 사진 {preload_mb} MB 포함) → 1 GB "
          f"{'이하' if summ['peak_sys_used_mb'] <= 1024 else '초과'}")
    print(f"저장: {out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yolo", default="optimize/engines/yolo_800x800_base_jetson.engine")       # ③ 통과 크기 800, 모드는 잠정 base
    ap.add_argument("--cp", default="optimize/engines/centerpose_128x128_b3_h4_base_jetson.engine")  # 잠정 base (uniform640 판정에선 lowmem 도 정확도 통과, 최적화.md 9-8)
    ap.add_argument("--no_cp", action="store_true", help="YOLO 만")
    ap.add_argument("--source", default="realsense", help="realsense / 사진 / 폴더")
    ap.add_argument("--check_ref", default="", help="CenterPose ref 폴더 (input.npy, reference_*.npy) → 정합 검사만 하고 끝")
    ap.add_argument("--acc_ref", default="", help="make_accuracy_ref.py 결과 폴더 → 원본 대비 정확도 비교만 하고 끝")
    ap.add_argument("--names", nargs="+", default=YOLO_NAMES)
    ap.add_argument("--conf", type=float, default=0.25, help="tutor detector_confidence")
    ap.add_argument("--iou", type=float, default=0.7)
    ap.add_argument("--crop_scales", type=float, nargs="+", default=CROP_SCALES,
                    help="crop 배율 (개수 = CenterPose 엔진 batch). 원본 scale3_median = 1.5 1.35 1.65")
    ap.add_argument("--K", type=float, nargs=4, default=None, metavar=("FX", "FY", "CX", "CY"),
                    help="사진 입력일 때 카메라 내부 파라미터 (realsense 는 스트림에서 읽음)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=0, help="0 = q/Esc/Ctrl+C 까지")
    ap.add_argument("--warm", type=int, default=10, help="통계에서 뺄 처음 프레임 수 (메모리 최고값은 포함해서 따로 기록)")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--save_every", type=int, default=0, help="N 프레임마다 결과 사진 저장 (0 = 안 함)")
    ap.add_argument("--log_every", type=int, default=30)
    ap.add_argument("--out", default="outputs/pipeline")
    ap.add_argument("--schedule", choices=["survey", "wrist"], default=None,
                    help="tutor 카메라 처리 주기 재현 (사진 폴더): survey = 20틱마다 상단1→2→3→손목 순환, wrist = 손목 매 틱")
    ap.add_argument("--tick_hz", type=float, default=20.0, help="틱 주기 (tutor synthetic_packet_20hz = 20, 실제 카메라 30)")
    ap.add_argument("--ticks", type=int, default=1200, help="재현할 틱 수 (20 Hz 면 1200 = 1분)")
    ap.add_argument("--top_dir", default="optimize/acc_ref_43real/images", help="상단 카메라(640x480) 사진 폴더")
    ap.add_argument("--wrist_dir", default="optimize/acc_ref_43real/images",
                    help="손목 카메라 사진 폴더 (uniform640 기본 = 640x480. mixed 프로필 시험은 acc_ref_1440real/images + --K_wrist 1200 1200 959.5 719.5)")
    ap.add_argument("--K_top", type=float, nargs=4, default=list(SIM_K["top"]), metavar=("FX", "FY", "CX", "CY"))
    ap.add_argument("--K_wrist", type=float, nargs=4, default=list(SIM_K["wrist"]), metavar=("FX", "FY", "CX", "CY"))
    ap.add_argument("--preload", type=int, default=8, help="주기 재현: 카메라 종류별로 미리 메모리에 올릴 사진 수")
    ap.add_argument("--read_each", action="store_true", help="주기 재현: 매 요청 디스크에서 읽기 (읽기 시간이 e2e 에 포함)")
    a = ap.parse_args()

    mem = MemSampler()
    mem.start()
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    cuda = Cuda()
    t0 = time.perf_counter()
    mem.phase = "load"
    cp = None if a.no_cp and not a.check_ref else TrtModel(trt, logger, cuda, a.cp)
    if a.check_ref:
        check_ref(cp, a.check_ref)
        return
    yolo = TrtModel(trt, logger, cuda, a.yolo)
    _, _, YH, YW = yolo.in_shape
    yout = yolo.outputs[0]
    nc = yolo.host[yout].shape[1] - 4
    if nc != len(a.names):
        print(f"주의: 엔진 클래스 수 {nc} ≠ --names {len(a.names)}개 → 번호로 표시")
        a.names = [str(i) for i in range(nc)]
    if cp is not None:
        miss = CP_NEED - set(cp.outputs)
        if miss:
            raise SystemExit(f"CenterPose 엔진 출력 {cp.outputs} 에 {miss} 없음 (scale 포함 4개 출력 엔진 필요)")
    if a.acc_ref:
        acc_check(yolo, yout, cp, a.acc_ref, os.path.join(a.out, "acc_" + time.strftime("%Y%m%d-%H%M%S")))
        return
    if cp is not None and cp.in_shape[0] != len(a.crop_scales):
        raise SystemExit(f"CenterPose 엔진 batch {cp.in_shape[0]} ≠ --crop_scales {a.crop_scales} 개수")
    cps = cp.in_shape[2] if cp else 0
    load_mem = dict(rss_mb=round(rss_mb(), 1), sys_used_mb=round(BASE_AVAIL - mem_available_mb(), 1))
    engines = {k: dict(path=m.path, file_mb=m.file_mb, device_mem_mb=m.device_mem_mb)
               for k, m in (("yolo", yolo), ("cp", cp)) if m is not None}
    print("엔진 크기·GPU 작업 메모리: " + ", ".join(f"{k} {v['file_mb']} MB / {v['device_mem_mb']} MB" for k, v in engines.items()))
    print(f"TensorRT {trt.__version__} | YOLO {a.yolo} 입력 {yolo.in_shape} 클래스 {nc} | "
          f"CenterPose {a.cp if cp else '끔'} {cp.in_shape if cp else ''} | 로드 {time.perf_counter() - t0:.1f} s")
    print(f"엔진 로드 직후 메모리: RSS {load_mem['rss_mb']} MB, 시스템 +{load_mem['sys_used_mb']} MB")
    if a.schedule:
        mem.phase = "warm"
        run_schedule(a, yolo, yout, cp, YH, YW, cps, mem, engines, load_mem)
        return

    # 입력 소스
    pipe, files, rs, K_cam = None, [], None, None
    if a.source == "realsense":
        import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
        prof = pipe.start(cfg)
        for sensor in prof.get_device().query_sensors():
            if sensor.supports(rs.option.frames_queue_size):
                sensor.set_option(rs.option.frames_queue_size, 1)
        it = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K_cam = np.array([[it.fx, 0, it.ppx], [0, it.fy, it.ppy], [0, 0, 1]], np.float64)
        print(f"RealSense {a.width}x{a.height}@{a.fps}, K fx {it.fx:.1f} fy {it.fy:.1f} cx {it.ppx:.1f} cy {it.ppy:.1f}")
    else:
        files = sorted(glob.glob(os.path.join(a.source, "*.jpg")) + glob.glob(os.path.join(a.source, "*.png"))) \
            if os.path.isdir(a.source) else [a.source]
        files = [f for f in files if os.path.isfile(f)]
        if not files:
            raise SystemExit(f"사진 없음: {a.source}")
        if not a.frames and a.headless:
            a.frames = len(files) if len(files) > 1 else 200
        if a.K:
            K_cam = np.array([[a.K[0], 0, a.K[2]], [0, a.K[1], a.K[3]], [0, 0, 1]], np.float64)

    def next_frame(i):
        """(영상, 카메라 시각 대비 나이 ms 또는 None)"""
        if pipe is not None:
            f = pipe.wait_for_frames(timeout_ms=5000).get_color_frame()
            if not f:
                return None, None
            age = None
            if f.get_frame_timestamp_domain() in (rs.timestamp_domain.system_time, rs.timestamp_domain.global_time):
                age = time.time() * 1000.0 - f.get_timestamp()
            return np.asanyarray(f.get_data()), age
        # 매번 디스크에서 읽음 (캐시하면 사진 수 x 프레임 크기만큼 메모리가 늘어 카메라와 다른 조건이 됨.
        #  1920x1440 50장이면 ~400 MB). 읽는 시간은 wait_ms 로 빠지고 total_ms 에는 안 들어감. 사진 입력의 loop_fps 는 참고용
        return cv2.imread(files[i % len(files)]), None

    out_dir = os.path.join(a.out, time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir)
    title = "YOLO + CenterPose (TensorRT) | q/Esc: quit | s: save"
    cols = ["frame", "n_det", "n_cp", "n_valid", "wait_ms", "age_ms", "pre_ms", "yolo_ms", "post_ms", "cp_ms",
            "total_ms", "loop_fps", "rss_mb", "sys_used_mb"]
    rows = []
    n, start = 0, None
    mem.phase = "warm"
    try:
        with open(os.path.join(out_dir, "timings.csv"), "w", newline="") as fcsv:
            wr = csv.DictWriter(fcsv, fieldnames=cols)
            wr.writeheader()
            while True:
                tW = time.perf_counter()
                img, age = next_frame(n)
                if img is None:
                    continue
                tA = time.perf_counter()
                K = K_cam if K_cam is not None else default_K(img.shape[1], img.shape[0])
                x, s, padx, pady = letterbox(img, YH, YW)
                tB = time.perf_counter()
                yo = yolo(x)[yout]
                tC = time.perf_counter()
                dets = yolo_decode(yo, a.conf, a.iou, s, padx, pady, img.shape[:2])
                tD = time.perf_counter()
                canvas = img if a.headless and not a.save_every else img.copy()
                n_cp, n_valid, cp_ms = 0, 0, 0.0
                for x1, y1, x2, y2, sc, c in dets:
                    name = a.names[c]
                    if not a.headless or a.save_every:
                        cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)),
                                      (80, 80, 255) if "red" in name else (80, 200, 80), 1)
                    if cp is None or c not in CP_CLASSES:
                        continue
                    tc = time.perf_counter()
                    res, _ = cp_pose(cp, img, (x1, y1, x2, y2), K, a.crop_scales, cps)   # crop 3장 + 엔진 1회 + PnP·깊이 + 중앙값
                    cp_ms += (time.perf_counter() - tc) * 1000
                    n_cp += 1
                    n_valid += bool(res["valid"])
                    if not a.headless or a.save_every:
                        draw_cuboid(canvas, res["corners_px"], res["center_px"], res["valid"])
                tE = time.perf_counter()
                n += 1
                if n == a.warm:
                    start = time.perf_counter()
                    mem.phase = "run"
                r_mb, s_mb = rss_mb(), BASE_AVAIL - mem_available_mb()
                if n > a.warm:
                    fps = (n - a.warm) / max(time.perf_counter() - start, 1e-6)
                    row = dict(frame=n - a.warm, n_det=len(dets), n_cp=n_cp, n_valid=n_valid,
                               wait_ms=round((tA - tW) * 1000, 3), age_ms=None if age is None else round(age, 2),
                               pre_ms=round((tB - tA) * 1000, 3), yolo_ms=round((tC - tB) * 1000, 3),
                               post_ms=round((tD - tC) * 1000, 3), cp_ms=round(cp_ms, 3),
                               total_ms=round((tE - tA) * 1000, 3), loop_fps=round(fps, 2),
                               rss_mb=round(r_mb, 1), sys_used_mb=round(s_mb, 1))
                    wr.writerow(row)
                    rows.append(row)
                    if row["frame"] % a.log_every == 0 or row["frame"] == 1:
                        print(f"frame {row['frame']:5d} | 검출 {len(dets):2d} 퍽 {n_cp:2d} (유효 {n_valid}) | YOLO {row['yolo_ms']:.1f} ms "
                              f"CP {cp_ms:.1f} ms 전체 {row['total_ms']:.1f} ms | {fps:.1f} FPS | "
                              f"RSS {r_mb:.0f} MB 시스템 +{s_mb:.0f} MB", flush=True)
                        fcsv.flush()
                    if a.save_every and row["frame"] % a.save_every == 0:
                        cv2.imwrite(os.path.join(out_dir, f"frame_{row['frame']:06d}.jpg"), canvas)
                if not a.headless:
                    hud = (f"YOLO {(tC - tB) * 1000:.1f} ms | CP {n_cp}x {cp_ms:.1f} ms | total {(tE - tA) * 1000:.1f} ms"
                           f" | RSS {r_mb:.0f} MB sys +{s_mb:.0f} MB")
                    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 24), (0, 0, 0), -1)
                    cv2.putText(canvas, hud, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow(title, canvas)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break
                    if key == ord("s"):
                        cv2.imwrite(os.path.join(out_dir, f"snap_{n:06d}.jpg"), canvas)
                if a.frames and n - a.warm >= a.frames:
                    break
    except KeyboardInterrupt:
        print("\n중지")
    finally:
        if pipe is not None:
            pipe.stop()
        if not a.headless:
            cv2.destroyAllWindows()
        mem.stop()

    if rows:
        def pct(rs_, k, q):
            v = [r[k] for r in rs_ if r[k] is not None]
            return round(float(np.percentile(v, q)), 2) if v else None
        k10 = max(1, len(rows) // 10)                                    # 메모리 추이: 처음 10% vs 마지막 10% 프레임
        trend = dict(first10_sys_mb=round(float(np.median([r["sys_used_mb"] for r in rows[:k10]])), 1),
                     last10_sys_mb=round(float(np.median([r["sys_used_mb"] for r in rows[-k10:]])), 1),
                     first10_rss_mb=round(float(np.median([r["rss_mb"] for r in rows[:k10]])), 1),
                     last10_rss_mb=round(float(np.median([r["rss_mb"] for r in rows[-k10:]])), 1))
        trend["sys_growth_mb"] = round(trend["last10_sys_mb"] - trend["first10_sys_mb"], 1)
        trend["rss_growth_mb"] = round(trend["last10_rss_mb"] - trend["first10_rss_mb"], 1)
        frame_hw = list(img.shape[:2]) if img is not None else None
        by_n = {}
        for k in sorted({r["n_cp"] for r in rows}):
            g = [r for r in rows if r["n_cp"] == k]
            by_n[str(k)] = dict(frames=len(g), total_p50=pct(g, "total_ms", 50), total_p99=pct(g, "total_ms", 99),
                                cp_p99=pct(g, "cp_ms", 99))
        summ = dict(frames=len(rows), headless=a.headless, source=a.source, yolo=a.yolo, cp=None if cp is None else a.cp,
                    engines=engines, frame_hw=frame_hw, mem_trend=trend,
                    yolo_input=list(yolo.in_shape), cp_input=None if cp is None else list(cp.in_shape),
                    K=None if K_cam is None else K_cam.tolist(), load_mem=load_mem,
                    peak_sys_used_mb=round(mem.peak["sys"], 1), peak_rss_mb=round(mem.peak["rss"], 1),
                    peak_by_phase={k: {kk: round(vv, 1) for kk, vv in v.items()} for k, v in mem.by_phase.items()},
                    yolo_ms_p50=pct(rows, "yolo_ms", 50), yolo_ms_p99=pct(rows, "yolo_ms", 99),
                    cp_ms_p50=pct(rows, "cp_ms", 50), cp_ms_p99=pct(rows, "cp_ms", 99),
                    total_ms_p50=pct(rows, "total_ms", 50), total_ms_p99=pct(rows, "total_ms", 99),
                    wait_ms_p50=pct(rows, "wait_ms", 50), age_ms_p50=pct(rows, "age_ms", 50), age_ms_p99=pct(rows, "age_ms", 99),
                    mean_det=round(float(np.mean([r["n_det"] for r in rows])), 2),
                    mean_pucks=round(float(np.mean([r["n_cp"] for r in rows])), 2), max_pucks=max(r["n_cp"] for r in rows),
                    by_puck_count=by_n, loop_fps=rows[-1]["loop_fps"],
                    note="peak_* = 50 Hz 스레드, 엔진 로드·워밍업 포함 전체 최고값 (peak_by_phase 에 구간별). "
                         "sys_used = 시작 전 대비 MemAvailable 감소 (다른 프로세스 영향 포함). 1 GB 판정은 --headless 값으로. "
                         "total = 처리 시작 → YOLO·NMS·CenterPose·PnP 끝, age = 카메라 시각 → 처리 시작")
        json.dump(to_jsonable(summ), open(os.path.join(out_dir, "summary.json"), "w"), indent=1, ensure_ascii=False)
        print("\n================ 요약 ================")
        print(f"프레임 {summ['frames']} | 평균 검출 {summ['mean_det']} | 퍽 평균 {summ['mean_pucks']} 최대 {summ['max_pucks']} | "
              f"{summ['loop_fps']} FPS")
        print(f"YOLO   p50 {summ['yolo_ms_p50']} ms  p99 {summ['yolo_ms_p99']} ms")
        print(f"CP합   p50 {summ['cp_ms_p50']} ms  p99 {summ['cp_ms_p99']} ms (프레임당 모든 퍽: crop·엔진·PnP·깊이)")
        print(f"전체   p50 {summ['total_ms_p50']} ms  p99 {summ['total_ms_p99']} ms (화면 표시 제외) | 카메라 대기 p50 "
              f"{summ['wait_ms_p50']} ms | 카메라 시각→처리 시작 p50 {summ['age_ms_p50']} p99 {summ['age_ms_p99']} ms")
        print("퍽 개수별 전체 p99: " + ", ".join(f"{k}개 {v['total_p99']} ms ({v['frames']}프레임)" for k, v in by_n.items()))
        print(f"영상 {frame_hw[1] if frame_hw else '?'}x{frame_hw[0] if frame_hw else '?'} | 메모리 추이 (처음 10% → 마지막 10%): "
              f"시스템+ {trend['first10_sys_mb']} → {trend['last10_sys_mb']} MB ({trend['sys_growth_mb']:+}), "
              f"RSS {trend['first10_rss_mb']} → {trend['last10_rss_mb']} MB ({trend['rss_growth_mb']:+})"
              f"{' ← 계속 늘면 누수 의심 (긴 시간 측정으로 확인)' if trend['rss_growth_mb'] > 20 else ''}")
        print("엔진 (파일 / GPU 작업 메모리): " + ", ".join(f"{k} {v['file_mb']} / {v['device_mem_mb']} MB" for k, v in engines.items()))
        print(f"메모리 최대 (로드·워밍업 포함): 시스템 +{summ['peak_sys_used_mb']} MB, RSS {summ['peak_rss_mb']} MB "
              f"→ 1 GB {'이하' if summ['peak_sys_used_mb'] <= 1024 else '초과'}{'' if a.headless else ' (화면 표시 포함)'} "
              f"| 구간별 {summ['peak_by_phase']}")
        print(f"저장: {out_dir}")


if __name__ == "__main__":
    main()
