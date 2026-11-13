"""바코드가 확정되지 않을 때 팔 자세를 보정해 다시 보기 (원본 해독 코드는 수정하지 않음).

1. live_mission_optimized.barcode_regions(원본 함수)로 화면의 라벨 후보 위치를 찾는다.
2. 후보가 있으면: 후보 중심 픽셀 → 카메라 광선, 라벨 폭 px 로 거리 추정 → 월드 목표점.
   그 점이 화면 중앙보다 aim_up_deg 위(집게 가림 회피)에, 원하는 거리·정면에 오도록
   현재 관절각 근처에서 FK 최적화로 새 팔 자세를 계산한다.
3. 후보가 없으면: 기준 자세 주변을 정해진 순서로 훑는다(손목 피치·팔 yaw).
최대 max_attempts 회. 보정량은 관절당 max_step_rad 이내, 실측 관절 한계 안쪽 0.05 rad.
"""
import math

import numpy as np
from scipy.optimize import minimize

BARS_WIDTH_M = 0.060  # barcode_support.BARS_M[0]: 후보 영역은 막대 줄무늬 기준


class BarcodeSeek:
    def __init__(self, model, cfg, lower, upper, fx, width, height):
        seek = cfg.get('barcode_seek') or {}
        self.model, self.cfg = model, cfg
        self.lower, self.upper = np.asarray(lower) + .05, np.asarray(upper) - .05
        self.fx, self.cx, self.cy = fx, (width - 1) / 2, (height - 1) / 2
        self.width, self.height = width, height
        self.wait_s = float(seek.get('wait_s', 1.5))
        self.max_attempts = int(seek.get('max_attempts', 6))
        self.max_step = float(seek.get('max_step_rad', 0.5))
        self.aim_up = math.radians(float(seek.get('aim_up_deg', 10.)))
        self.distance = float(seek.get('distance_m', 0.25))
        self.normal = np.asarray(seek.get('label_normal', [1., 0., 0.]), float)
        self.scan = [np.asarray(d, float) for d in seek.get('scan_offsets_rad', [
            [0, 0, 0, .15, 0], [0, 0, 0, -.15, 0], [.2, 0, 0, 0, 0], [-.2, 0, 0, 0, 0],
            [0, 0, 0, .3, 0], [0, 0, 0, -.3, 0]])]
        self.attempts, self.scan_index = 0, 0
        self.log = []

    def due(self, t, since):
        return self.attempts < self.max_attempts and t - since >= self.wait_s

    def _candidate(self, bgr):
        from live_mission_optimized import barcode_regions
        regions = barcode_regions(bgr)
        if not regions:
            return None
        _, _, quad = regions[0]                       # 가장 강한 후보
        quad = np.asarray(quad, float)
        center = quad.mean(axis=0)
        edges = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
        # barcode_regions 는 줄무늬 폭 w 에 좌우 max(10, 0.18w) 여유를 더한 사각형 → 줄무늬 폭으로 되돌림
        long_edge = float(edges.max())
        stripe = long_edge / 1.36 if long_edge > 1.36 * 55.6 else long_edge - 20.
        return center, max(stripe, 10.)               # 중심 px, 줄무늬 폭 px

    def _cost(self, q, q0, target):
        cam = self.model.camera_world(q, self.base)
        p, y, z = cam[:3, 3], cam[:3, 1], cam[:3, 2]
        v = target - p
        d = float(np.linalg.norm(v))
        aim = z * math.cos(self.aim_up) - y * math.sin(self.aim_up)
        c = 20 * (1 - float(aim @ v) / d)
        c += 30 * (d - self.distance) ** 2
        c += 5 * cam[2, 0] ** 2                       # 화면 수평
        incidence = math.acos(np.clip(-(z @ self.normal), -1, 1))
        c += 2 * max(0., incidence - math.radians(35)) ** 2
        c += .2 * float(np.sum((q - q0) ** 2))        # 현재 자세 근처 유지
        return c

    def propose(self, bgr, q, base, t):
        """새 팔 목표(5,) 또는 None(시도 소진)."""
        if self.attempts >= self.max_attempts:
            return None
        self.attempts += 1
        self.base = base
        q = np.asarray(q, float)
        found = self._candidate(bgr)
        if found is not None:
            (u, v), width_px = found
            cam = self.model.camera_world(q, base)
            ray = cam[:3, :3] @ np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fx, 1.])
            ray /= np.linalg.norm(ray)
            depth = self.fx * BARS_WIDTH_M / width_px
            target = cam[:3, 3] + ray * depth
            lo = np.maximum(self.lower, q - self.max_step)
            hi = np.minimum(self.upper, q + self.max_step)
            r = minimize(self._cost, q, args=(q, target), method='L-BFGS-B', bounds=list(zip(lo, hi)))
            new = np.clip(r.x, lo, hi)
            self.log.append(dict(t_s=t, mode='candidate', pixel=[float(u), float(v)], width_px=width_px,
                                 est_depth_m=depth, target_world=target.tolist(), q_from=q.tolist(),
                                 q_to=new.tolist(), cost=float(r.fun)))
            return new
        if self.scan_index >= len(self.scan):
            self.log.append(dict(t_s=t, mode='scan_exhausted'))
            return None
        nominal = np.asarray(self.cfg['barcode_arm_q_rad'], float)
        new = np.clip(nominal + self.scan[self.scan_index], self.lower, self.upper)
        self.log.append(dict(t_s=t, mode='scan', offset=self.scan[self.scan_index].tolist(),
                             q_from=q.tolist(), q_to=new.tolist()))
        self.scan_index += 1
        return new
