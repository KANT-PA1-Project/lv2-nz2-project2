"""Camera-only OFF-lens acquisition, separately calibrated from autolabelling.

Candidate bands for the supplied exposure150/WB4600 screenshot; not a general
real-world accuracy claim. The lamp ON/OFF classifier is not changed here.
OpenCV hue units: 0..179. Keep red/yellow disjoint to avoid merged lens bands.
"""
from collections import OrderedDict

import cv2
import numpy as np

BANDS = dict(red_max=6, red_wrap=165, yellow_min=7, yellow_max=38,
             green_min=39, green_max=110, min_s=60, min_v=25)


def masks(hsv):
    h, s, v = cv2.split(hsv)
    p = BANDS
    valid = (s > p["min_s"]) & (v > p["min_v"])
    return {
        "R": valid & ((h <= p["red_max"]) | (h >= p["red_wrap"])),
        "Y": valid & (h >= p["yellow_min"]) & (h <= p["yellow_max"]),
        "G": valid & (h >= p["green_min"]) & (h <= p["green_max"]),
    }


def _blobs(mask, min_area):
    mask = cv2.morphologyEx(mask.astype(np.uint8)*255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(map(int, stats[i, :4])) for i in range(1, count) if stats[i, 4] >= min_area]


# 위에서 비스듬히 내려다보면(상단 카메라 60°) 이웃 칸이 화면에서 겹친다(B07·B09: 겹침 15 px).
# 칸 높이의 70% 까지 겹침 허용, 대신 위 칸 중심이 아래 칸 중심보다 위에 있어야 한다.
OVERLAP_RATIO = .7
_TIERS_MAX = 64
_tiers = OrderedDict()   # detect() 가 고른 박스 → 실제 R·Y·G 칸 영역 (x, y, w, h)


def _stacked(top, bottom):
    tx, ty, tw, th = top
    bx, by, bw, bh = bottom
    gap = by-(ty+th)
    return (abs(tx+tw/2-bx-bw/2) <= .6*max(tw, bw)
            and .5 < tw/max(bw, 1) < 2.
            and ty+th/2 < by+bh/2
            and -OVERLAP_RATIO*min(th, bh) <= gap <= .6*bh+3)


def observed_tiers(box):
    """detect() 가 이 박스를 고를 때 쓴 R·Y·G 칸 영역(사본). 없으면 None."""
    if box is None:
        return None
    tiers = _tiers.get(tuple(int(v) for v in box))
    return None if tiers is None else dict(tiers)


def clear_tiers():
    """시점이 바뀌면(차체·카메라 이동) 이전 칸 정보를 버린다."""
    _tiers.clear()


def detect(bgr):
    evidence = masks(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    minimum = max(8, bgr.shape[0]*bgr.shape[1]//40000)
    red, yellow, green = [_blobs(evidence[c], minimum) for c in "RYG"]
    candidates = []
    for r in red:
        for y in yellow:
            if _stacked(r, y):
                for g in green:
                    if _stacked(y, g):
                        x0 = min(r[0], y[0], g[0])
                        x1 = max(r[0]+r[2], y[0]+y[2], g[0]+g[2])
                        candidates.append(((x0, r[1], x1, g[1]+g[3]), dict(R=r, Y=y, G=g)))
    if not candidates:
        return None
    box, tiers = max(candidates, key=lambda c: (c[0][2]-c[0][0])*(c[0][3]-c[0][1]))
    key = tuple(int(v) for v in box)
    _tiers[key] = tiers
    _tiers.move_to_end(key)
    while len(_tiers) > _TIERS_MAX:
        _tiers.popitem(last=False)
    return box
