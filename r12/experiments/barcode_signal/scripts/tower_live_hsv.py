"""Camera-only OFF-lens acquisition, separately calibrated from autolabelling.

Candidate bands for the supplied exposure150/WB4600 screenshot; not a general
real-world accuracy claim. The lamp ON/OFF classifier is not changed here.
OpenCV hue units: 0..179. Keep red/yellow disjoint to avoid merged lens bands.
"""
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


def _stacked(top, bottom):
    tx, ty, tw, th = top
    bx, by, bw, bh = bottom
    gap = by-(ty+th)
    return (abs(tx+tw/2-bx-bw/2) <= .6*max(tw, bw)
            and .5 < tw/max(bw, 1) < 2.
            and -.4*bh <= gap <= .6*bh+3)


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
                        candidates.append((x0, r[1], x1, g[1]+g[3]))
    return max(candidates, key=lambda b:(b[2]-b[0])*(b[3]-b[1]), default=None)
