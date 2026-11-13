"""비스듬히 내려다보는 신호등 구도용 판정 개선 후보 (원본 파일은 수정하지 않음).

install() 을 부르면 실행 중에만 다음 두 함수를 바꿔 끼운다.
- tower_live_hsv.detect: R·Y·G 세로 배열 검사에서 칸 겹침 허용 40% → OVERLAP(기본 70%).
  위→아래 순서(중심 y)와 가로 정렬·폭 비 조건은 유지. 찾은 칸 영역(blob)을 박스별로 기억한다.
- live_mission_optimized.lens_boxes: 박스를 세로 3등분하던 것을, 기억해 둔 실제 칸 영역으로 바꾼다.
  이웃 칸과 겹치는 부분은 두 칸 중심의 중간선에서 자르고, 원래처럼 안쪽 20% 여백을 둔다.
  기억된 칸 정보가 없는 박스는 원래 3등분 함수를 그대로 쓴다.

근거: B07(734,950)·B09(600,700) 상단 카메라 60° 렌더에서 R·Y 칸이 화면에서 겹쳐
  몸통 검색 실패(겹침 15 px > 허용 14.8 px) 또는 R 점등이 Y 칸에 번짐(multiple_lit:RY).
"""
import numpy as np

OVERLAP = 0.7
_tiers = {}
_original = {}


def _stacked(top, bottom, overlap=None):
    overlap = OVERLAP if overlap is None else overlap
    tx, ty, tw, th = top
    bx, by, bw, bh = bottom
    gap = by - (ty + th)
    return (abs(tx + tw/2 - bx - bw/2) <= .6*max(tw, bw)
            and .5 < tw/max(bw, 1) < 2.
            and ty + th/2 < by + bh/2                       # 위 칸 중심이 더 위
            and -overlap*min(th, bh) <= gap <= .6*bh + 3)


def detect(bgr):
    import cv2
    import tower_live_hsv as T
    evidence = T.masks(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
    minimum = max(8, bgr.shape[0]*bgr.shape[1]//40000)
    red, yellow, green = [T._blobs(evidence[c], minimum) for c in "RYG"]
    candidates = []
    for r in red:
        for y in yellow:
            if _stacked(r, y):
                for g in green:
                    if _stacked(y, g):
                        x0 = min(r[0], y[0], g[0])
                        x1 = max(r[0]+r[2], y[0]+y[2], g[0]+g[2])
                        box = (x0, r[1], x1, g[1]+g[3])
                        candidates.append((box, dict(R=r, Y=y, G=g)))
    if not candidates:
        return None
    box, tiers = max(candidates, key=lambda c: (c[0][2]-c[0][0])*(c[0][3]-c[0][1]))
    _tiers[tuple(int(v) for v in box)] = tiers
    return box


def lens_boxes(box, shape):
    import live_mission_optimized as L
    tiers = _tiers.get(tuple(int(v) for v in box))
    if tiers is None:
        return _original['lens_boxes'](box, shape)
    centers = {c: t[1] + t[3]/2 for c, t in tiers.items()}
    cuts = [-np.inf, (centers['R']+centers['Y'])/2, (centers['Y']+centers['G'])/2, np.inf]
    result = {}
    for i, c in enumerate('RYG'):
        x, y, w, h = tiers[c]
        top, bottom = max(y, cuts[i]), min(y+h, cuts[i+1])
        height = bottom - top
        b = (int(np.floor(x + .2*w)), int(np.floor(top + .2*height)),
             int(np.ceil(x + w - .2*w)), int(np.ceil(bottom - .2*height)))
        if b[0] < 0 or b[1] < 0 or b[2] > shape[1] or b[3] > shape[0] or b[2] <= b[0] or b[3] <= b[1]:
            return None
        result[c] = (*b, 0.)
    return result


def install():
    import live_mission_optimized as L
    import tower_live_hsv as T
    if _original:
        return
    _original.update(detect=T.detect, lens_boxes=L.lens_boxes)
    T.detect = detect
    L.lens_boxes = lens_boxes


def observed_tiers(box):
    """A copy of RGB blob boxes used by the latest matching observation."""
    if box is None:
        return None
    tiers = _tiers.get(tuple(int(v) for v in box))
    return None if tiers is None else dict(tiers)


def uninstall():
    """Restore B08 wrist acquisition and discard all mast ROI cache entries."""
    import live_mission_optimized as L
    import tower_live_hsv as T
    if _original:
        T.detect = _original['detect']
        L.lens_boxes = _original['lens_boxes']
    _original.clear()
    _tiers.clear()
