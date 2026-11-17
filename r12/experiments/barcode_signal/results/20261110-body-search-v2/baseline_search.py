"""Experimental local R/Y/G body registration; no detector network or scene truth.

Acquire on explicitly confirmed OFF frames, then keep ROIs fixed while camera,
exposure and tower stay stationary. A coarse pose supplies geometry and search
bounds only. Lighting and camera calibration still need real-image validation.
"""
import hashlib
import time

import cv2
import numpy as np

SEARCH = dict(radius_px=60, scales=(.85, .925, 1., 1.075, 1.15),
              min_fraction=.50, min_mean=.65, ambiguity_gap=.08,
              saturation_min=45, value_min=25, presence_fraction=.30)


def _evidence(rgb, params):
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb[:, :, :3]), cv2.COLOR_RGB2HSV)
    h, sat, v = cv2.split(hsv)
    usable = (sat >= params['saturation_min']) & (v >= params['value_min'])
    return {c: (usable & mask).astype(np.float32) for c, mask in {
        'R': (h <= 12) | (h >= 165), 'Y': (h >= 15) & (h <= 38),
        'G': (h >= 39) & (h <= 100)}.items()}


def _invalid(reason, started, **detail):
    return dict(found=False, reason=reason, boxes={}, polygons={},
                elapsed_ms=(time.perf_counter()-started)*1000, **detail)


def locate_body(rgb, projected_boxes, projected_polygons, *, confirmed_off=False,
                params=None):
    """Find a unique three-colour body near coarse projected polygons.

    No true camera pose, target mask, renderer object ID or active colour enters
    this function. Refuse missing, ambiguous, partial and unconfirmed targets.
    """
    started = time.perf_counter()
    p = dict(SEARCH, **(params or {}))
    rgb = np.asarray(rgb)
    if not confirmed_off:
        return _invalid('off_confirmation_required', started)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] < 3:
        return _invalid('invalid_rgb', started)
    if set(projected_boxes or {}) != set('RYG') or set(projected_polygons or {}) != set('RYG'):
        return _invalid('invalid_projection', started)
    polygons = {c: np.asarray(projected_polygons[c], float) for c in 'RYG'}
    if any(a.shape != (4, 2) or not np.isfinite(a).all() for a in polygons.values()):
        return _invalid('invalid_projection', started)
    all_points = np.concatenate(list(polygons.values()))
    center = all_points.mean(axis=0)
    h, w = rgb.shape[:2]
    radius = int(p['radius_px'])
    if radius < 1 or radius > 256:
        return _invalid('invalid_search_radius', started)
    evidence = _evidence(rgb, p)
    candidates = []
    centers = np.array([polygons[c].mean(axis=0) for c in 'RYG'])
    axis = centers[2]-centers[0]
    axis /= np.linalg.norm(axis)
    across = np.array([axis[1], -axis[0]])
    along_radius = max(4., min(np.linalg.norm(centers[0]-centers[1]),
                              np.linalg.norm(centers[1]-centers[2]))*.7)
    across_radius = max(4., np.median([np.linalg.norm(a[0]-a[1]) for a in polygons.values()])*1.3)
    def different_body(delta):
        return (np.dot(delta,axis)/along_radius)**2 + (np.dot(delta,across)/across_radius)**2 >= 1
    search_bounds = None
    for scale in p['scales']:
        if not .5 <= scale <= 1.5:
            return _invalid('invalid_scale', started)
        scaled = {c: (a-center)*scale+center for c, a in polygons.items()}
        pts = np.concatenate(list(scaled.values()))
        lo, hi = np.floor(pts.min(axis=0)).astype(int), np.ceil(pts.max(axis=0)).astype(int)+1
        tw, th = (hi-lo).tolist()
        x0, y0 = np.maximum(0, lo-radius)
        x1, y1 = np.minimum([w, h], hi+radius)
        if tw < 2 or th < 2 or x1-x0 < tw or y1-y0 < th:
            continue
        search_bounds = [int(x0), int(y0), int(x1), int(y1)]
        response = []
        for c in 'RYG':
            mask = np.zeros((th, tw), np.float32)
            cv2.fillPoly(mask, [np.rint(scaled[c]-lo).astype(np.int32)], 1.)
            area = mask.sum()
            if area < 8:
                return _invalid('projection_too_small', started)
            response.append(cv2.matchTemplate(evidence[c][y0:y1, x0:x1], mask,
                                              cv2.TM_CCORR)/area)
        fractions = np.stack(response)
        valid = fractions.min(axis=0) >= p['min_fraction']
        mean = fractions.mean(axis=0)
        rank = np.where(valid, mean, -1.).astype(np.float32)
        # Keep distinct spatial peaks, not several pixels on one colour patch.
        for _ in range(4):
            _, peak, _, loc = cv2.minMaxLoc(rank)
            if peak < p['min_mean']:
                break
            ix, iy = loc
            shift = np.array([x0+ix, y0+iy])-lo
            moved = {c: a+shift for c, a in scaled.items()}
            candidates.append(dict(score=float(peak), scale=float(scale),
                shift_px=shift.astype(float).tolist(), polygons=moved,
                center=center+shift,
                fractions={c: float(fractions[j, iy, ix]) for j, c in enumerate('RYG')}))
            yy, xx = np.ogrid[:rank.shape[0], :rank.shape[1]]
            dx, dy = xx-ix, yy-iy
            cluster = ((dx*axis[0]+dy*axis[1])/along_radius)**2 + ((dx*across[0]+dy*across[1])/across_radius)**2 < 1
            rank[cluster] = -1
    if not candidates:
        return _invalid('body_not_found', started, search_bounds=search_bounds)
    # Tiny preference for the projected scale resolves equally good fits only.
    candidates.sort(key=lambda c: c['score']-.01*abs(c['scale']-1), reverse=True)
    best = candidates[0]
    alternatives = [c for c in candidates[1:]
                    if different_body(c['center']-best['center'])]
    runner = max((c['score'] for c in alternatives), default=0.)
    if best['score']-runner < p['ambiguity_gap']:
        return _invalid('ambiguous_bodies', started, score=best['score'], runner=runner)
    boxes = {}
    for c, a in best['polygons'].items():
        lo, hi = np.floor(a.min(axis=0)).astype(int), np.ceil(a.max(axis=0)).astype(int)
        if np.any(lo < 0) or hi[0] > w or hi[1] > h:
            return _invalid('registered_roi_outside_image', started)
        boxes[c] = (*map(int, lo), *map(int, hi), float(projected_boxes[c][4]))
    polys = {c: a.tolist() for c, a in best.pop('polygons').items()}
    best.pop('center')
    key = hashlib.sha256(repr((boxes, polys, rgb.shape)).encode()).hexdigest()
    return dict(found=True, reason=None, boxes=boxes, polygons=polys,
                key=key, search_bounds=search_bounds, runner=runner,
                elapsed_ms=(time.perf_counter()-started)*1000, **best)


def read_registered(signal, rgb, registration, *, context_key, off_reference,
                    depth=None, params=None):
    """Check two unlit colour bands still exist, then use unchanged colour rule.

    This limited stationary-view check is not a full tracker. Reacquire from
    confirmed OFF frames after a pose/exposure change; never slide an OFF
    baseline to another ROI or reuse it after reacquisition.
    """
    started = time.perf_counter()
    p = dict(SEARCH, **(params or {}))
    fail = lambda reason: dict(color='UNKNOWN', reason=reason, boxes={}, ratios={})
    if not registration.get('found'):
        return fail('registration:'+str(registration.get('reason')))
    boxes, polygons = registration['boxes'], registration['polygons']
    if off_reference is None:
        return fail('registered_off_reference_required')
    evidence = _evidence(np.asarray(rgb), p)
    support = {}
    for c, (x0, y0, x1, y1, expected) in boxes.items():
        mask = np.zeros((y1-y0, x1-x0), np.uint8)
        cv2.fillPoly(mask, [np.rint(np.asarray(polygons[c])-[x0, y0]).astype(np.int32)], 1)
        support[c] = float(evidence[c][y0:y1, x0:x1][mask.astype(bool)].mean())
        if depth is not None:
            patch = np.asarray(depth)[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size and np.median(valid) < expected-signal.PARAMS['occlusion_tol_m']:
                return fail('registered_occluded:'+c)
    if sum(v >= p['presence_fraction'] for v in support.values()) < 2:
        return fail('registered_body_lost')
    result = signal.color_from_boxes(rgb, boxes, roi_polygons=polygons,
                                    off_reference=off_reference, context_key=context_key)
    result['body_support'] = support
    result['registration_read_ms'] = (time.perf_counter()-started)*1000
    return result
