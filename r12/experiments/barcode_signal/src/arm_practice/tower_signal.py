"""신호등(세로 3단 스택 라이트) 색 판정. 학습 모델 없음.

방식: 맵에 고정된 신호등의 실측 3D 위치를 현재 카메라 자세(FK)로 화면에 투영하고,
      렌즈 칸(위=빨, 가운데=노, 아래=초)마다 밝은 픽셀 비율로 켜짐을 판정한다.
      바코드 위치에 의존하지 않는다 (기존 r12_sensor_sim.py 509~518줄 방식 대체).

결과 색: 'R' / 'Y' / 'G' / 'OFF' / 'UNKNOWN'
  - 밝은 면적과 흰 포화 면적을 함께 비교해 한 칸이 충분히 우세하면 그 색
  - 세 칸 모두 최소 밝은 면적 미달 → 'OFF'
  - 칸별 점수가 비슷함, 화면 밖, 너무 작게 보임, 앞이 가려짐 → 'UNKNOWN'
    (SignalRecorder 가 UNKNOWN 을 받으면 오류로 멈춤 = 추정하지 않음)

좌표 약속
  - T_world_cam: 월드(맵) → 카메라 광학 좌표 변환의 역이 아니라, 카메라 광학 프레임의 월드 자세 (4×4).
    광학 프레임 축 = x 오른쪽, y 아래, z 앞 (mixed_camera_schema 의 optical_axes 와 같음)
  - depth: distance_to_image_plane (m), rgb 와 같은 해상도
"""
import numpy as np

# ---------------------------------------------------------------------------
# 신호등 치수·위치 (여기 한곳에서만 바꾼다)
#   실측 (choi, 2026-10-29, p2_perception 수정.md "합성 신호등 치수 실측값 반영"):
#     기둥 68.9 mm, 렌즈 지름 39.6 mm, 렌즈 한 단 34.9 mm, "렌즈 3단 108 mm", 위 덮개 11 mm
#   아래 값들은 실측이 아니라 [가정] 이 들어간 계산이다. 정확한 실측 모델로 보지 말 것:
#     - 받침 높이 15 mm: 사진 추정 (미측정)
#     - "3단 108 mm" 를 덮개 제외 렌즈 3단으로 해석 → 단 간격 36 mm (덮개 포함이면 32.3 mm 로 바뀜)
#     - 렌즈는 단 간격 안 가운데 (위아래 이음매 0.55 mm 씩)
#   → 빨강 렌즈 위 = 15 + 68.9 + 2×36 + 0.55 + 34.9 = 191.35 mm (기둥 68.9 + 108 = 176.9 와의 차이는 받침 15 mm 와 이음매)
#   가장 확실한 보정: 실물에서 "바닥 → 초록 렌즈 아래", "바닥 → 빨강 렌즈 위" 높이를 직접 재서 아래 두 값을 고칠 것
#   xy_m: [가정] 시뮬 장면 기준 위치. 실물 경기장에서 반드시 실측해 바꿀 것
# ---------------------------------------------------------------------------
_BASE_H = 0.015        # [가정] 받침 높이 (m)
_POLE_H = 0.0689       # 실측 기둥 높이 (m)
_TIER_PITCH = 0.036    # [가정] 단 간격 (m) = 108 / 3
_LENS_H = 0.0349       # 실측 렌즈 한 단 높이 (m)
_GAP = (_TIER_PITCH - _LENS_H) / 2


def _tier(k):
    z0 = _BASE_H + _POLE_H + k * _TIER_PITCH + _GAP
    return (round(z0, 5), round(z0 + _LENS_H, 5))


TOWER = {
    'xy_m': (0.12, 0.95),          # [가정] 신호등 중심축의 맵 좌표 (m)
    'base_z_m': 0.0,               # 신호등 받침이 놓인 바닥 높이 (m). 박스 위에 있으면 70 mm 등으로
    'lens_diameter_m': 0.0396,     # 실측
    'tiers': {'G': _tier(0), 'Y': _tier(1), 'R': _tier(2)},   # 받침 바닥 기준 렌즈 칸 z 범위 (m), 위 [가정] 포함
}

# 외형 (시뮬 렌더용, 판정에는 쓰지 않음) — 실사 사진 기준
#   p2_perception data/real/raw/1025_tower/tower_light_off/real_20261024_161703_0000.jpg 에서 측정한 꺼진 렌즈 평균 sRGB:
#     빨강 (121, 28, 27), 노랑 (145, 96, 16), 초록 (9, 91, 68)  — 자동 노출 사진이라 근사값 [가정]
#   꺼진 렌즈도 색이 있는 반투명 줄무늬 렌즈, 위에 흰 덮개(11 mm, 실측), 검은 기둥, 파란 3D 프린트 받침
TOWER_LOOK = {
    'lens_off_srgb': {'R': (121, 28, 27), 'Y': (145, 96, 16), 'G': (9, 91, 68)},
    'lens_on_srgb': {'R': (255, 40, 30), 'Y': (255, 170, 30), 'G': (40, 255, 90)},   # [가정] 켜진 사진 노랑 (233,163,26) 참고
    'cap_h_m': 0.011, 'cap_srgb': (225, 225, 215),                                  # 덮개 높이 실측, 색 [가정]
    'pole_radius_m': 0.006, 'pole_srgb': (20, 20, 20),                              # [가정]
    'base_size_m': (0.08, 0.04), 'base_srgb': (30, 80, 220),                        # 받침 가로·세로 [가정, 사진 추정]
}


def srgb_to_linear(rgb):
    """USD 재질 색(선형)으로 변환."""
    c = np.asarray(rgb, float) / 255.0
    return tuple(np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).tolist())


# 판정 기준값 [가정 → 실물 노출 고정 후 조정]
PARAMS = {
    'v_on': 200,          # 밝기(HSV V = RGB 최댓값) 이 이상이면 "밝은 픽셀"
    'on_ratio': 0.15,     # 최소 밝은 면적 비율. 실사 고정 노출에서 재조정 필요
    'white_v': 235,       # 흰 포화 후보의 최소 RGB 최댓값
    'white_s_max': 80,    # HSV S(0~255) 상한. 한 채널만 포화된 색광과 구분
    'white_unique_enabled': True,
    'white_unique_min': 0.30,  # candidate, requires real-video validation
    'white_other_max': 0.10,
    'delta_v': 40.,          # rise above confirmed OFF median, same pose/exposure
    'white_weight': 0.25, # 흰 포화 면적의 보조 점수 (검증 전 후보값)
    'dominance_ratio': 1.8, # 1위/2위 최소 점수 비
    'dominance_margin': 0.10, # 1위-2위 최소 점수 차
    'inner': 0.6,         # 칸 영역을 중심 기준 이 비율로 줄여서 봄 (이웃 칸 번짐·테두리 제외)
    'min_tier_px': 20,    # 줄인 칸 하나의 최소 픽셀 수. 이보다 작게 보이면 UNKNOWN (너무 멀다)
    'min_depth_m': 0.05,  # 카메라 앞 최소 거리
    'occlusion_tol_m': 0.15,  # 측정 깊이가 예상보다 이만큼 이상 가까우면 앞이 가려졌다고 봄
}


def _project(points_world, T_world_cam, K):
    """월드 점들 (N×3) → 화면 좌표 (N×2), 카메라 z (N,)."""
    T_cam_world = np.linalg.inv(np.asarray(T_world_cam, float))
    p = np.c_[np.asarray(points_world, float), np.ones(len(points_world))] @ T_cam_world.T
    z = p[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = K[0, 0] * p[:, 0] / z + K[0, 2]
        v = K[1, 1] * p[:, 1] / z + K[1, 2]
    return np.c_[u, v], z


def tier_boxes(T_world_cam, K, image_shape, tower=TOWER, params=PARAMS):
    """칸별 화면 사각형 {색: (x0, y0, x1, y1, 예상 깊이)}. 하나라도 못 보이면 (None, 이유)."""
    K = np.asarray(K, float)
    h, w = image_shape[:2]
    cx, cy = tower['xy_m']
    r = tower['lens_diameter_m'] / 2
    # 뒤쪽 원주까지 포함한 사각형은 위에서 볼 때 덮개·윗단과 겹친다.
    # 카메라를 향한 원통 옆면의 앞쪽 120도만 투영한다.
    camera_xy = np.asarray(T_world_cam, float)[:2, 3]
    toward_camera = camera_xy - np.array([cx, cy])
    if np.linalg.norm(toward_camera) < 1e-6:
        return None, 'view_too_vertical'
    front_angle = np.arctan2(toward_camera[1], toward_camera[0])
    angles = front_angle + np.linspace(-np.pi / 3, np.pi / 3, 12)
    boxes = {}
    for color, (z0, z1) in tower['tiers'].items():
        ring = np.c_[cx + r * np.cos(angles), cy + r * np.sin(angles)]
        pts = np.r_[np.c_[ring, np.full(12, tower['base_z_m'] + z0)],
                    np.c_[ring, np.full(12, tower['base_z_m'] + z1)]]
        uv, z = _project(pts, T_world_cam, K)
        if not np.all(np.isfinite(uv)) or np.any(z < params['min_depth_m']):
            return None, 'behind_camera'
        x0, y0 = uv.min(axis=0)
        x1, y1 = uv.max(axis=0)
        # 중심 기준으로 inner 배 축소
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        hw, hh = (x1 - x0) * params['inner'] / 2, (y1 - y0) * params['inner'] / 2
        bx = (int(np.floor(mx - hw)), int(np.floor(my - hh)), int(np.ceil(mx + hw)), int(np.ceil(my + hh)))
        if bx[0] < 0 or bx[1] < 0 or bx[2] > w or bx[3] > h:
            return None, 'outside_image'
        if (bx[2] - bx[0]) * (bx[3] - bx[1]) < params['min_tier_px']:
            return None, 'too_small'
        boxes[color] = (*bx, float(np.median(z)))
    return boxes, None


def signal_frame_fresh(packet, *, now_ns, max_age_ns):
    """기존 checked_signal_color 의 신원·신선도 검사만 재사용한다 (색 값은 보지 않음).

    생산자(시뮬)가 넣은 색 대신 이 모듈이 다시 판정하므로, 검사용 사본에서만 색을 'OFF' 로 채운다.
    원본 packet 은 바꾸지 않는다. 신원·신선도가 맞으면 True.
    """
    try:
        from .signal_observation import checked_signal_color          # tutor src/arm_practice 안에 넣었을 때
    except ImportError:
        from arm_practice.signal_observation import checked_signal_color   # 단독 실행: tutor src 를 sys.path 에 둔 경우
    obs = packet.get('signal_observation') if isinstance(packet, dict) else None
    if not isinstance(obs, dict):
        return False
    probe = dict(packet, signal_observation=dict(obs, color='OFF'))
    return checked_signal_color(probe, now_ns=now_ns, max_age_ns=max_age_ns) == 'OFF'


def tier_rois(T_world_cam, K, image_shape, tower=TOWER, params=PARAMS):
    """Project interior side-surface patches; exclude caps and follow camera roll."""
    cx, cy = tower['xy_m']
    toward = np.asarray(T_world_cam, float)[:2, 3] - [cx, cy]
    if np.linalg.norm(toward) < 1e-6:
        return None, None, 'view_too_vertical'
    front = np.arctan2(toward[1], toward[0])
    span = np.pi / 3 * params['inner']
    angles = np.array([front-span, front+span])
    ring = np.c_[cx+tower['lens_diameter_m']/2*np.cos(angles),
                 cy+tower['lens_diameter_m']/2*np.sin(angles)]
    height, width = image_shape[:2]
    boxes, polygons = {}, {}
    for color, (z0, z1) in tower['tiers'].items():
        mid, half = (z0+z1)/2, (z1-z0)*params['inner']/2
        pts = np.array([[*ring[0], mid-half+tower['base_z_m']],
                        [*ring[1], mid-half+tower['base_z_m']],
                        [*ring[1], mid+half+tower['base_z_m']],
                        [*ring[0], mid+half+tower['base_z_m']]])
        uv, depth = _project(pts, T_world_cam, K)
        if not np.all(np.isfinite(uv)) or np.any(depth < params['min_depth_m']):
            return None, None, 'behind_camera'
        x0,y0 = np.floor(uv.min(axis=0)).astype(int)
        x1,y1 = np.ceil(uv.max(axis=0)).astype(int)
        if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
            return None, None, 'outside_image'
        area = .5*abs(np.dot(uv[:,0],np.roll(uv[:,1],1))-np.dot(uv[:,1],np.roll(uv[:,0],1)))
        if area < params['min_tier_px']:
            return None, None, 'too_small'
        boxes[color] = (int(x0),int(y0),int(x1),int(y1),float(np.median(depth)))
        polygons[color] = uv.tolist()
    return boxes, polygons, None


def camera_context(T_world_cam, K, image_shape, exposure_token=None):
    """Reference identity includes pose, intrinsics, frame size and exposure identity."""
    import hashlib
    import json
    value = [np.asarray(T_world_cam,float).tolist(),np.asarray(K,float).tolist(),
             list(image_shape[:2]), exposure_token]
    return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()


def _roi_masks(rgb, boxes, roi_polygons=None):
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError('invalid_image')
    if not boxes or set(boxes) != set('RYG'):
        raise ValueError('invalid_boxes')
    height,width = rgb.shape[:2]
    masks={}
    for c,(x0,y0,x1,y1,_) in boxes.items():
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise ValueError('invalid_boxes')
        mask=np.ones((y1-y0,x1-x0),bool)
        if roi_polygons is not None:
            import cv2
            if set(roi_polygons)!=set('RYG'):
                raise ValueError('invalid_polygons')
            points=np.asarray(roi_polygons[c],float)
            if points.shape!=(4,2) or not np.all(np.isfinite(points)):
                raise ValueError('invalid_polygons')
            if np.any(points[:,0]<x0) or np.any(points[:,0]>x1) or np.any(points[:,1]<y0) or np.any(points[:,1]>y1):
                raise ValueError('invalid_polygons')
            mask=np.zeros((y1-y0,x1-x0),np.uint8)
            cv2.fillConvexPoly(mask,np.rint(points-[x0,y0]).astype(np.int32),1)
            mask=mask.astype(bool)
        if not mask.any():
            raise ValueError('empty_roi')
        masks[c]=mask
    return masks


def _roi_key(boxes, roi_polygons):
    return (tuple((c,tuple(boxes[c])) for c in 'RYG'),
            tuple((c,tuple(map(tuple,roi_polygons[c]))) for c in 'RYG') if roi_polygons is not None else None)


class OffReference:
    """Known-OFF median, tied to one fixed camera/exposure/ROI configuration."""
    def __init__(self, rgb, boxes, polygons, context_key, frame_count):
        self.rgb=np.array(rgb,np.uint8,copy=True)
        self.rgb.setflags(write=False)
        self.brightness=self.rgb.max(axis=2).astype(np.float32)
        self.brightness.setflags(write=False)
        self.roi_key=_roi_key(boxes,polygons)
        self.context_key=context_key
        self.frame_count=frame_count


def make_off_reference(frames, boxes, *, context_key, confirmed_off=False, roi_polygons=None):
    """Only use actual confirmed OFF frames; never fill this from predictions or other poses."""
    if confirmed_off is not True or not isinstance(context_key,str) or not context_key:
        raise ValueError('confirmed OFF and a camera/exposure context are required')
    frames=np.asarray(frames)
    if frames.ndim!=4 or frames.shape[0]<3 or frames.shape[3]<3 or frames.dtype!=np.uint8:
        raise ValueError('at least three uint8 RGB OFF frames required')
    _roi_masks(frames[0],boxes,roi_polygons)
    median_rgb=np.median(frames[:,:,:,:3],axis=0).astype(np.uint8)
    return OffReference(median_rgb,boxes,roi_polygons,context_key,len(frames))


def tower_signal_color(rgb, T_world_cam, K, depth=None, tower=TOWER, params=PARAMS,
                       *, off_reference=None, exposure_token=None):
    """한 프레임의 신호등 색. 반환: dict(color, reason, ratios, boxes)."""
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        return dict(color='UNKNOWN', reason='invalid_image', ratios={}, boxes={})
    boxes, polygons, reason = tier_rois(T_world_cam, K, rgb.shape, tower, params)
    if boxes is None:
        return dict(color='UNKNOWN', reason=reason, ratios={}, boxes={})
    # 가림 검사: 칸 영역의 측정 깊이가 예상보다 확실히 가까우면 앞에 다른 물체가 있음
    if depth is not None:
        depth = np.asarray(depth, float)
        if depth.shape[:2] != rgb.shape[:2]:
            return dict(color='UNKNOWN', reason='depth_shape_mismatch', ratios={}, boxes=boxes)
        for color, (x0, y0, x1, y1, expected) in boxes.items():
            patch = depth[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size and np.median(valid) < expected - params['occlusion_tol_m']:
                return dict(color='UNKNOWN', reason='occluded:' + color, ratios={}, boxes=boxes)
    return color_from_boxes(rgb, boxes, params, roi_polygons=polygons,
                            off_reference=off_reference,
                            context_key=camera_context(T_world_cam,K,rgb.shape,exposure_token))


def color_from_boxes(rgb, boxes, params=PARAMS, *, roi_polygons=None,
                     off_reference=None, context_key=None):
    """Candidate color rule on explicit lens interiors. OFF reference is optional and explicit."""
    rgb=np.asarray(rgb)
    try:
        masks=_roi_masks(rgb,boxes,roi_polygons)
    except ValueError as exc:
        return dict(color='UNKNOWN',reason=str(exc),ratios={},boxes=boxes or {})
    if off_reference is not None:
        if (off_reference.brightness.shape!=rgb.shape[:2] or
                off_reference.roi_key!=_roi_key(boxes,roi_polygons) or
                not context_key or off_reference.context_key!=context_key):
            return dict(color='UNKNOWN',reason='off_reference_mismatch',ratios={},boxes=boxes)
    ratios,white_ratios,scores,raw_ratios,raw_white_ratios,delta_ratios={},{},{},{},{},{}
    for c,(x0,y0,x1,y1,_) in boxes.items():
        mask=masks[c]
        # Preserve full-frame reference/ROI identity checks, compute only ROI pixels.
        pixels=rgb[y0:y1,x0:x1,:3].astype(np.float32)
        brightness=pixels.max(axis=2)
        saturation=np.divide(brightness-pixels.min(axis=2),brightness,
                             out=np.zeros_like(brightness),where=brightness>0)*255.
        bright=brightness>=params['v_on']
        white=(brightness>=params['white_v']) & (saturation<=params['white_s_max'])
        changed=np.ones_like(bright)
        if off_reference is not None:
            ref_b=off_reference.brightness[y0:y1,x0:x1]
            ref_pixels=off_reference.rgb[y0:y1,x0:x1].astype(np.float32)
            ref_s=np.divide(ref_b-ref_pixels.min(axis=2),ref_b,
                            out=np.zeros_like(ref_b),where=ref_b>0)*255.
            ref_white=(ref_b>=params['white_v']) & (ref_s<=params['white_s_max'])
            changed=((brightness-ref_b)>=params['delta_v']) | (white & ~ref_white & (brightness>=ref_b))
        b,w,d=bright[mask],white[mask],changed[mask]
        raw_ratios[c]=float(b.mean())
        raw_white_ratios[c]=float(w.mean())
        delta_ratios[c]=float(d.mean()) if off_reference is not None else None
        ratios[c]=float((b & d).mean())
        white_ratios[c]=float((w & d).mean())
        scores[c]=ratios[c]+params['white_weight']*white_ratios[c]
    detail=dict(ratios=ratios,white_ratios=white_ratios,scores=scores,boxes=boxes,
                roi_polygons=roi_polygons,raw_ratios=raw_ratios,raw_white_ratios=raw_white_ratios,
                delta_ratios=delta_ratios,off_reference_used=off_reference is not None)
    lit=[c for c in 'RYG' if ratios[c]>=params['on_ratio']]
    if not lit:
        return dict(color='OFF',reason=None,decision='off_delta' if off_reference is not None else 'off',**detail)
    if params.get('white_unique_enabled',True):
        unique=[c for c in lit if white_ratios[c]>=params['white_unique_min'] and
                all(white_ratios[other]<=params['white_other_max'] for other in 'RYG' if other!=c)]
        if len(unique)==1:
            return dict(color=unique[0],reason=None,decision='unique_white',**detail)
    winner=max(lit,key=scores.get)
    runner=max(scores[c] for c in 'RYG' if c!=winner)
    margin=scores[winner]-runner
    dominant=margin>=params['dominance_margin'] and scores[winner]>=params['dominance_ratio']*runner
    if dominant:
        return dict(color=winner,reason=None,decision='dominant_bloom' if len(lit)>1 else 'single',margin=margin,**detail)
    reason=('multiple_lit:'+''.join(lit)) if len(lit)>1 else 'ambiguous:'+winner
    return dict(color='UNKNOWN',reason=reason,decision='ambiguous',margin=margin,**detail)
