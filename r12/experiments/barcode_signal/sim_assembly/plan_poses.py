#!/usr/bin/env python3
"""최종 URDF 정기구학으로 바코드·신호등 관측 팔 자세 후보를 계산한다 (Isaac 불필요).

목표(가정값은 config 에서 바꿈):
- 바코드: 라벨 중심을 화면 중앙에, 정면(라벨 법선 +X) 가까이, 지정 거리.
- 신호등: 렌즈 3단 중심을 화면 중앙에, 지정 거리, 내려다보는 각 제한
  (2026-11-12 profile160 시험: 약 56.6° 내려다보면 기존 배열 검사가 거부함).
- 공통: 화면 수평 유지, 관절 범위 여유 0.05 rad, 차체 판·마스트와 거리 확보.

결과는 계산 후보이며 도달·가림·검출은 run.py 렌더로 확인해야 한다.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import minimize

from common import ARM_JOINTS, PERCEPTION_ROOT, Model, arm_limits, base_transform, load_config

LABEL_WIDTH_M = 0.075
# 마스트(시각 메시만 있음): base_link 기준 x≈0.006, y≈-0.025, z 0~0.48 근처 [CAD 렌즈 위치에서 추정]
MAST_XY = np.array([0.006, -0.025])


def tower_target(cfg=None):
    sys.path.insert(0, str(PERCEPTION_ROOT / 'scripts'))
    from live_mission_optimized import load_color_rule
    tower = dict(load_color_rule().TOWER)
    if cfg and cfg.get('tower_scene'):
        tower['xy_m'] = cfg['tower_scene']['xy_m']
        tower['base_z_m'] = cfg['tower_scene'].get('base_z_m', tower['base_z_m'])
    lo = min(p[0] for p in tower['tiers'].values())
    hi = max(p[1] for p in tower['tiers'].values())
    return np.array([*tower['xy_m'], tower['base_z_m'] + (lo+hi)/2]), hi-lo


def evaluate(model, q, base, target, distance, fx, normal=None, max_elev_deg=None, aim_up_deg=0.):
    cam = model.camera_world(q, base)
    p, x, y, z = cam[:3, 3], cam[:3, 0], cam[:3, 1], cam[:3, 2]
    v = target - p
    d = float(np.linalg.norm(v))
    # 목표를 화면 중앙보다 aim_up_deg 위에 둔다(아래쪽은 집게가 가림). y = 화면 아래 방향.
    a = math.radians(aim_up_deg)
    aim = z*math.cos(a) - y*math.sin(a)
    off_axis = math.degrees(math.acos(np.clip(aim @ v / d, -1, 1)))
    info = dict(distance_m=d, off_axis_deg=off_axis, camera_world_m=p.tolist(),
                image_right_tilt_deg=math.degrees(math.asin(np.clip(x[2], -1, 1))),
                look_down_deg=math.degrees(math.asin(np.clip(-v[2]/d, -1, 1))))
    if normal is not None:
        info['incidence_deg'] = math.degrees(math.acos(np.clip(-(z @ normal), -1, 1)))
        info['label_width_px_est'] = fx * LABEL_WIDTH_M / d * math.cos(math.radians(info['incidence_deg']))
    # 차체 판/마스트 근접(손목·카메라·팔꿈치 지점, base 좌표)
    inv = np.linalg.inv(base_transform(base))
    clear = []
    for link in ('elbow', 'wrist_pitch', 'wrist_roll'):
        pt = (inv @ model.link_transform(link, model.arm_q(q), base_transform(base)))[:3, 3]
        clear.append(pt)
    clear.append((inv @ cam)[:3, 3])
    mast = min(float(np.linalg.norm(pt[:2]-MAST_XY)) for pt in clear if pt[2] < 0.5)\
        if any(pt[2] < 0.5 for pt in clear) else 1.
    plate = min(float(pt[2]) if (abs(pt[0]) < 0.2 and abs(pt[1]) < 0.2) else 1. for pt in clear)
    info.update(mast_clearance_m=mast, plate_height_m=plate)
    return info, z, x, y


def cost(q, model, base, target, distance, fx, normal, max_elev_deg, max_incidence_deg, aim_up_deg):
    info, z, x, y = evaluate(model, q, base, target, distance, fx, normal, max_elev_deg, aim_up_deg)
    c = 20*(1-math.cos(math.radians(info['off_axis_deg'])))
    c += 50*(info['distance_m']-distance)**2
    c += 5*x[2]**2                                   # 화면 수평
    c += 5*max(0., y[2])**2                          # 화면 아래 = 월드 아래
    if normal is not None:
        c += 2*max(0., info['incidence_deg']-max_incidence_deg)**2/100
    if max_elev_deg is not None:
        c += 2*max(0., info['look_down_deg']-max_elev_deg)**2/100
    c += 50*max(0., 0.08-info['mast_clearance_m'])**2
    c += 50*max(0., 0.08-info['plate_height_m'])**2
    return c


def solve(model, base, target, distance, fx, lower, upper, normal=None, max_elev_deg=None,
          max_incidence_deg=15., starts=200, seed=0, aim_up_deg=0.):
    rng = np.random.default_rng(seed)
    lo, hi = lower+.05, upper-.05
    best = None
    args = (model, base, target, distance, fx, normal, max_elev_deg, max_incidence_deg, aim_up_deg)
    for i in range(starts):
        q0 = np.zeros(5) if i == 0 else rng.uniform(lo, hi)
        r = minimize(cost, q0, args=args, method='L-BFGS-B', bounds=list(zip(lo, hi)))
        if best is None or r.fun < best.fun:
            best = r
    info = evaluate(model, best.x, base, target, distance, fx, normal, max_elev_deg, aim_up_deg)[0]
    info.update(cost=float(best.fun), aim_up_deg=aim_up_deg, q_rad=[round(float(v), 4) for v in best.x],
                gravity_torque_nm=[round(t, 3) for t in model.gravity_torque(best.x, base)])
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    ap.add_argument('--barcode-distance', type=float, default=0.22)
    ap.add_argument('--tower-distance', type=float, default=0.35)
    ap.add_argument('--tower-max-look-down', type=float, default=25.)
    ap.add_argument('--barcode-aim-up', type=float, default=10., help='목표를 화면 중앙보다 위로(도)')
    ap.add_argument('--tower-aim-up', type=float, default=12.)
    ap.add_argument('--starts', type=int, default=200)
    a = ap.parse_args()
    cfg, model = load_config(a.config, check_poses=False)  # 새 자세를 계산하므로 기존 자세 검사 생략
    lower, upper = arm_limits(cfg, model)
    cam = cfg['camera']
    fx = cam['width']/2/math.tan(math.radians(cam['hfov_deg'])/2)
    tower, tower_h = tower_target(cfg)
    barcode = solve(model, cfg['barcode_base_xyz_yaw'], np.array(cfg['barcode']['center_m']),
                    a.barcode_distance, fx, lower, upper, normal=np.array([1., 0, 0]), starts=a.starts,
                    aim_up_deg=a.barcode_aim_up)
    tower_pose = solve(model, cfg['tower_base_xyz_yaw'], tower, a.tower_distance, fx, lower, upper,
                       max_elev_deg=a.tower_max_look_down, starts=a.starts, aim_up_deg=a.tower_aim_up)
    tower_pose['tower_stack_height_px_est'] = fx*tower_h/tower_pose['distance_m']
    print(json.dumps(dict(fx_px=fx, barcode=barcode, tower=tower_pose,
                          note='계산 후보. 렌더로 가림/검출 확인 필요'), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
