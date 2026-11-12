"""NZ2 퍽 인식 오차 평가 — percep-dump 로 저장한 시뮬 영상(npz)에 nz2_puck_vision.detect 를 돌려 정답과 비교

설치 위치: ~/manipulator-isaac61-lyrical-practice/scripts/nz2_percep_eval.py
실행 (개발 PC, Isaac 불필요: numpy + opencv):
  python3 scripts/nz2_percep_eval.py agent/percep/<이름>
  → 표 출력 + agent/percep/<이름>/eval.json
깊이 조건
  ideal   : 시뮬 깊이 그대로
  d435_175: D435 흉내 [가정] — 0.175 m 보다 가까우면 0, 잡음 σ 1 mm @0.4 m (거리² 비례), 화소 2 % 구멍
  d435_250: 같은데 최소 거리 0.25 m (사용자 기억: 0.20~0.25 m 에서 깊이 0)
방법
  ours : nz2_puck_vision.detect (색 분할 + 3D 점 + 윗면/능선 사각형)
  team : 팀 PR #26 puck_registry._depth_center 그대로 (YOLO 박스 대신 색 분할 박스 = 완벽한 YOLO 가정,
         박스 가운데 50 % 깊이 중앙값 → 박스 중심 시선 + 15 mm). 태그(16 mm AprilTag) 경로는 시뮬 퍽에 태그가 없어 평가 못 함
look 은 퍽 2개 모두, via·pre 는 명령(목표) 퍽만 평가
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, "..", "src", "arm_practice", "tasks"), HERE):
    sys.path.insert(0, os.path.abspath(p))
import nz2_puck_vision as PV  # noqa: E402

PUCKS = ["r_box", "r_cyl", "g_box", "g_cyl"]          # nz2_vispick_cfg.PUCKS 순서 (번호 = 색*2 + 모양)


def quat_rot(q):
    """(w, x, y, z) → 3×3"""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def team_depth_center(depth, box, K):
    """팀 PR #26 puck_registry.PuckRegistry._depth_center 를 그대로 옮김 (카메라 좌표 중심)"""
    h, w = depth.shape
    x1, y1, x2, y2 = box
    cx, cy, bw, bh = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
    u0, u1 = int(max(0, cx - bw / 4)), int(min(w, cx + bw / 4 + 1))
    v0, v1 = int(max(0, cy - bh / 4)), int(min(h, cy + bh / 4 + 1))
    roi = depth[v0:v1, u0:u1]
    z = roi[(roi > 0.05) & (roi < 3.0) & np.isfinite(roi)]
    if z.size < 10:
        return None
    zm = float(np.median(z))
    ray = np.array([(cx - K[0, 2]) / K[0, 0], (cy - K[1, 2]) / K[1, 1], 1.0])
    return ray * zm + ray / np.linalg.norm(ray) * 0.015


def depth_variant(d, kind, rng):
    d = d.astype(np.float64).copy()
    d[~np.isfinite(d)] = 0
    if kind == "ideal":
        return d
    d[d < (0.25 if kind == "d435_250" else 0.175)] = 0
    v = d > 0
    d[v] += rng.normal(0, 1, v.sum()) * 0.001 * (d[v] / 0.4) ** 2
    d[rng.random(d.shape) < 0.02] = 0
    return d


def gt_list(z):
    out = []
    for c, cname in enumerate(("r", "g")):
        if not bool(z["on_box"][c]):
            continue
        sh = int(z["shape"][c])
        k = c * 2 + sh
        pos = z["puck_pos"][k]
        ax = quat_rot(z["puck_quat"][k]) @ np.array([0, 0, 1.0])
        out.append(dict(color=cname, shape=("box", "cyl")[sh], lying=bool(z["lying"][c]), xy=pos[:2],
                        z=float(pos[2]), yaw=math.atan2(ax[1], ax[0]), target=(c == int(z["target_color"]))))
    return out


def evaluate(folder):
    rng = np.random.default_rng(0)
    files = sorted(glob.glob(os.path.join(folder, "*.npz")))
    if not files:
        raise SystemExit(f"npz 없음: {folder}")
    rows = []
    for f in files:
        z = np.load(f)
        tag = str(z["tag"])
        gts = gt_list(z)
        masks = PV.color_masks(z["rgb"])
        for kind in ("ideal", "d435_175", "d435_250"):
            dep = depth_variant(z["depth"], kind, rng)
            P = dict(PV.PARAMS, depth_min_m=0.05)            # 최소 거리는 depth_variant 가 이미 처리
            dets = PV.detect(z["rgb"], dep, z["K"], z["R_ac"], z["t_ac"], float(z["box_top"]), P)
            best = PV.best_per_color(dets)
            # 팀 방법: 색마다 가장 큰 성분의 외접 박스 = YOLO 박스 대용
            team = {}
            for cname, m in masks.items():
                n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
                if n < 2:
                    continue
                i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
                if st[i, cv2.CC_STAT_AREA] < PV.PARAMS["min_area_px"]:
                    continue
                x, y, bw, bh = st[i, :4]
                c = team_depth_center(dep, (x, y, x + bw, y + bh), z["K"])
                if c is not None:
                    team[cname] = z["R_ac"] @ c + z["t_ac"]
            near = float(np.median(z["depth"][z["depth"] > 0])) if (z["depth"] > 0).any() else float("nan")
            for g in gts:
                if tag != "look" and not g["target"]:
                    continue
                d = best.get(g["color"])
                r = dict(file=os.path.basename(f), tag=tag, depth=kind, kind=("누움" if g["lying"] else "세움") + "_" + g["shape"],
                         found=d is not None, cam_med_depth_m=round(near, 3))
                # 그 색 화소가 있었는지 (거부 이유)
                rej = [x.info.get("reject") for x in dets if x.color == g["color"] and x.info.get("reject")]
                r["reject"] = rej[0] if (d is None and rej) else ""
                tc = team.get(g["color"])
                r["team_found"] = tc is not None
                if tc is not None:
                    r["team_xy_err_mm"] = float(np.linalg.norm(tc[:2] - g["xy"]) * 1000)
                    r["team_z_err_mm"] = float((tc[2] - g["z"]) * 1000)
                if d is not None:
                    r["xy_err_mm"] = float(np.linalg.norm(d.xy - g["xy"]) * 1000)
                    r["lying_ok"] = bool(d.lying == g["lying"])
                    r["shape_ok"] = bool(d.shape == g["shape"])
                    if g["lying"]:
                        e = (d.yaw - g["yaw"] + math.pi / 2) % math.pi - math.pi / 2
                        r["yaw_err_deg"] = abs(math.degrees(e))
                rows.append(r)
    return rows


def summarize(rows):
    def q(v, p):
        return float(np.percentile(v, p)) if len(v) else float("nan")
    groups = {}
    for r in rows:
        groups.setdefault((r["tag"], r["depth"], r["kind"]), []).append(r)
    order = {"look": 0, "via": 1, "pre": 2}
    table = []
    for key in sorted(groups, key=lambda k: (order.get(k[0], 9), k[1], k[2])):
        g = groups[key]
        f = [x for x in g if x["found"]]
        xy = [x["xy_err_mm"] for x in f]
        yaw = [x["yaw_err_deg"] for x in f if "yaw_err_deg" in x]
        rej = {}
        for x in g:
            if x["reject"]:
                rej[x["reject"]] = rej.get(x["reject"], 0) + 1
        table.append(dict(tag=key[0], depth=key[1], kind=key[2], n=len(g), found=len(f) / len(g),
                          xy_mean=float(np.mean(xy)) if xy else float("nan"), xy_p95=q(xy, 95), xy_max=max(xy) if xy else float("nan"),
                          over6=sum(v > 6 for v in xy), over10=sum(v > 10 for v in xy),
                          lying_ok=float(np.mean([x["lying_ok"] for x in f])) if f else float("nan"),
                          shape_ok=float(np.mean([x["shape_ok"] for x in f])) if f else float("nan"),
                          yaw_p95=q(yaw, 95), reject=rej,
                          team_found=float(np.mean([x["team_found"] for x in g])),
                          team_xy_mean=float(np.mean([x["team_xy_err_mm"] for x in g if "team_xy_err_mm" in x])) if any("team_xy_err_mm" in x for x in g) else float("nan"),
                          team_xy_p95=q([x["team_xy_err_mm"] for x in g if "team_xy_err_mm" in x], 95),
                          team_z_mean=float(np.mean([x["team_z_err_mm"] for x in g if "team_z_err_mm" in x])) if any("team_z_err_mm" in x for x in g) else float("nan"), cam_depth_med=float(np.median([x["cam_med_depth_m"] for x in g]))))
    return table


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    rows = evaluate(folder)
    table = summarize(rows)
    hdr = f"{'순간':5s} {'깊이':8s} {'종류':8s} {'n':>4s} {'검출':>5s} {'xy평균':>7s} {'xy95%':>7s} {'xy최대':>7s} {'>6mm':>5s} {'>10mm':>6s} {'자세':>5s} {'모양':>5s} {'yaw95%':>7s} {'화면깊이':>7s} | 팀:검출 xy평균 xy95% z평균 | 거부"
    print(hdr)
    for t in table:
        print(f"{t['tag']:5s} {t['depth']:8s} {t['kind']:8s} {t['n']:4d} {t['found']:5.2f} {t['xy_mean']:7.2f} {t['xy_p95']:7.2f} {t['xy_max']:7.2f} "
              f"{t['over6']:5d} {t['over10']:6d} {t['lying_ok']:5.2f} {t['shape_ok']:5.2f} {t['yaw_p95']:7.2f} {t['cam_depth_med']:7.3f} | {t['team_found']:5.2f} {t['team_xy_mean']:6.1f} {t['team_xy_p95']:6.1f} {t['team_z_mean']:6.1f} | {t['reject']}")
    with open(os.path.join(folder, "eval.json"), "w", encoding="utf-8") as f:
        json.dump({"table": table, "rows": rows}, f, ensure_ascii=False, indent=1)
    print(f"→ {os.path.join(folder, 'eval.json')}")


if __name__ == "__main__":
    main()
