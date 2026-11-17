#!/usr/bin/env python3
"""signal.json · allocation.json 생성 (팀 코드 사용, ROS 2 불필요)

- signal.json   : Isaac 시뮬 관측 결과(B16)를 제출 형식으로 옮김. 관측값을 바꾸지 않는다.
- allocation.json: 팀 nz2_mission 코드(mission_spec.assign_locations, allocator.allocate,
                   dice_layout.place_order)로 계산. 퍽 등록표는 팀 확정본(아래 REGISTRY).

실행 (개발 PC, 저장소 루트에서):
  python3 tools/build_results.py --repo . \
    --run r12/experiments/barcode_signal/results/assembly-auto-30cm-control-20261112-03 \
    --out results
"""
import argparse, json, math, sys
from pathlib import Path

# 팀 확정 퍽 등록표 (2026-11-17): 빨강 6 · 초록 6, 색마다 원통 3 · 직육면체 3
# 시작 박스 k (0~5) 에 ID 2k (X-40), 2k+1 (X+40)
BOX_CENTERS = [(1468, 950), (3670, 3152), (2936, 5354), (734, 5354), (2202, 5354), (3670, 3886)]
REGISTRY = [{"id": i, "color": "red" if i < 6 else "green", "shape": "cyl" if i % 6 < 3 else "cube",
             "box": i // 2, "start_mm": [BOX_CENTERS[i // 2][0] + (-40 if i % 2 == 0 else 40), BOX_CENTERS[i // 2][1], 70]}
            for i in range(12)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--run", required=True, help="Isaac 결과 폴더 (result.json, audit.json)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    repo = Path(a.repo)
    sys.path.insert(0, str(repo / "ros2_ws/src/planning/nz2_mission"))
    from nz2_mission.core.mission_spec import assign_locations
    from nz2_mission.core.world_model import WorldModel, PuckStatus
    from nz2_mission.core.allocator import allocate
    from nz2_mission.core.dice_layout import place_order, cell_world_mm
    import yaml

    run = repo / a.run
    res = json.loads((run / "result.json").read_text())
    aud = json.loads((run / "audit.json").read_text())
    m = res["mission"]
    seq = "".join(m["sequence"])
    phases = {p["phase"]: p["t_s"] for p in aud["phase_changes"]}

    signal = {
        "run_id": "assembly-sim-B16 (assembly-auto-30cm-control-20261112-03)",
        "env": "모의 — Isaac Sim, 전체 로봇 assembly.urdf, 손목 D435 640×480, PathTracing. 실물 아님",
        "source": f"{a.run}/result.json, audit.json",
        "barcode": {"text": m["barcode"], "order": m["direction"], "camera": "mast",
                    "saved_at_s": round(res["barcode_saved_at_s"], 3)},
        "observe": {"camera": res["signal_route"]["camera"], "record_start_s": round(phases["record"], 3),
                    "saved_at_s": round(phases["saved"], 3),
                    "record_duration_s": round(phases["saved"] - phases["record"], 2),
                    "ready_evidence": res["ready_evidence"]},
        "sequence": m["sequence"], "counts": m["counts"], "total": len(m["sequence"]),
        "end_detected": {"ok": bool(m["signal_complete"])},
        "frames": {"classified": res["classified_frames"], "correct_color": res["correct_color_frames"]},
        "scoring_truth": {"sequence": res["expected_sequence_for_scoring"], "barcode": res["expected_barcode_for_scoring"],
                          "sequence_match": res["observed_sequence"] == res["expected_sequence_for_scoring"]},
        "checks": aud["checks"],
        "status": "OK" if m["status"] == "ready" and m["error"] is None else "ERROR_STOP",
        "error": m["error"],
        "limits": ["시뮬 렌더 영상. 실물 D435 노출·화이트밸런스 미검증",
                   "점멸별 켜짐·꺼짐 시각은 결과 파일에 없음 (프레임 수만 기록)"],
    }

    groups = assign_locations(seq, m["barcode"])
    wm = WorldModel()
    for p in REGISTRY:
        wm.observe(p["id"], p["color"], p["shape"], 1.0, 1.0, box=p["box"], pos_mm=tuple(p["start_mm"][:2]))
        wm.pucks[p["id"]].status = PuckStatus.IN_BOX
    locs = yaml.safe_load((repo / "ros2_ws/src/planning/nz2_mission/config/mission_locations.yaml").read_text())["locations"]
    cost = lambda pk, L: math.dist(pk.pos_mm, locs[L]["center"])
    al = allocate(groups, wm, cost)
    if al is None:
        raise SystemExit("할당 불가 → ERROR_STOP")
    by = {p["id"]: p for p in REGISTRY}
    places, used = [], []
    for g in groups:
        L = g.location
        ids = sorted(pid for pid, loc in al.by_pid.items() if loc == L)
        cells = place_order(g.count, locs[L]["approach"][2], locs[L]["place_order"])
        tg = [{"puck_id": pid, "cell_col_row": list(c), "xyz": [*cell_world_mm(locs[L]["center"], c), 70]}
              for pid, c in zip(ids, cells)]
        cond = {"type": "shape", "shape": by[ids[0]]["shape"]} if g.same_shape else {"type": "color", "color": g.color}
        ok = len(ids) == g.count and (len({by[i]["shape"] for i in ids}) == 1 if g.same_shape
                                      else all(by[i]["color"] == g.color for i in ids))
        places.append({"place": L, "tag_id": {"L1": 12, "L2": 13, "L3": 14}[L], "center_mm": locs[L]["center"],
                       "approach": locs[L]["approach"], "group": g.group, "n": g.count, "condition": cond,
                       "puck_ids": ids, "place_order_and_targets_mm": tg, "condition_met": ok})
        used += ids
    alloc = {
        "run_id": signal["run_id"], "env": "소프트웨어 계산 (팀 nz2_mission 코드, ROS 2 미사용)",
        "input": {"sequence": seq, "counts": m["counts"], "barcode": m["barcode"], "order": m["direction"],
                  "tie_rule": "R>Y>G", "signal_source": "signal.json"},
        "registry": REGISTRY, "registry_note": "팀 확정 등록표 (2026-11-17)",
        "algorithm": "nz2_mission.core.allocator.allocate — Y 모양 조합 전수 + R/G 거리 비용 최소",
        "cost": round(al.cost, 1), "places": places,
        "validation": {"all_12_used": sorted(used) == list(range(12)),
                       "duplicates": sorted({i for i in used if used.count(i) > 1}),
                       "missing": sorted(set(range(12)) - set(used)),
                       "conditions_met": {p["place"]: p["condition_met"] for p in places}},
        "status": "OK", "error": None,
    }
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    (out / "signal.json").write_text(json.dumps(signal, ensure_ascii=False, indent=2))
    (out / "allocation.json").write_text(json.dumps(alloc, ensure_ascii=False, indent=2))
    print(seq, m["counts"], m["barcode"])
    for p in places:
        print(p["place"], p["group"], p["puck_ids"], [t["xyz"][:2] for t in p["place_order_and_targets_mm"]])
    print(alloc["validation"])


if __name__ == "__main__":
    main()
