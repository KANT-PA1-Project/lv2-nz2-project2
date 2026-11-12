#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
disable_front_d435.py — 학습 USD 에서 '앞 RealSense D435 + 마운트' 충돌상자만 끈 사본(_noD435.usda)과 json 을 만든다

왜: 사용자 지시 "로봇 앞 D435 는 무시". 팔은 차체 중앙 그대로 두고, 앞 D435 충돌상자 6개
    (URDF chassis_vox_007/017/018/020/021/153: x 0.16~0.22 m, |y| ≤ 0.04 m, z 최대 0.308 m)가
    잡는 높이에서 팔·손가락과 부딪히던 문제(Reach v3 분석: 충돌 없는 자세 0 %)만 없앤다.
    원본 USD 는 건드리지 않는다. 새 파일은 원본을 sublayer 로 깔고 그 위에 'over' 만 기록한다.

실행 위치: [개발 PC]  Isaac Sim 파이썬. pxr 를 바로 못 불러오면 Kit 을 headless 로 띄운 뒤 불러온다(10~30 s)
    cd ~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD
    ~/isaacsim/python.sh scripts/disable_front_d435.py urdf/r12_arm_train_pos/r12_arm_train/r12_arm_train.usda
결과
    urdf/r12_arm_train_pos/r12_arm_train/r12_arm_train_noD435.usda   (원본 + 충돌 끔 over)
    isaac/nz2_arm_noD435.json                                         (usd_path 만 바꾼 nz2_arm.json 사본)
찾는 방법
    1) prim 이름이 chassis_vox_007 … 153 인 것 (URDF 변환기가 collision 이름을 prim 이름으로 씀)
    2) 이름으로 못 찾으면: base_link 아래 충돌 prim 중 월드 경계상자 중심이 URDF 6개 상자 중심과 3 mm 안인 것
    6개를 정확히 못 찾으면 아무것도 저장하지 않고 종료 코드 1 (잘못 끄는 것 방지)
"""
import json
import os
import sys

_APP = None
try:                                   # usd-core 또는 이미 USD 가 올라온 파이썬
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:                    # Isaac Sim 6.x python.sh: pxr 는 Kit 을 띄워야 불러올 수 있음
    from isaacsim import SimulationApp
    _APP = SimulationApp({"headless": True})
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# URDF r12_arm_train.urdf base_link 충돌상자 (이름: 중심 xyz [m], 크기 xyz [m])
D435_BOXES = {
    "chassis_vox_007": ((0.169999, -1e-06, 0.157999), (0.02, 0.08, 0.30)),
    "chassis_vox_017": ((0.209999, -0.030001, 0.147999), (0.02, 0.02, 0.16)),
    "chassis_vox_018": ((0.209999, 0.029999, 0.147999), (0.02, 0.02, 0.16)),
    "chassis_vox_020": ((0.189999, -1e-06, 0.207999), (0.02, 0.08, 0.20)),
    "chassis_vox_021": ((0.209999, -1e-06, 0.177999), (0.02, 0.04, 0.10)),
    "chassis_vox_153": ((0.149999, -1e-06, 0.257999), (0.02, 0.08, 0.06)),
}
TOL_M = 0.003


def collision_prims(stage):
    it = Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
    return [p for p in it if p.HasAPI(UsdPhysics.CollisionAPI)]


def find_targets(stage):
    cols = collision_prims(stage)
    by_name = {}
    for p in cols:
        for name in D435_BOXES:
            if p.GetName() == name or p.GetName().endswith(name):
                by_name.setdefault(name, []).append(p)
    if len(by_name) == len(D435_BOXES) and all(len(v) == 1 for v in by_name.values()):
        return {k: v[0] for k, v in by_name.items()}, "이름"
    # 경계상자 중심으로 찾기
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                                                       UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide])
    found = {}
    for p in cols:
        if "base_link" not in str(p.GetPath()):
            continue
        box = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        c = (box.GetMin() + box.GetMax()) / 2.0
        for name, (ctr, _) in D435_BOXES.items():
            if (Gf.Vec3d(*ctr) - c).GetLength() < TOL_M:
                found.setdefault(name, []).append(p)
    if len(found) == len(D435_BOXES) and all(len(v) == 1 for v in found.values()):
        return {k: v[0] for k, v in found.items()}, "경계상자"
    return None, f"이름 {len(by_name)}/6, 경계상자 {len(found)}/6"


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    src = os.path.abspath(sys.argv[1])
    if "_noD435" in os.path.basename(src):
        print("이미 _noD435 파일입니다. 원본(.usda)을 넣으세요")
        sys.exit(2)
    stem, ext = os.path.splitext(src)
    dst = stem + "_noD435" + ext
    if os.path.exists(dst):            # 다시 실행하면 새로 만든다 (원본은 그대로)
        os.remove(dst)
    src_stage = Usd.Stage.Open(src)
    default_prim = src_stage.GetRootLayer().defaultPrim
    up_axis = UsdGeom.GetStageUpAxis(src_stage)
    mpu = UsdGeom.GetStageMetersPerUnit(src_stage)

    layer = Sdf.Layer.CreateNew(dst)
    layer.subLayerPaths.append("./" + os.path.basename(src))
    layer.defaultPrim = default_prim
    layer.Save()
    stage = Usd.Stage.Open(dst)
    UsdGeom.SetStageUpAxis(stage, up_axis)
    UsdGeom.SetStageMetersPerUnit(stage, mpu)
    stage.SetEditTarget(stage.GetRootLayer())

    targets, how = find_targets(stage)
    if targets is None:
        os.remove(dst)
        print(f"[실패] 앞 D435 충돌상자 6개를 정확히 못 찾음 ({how}). 저장하지 않았습니다.")
        print("  base_link 아래 충돌 prim 예:", [str(p.GetPath()) for p in collision_prims(stage)][:10])
        sys.exit(1)
    for name, p in sorted(targets.items()):
        if p.IsInstanceProxy():
            # 인스턴스 안은 직접 못 고침 → 인스턴스 루트의 instanceable 을 끈다
            root = p
            while root and not root.IsInstance():
                root = root.GetParent()
            if root:
                stage.OverridePrim(root.GetPath()).SetInstanceable(False)
                print(f"  인스턴스 해제: {root.GetPath()}")
            p = stage.GetPrimAtPath(p.GetPath())
        over = stage.OverridePrim(p.GetPath())
        UsdPhysics.CollisionAPI(over).CreateCollisionEnabledAttr().Set(False)
        UsdGeom.Imageable(over).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        print(f"  충돌 끔: {name} → {p.GetPath()}")
    stage.GetRootLayer().Save()
    print(f"저장: {dst}  (찾은 방법: {how})")

    # json 사본
    root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(dst))))   # R12 폴더
    js = os.path.join(root_dir, "isaac", "nz2_arm.json")
    if os.path.exists(js):
        c = json.load(open(js, encoding="utf-8"))
        c["usd_path"] = os.path.relpath(dst, root_dir)
        out = os.path.join(root_dir, "isaac", "nz2_arm_noD435.json")
        json.dump(c, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
        print(f"저장: {out}  (usd_path = {c['usd_path']})")
    else:
        print(f"[주의] {js} 없음 → nz2_arm.json 의 usd_path 를 {dst} 로 바꾼 사본을 직접 만드세요")


if __name__ == "__main__":
    code = 0
    try:
        main()
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 1
    finally:
        if _APP is not None:
            _APP.close()
    sys.exit(code)
