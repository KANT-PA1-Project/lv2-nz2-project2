#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nz2_make_armonly_usd.py — 학습 USD 에서 차체(base_link 몸체·바퀴·조향·센서 마운트)를 지우고 '팔만' 남긴 사본을 만든다

왜: 사용자 결정(2026-11-10) "차체를 없애고 로봇팔만 바닥에 고정한 채 학습".
    URDF 를 새로 가져오기(GUI 변환 + add_physx_mimic) 하지 않고, 검증된 USD 위에 'over' 만 덧씌운다.
    → 그리퍼 mimic·관절 설정은 원본 그대로. 원본 USD 는 건드리지 않음.
방법: base_link 의 자식 중 arm_link1 을 뺀 모든 자식(차체 메시·충돌상자·바퀴·조향·센서 프레임)을
      보이지 않게(invisible) + 충돌 끔(collisionEnabled = false). base_link 자체는 관절 뿌리라 남긴다.
      arm_joint1 이 base_link 어디에 붙어 있는지(localPos0)를 읽어 json 의 arm_j1_xyz 로 저장
      → 환경 설정이 이 값만큼 뿌리를 옮겨 팔 베이스를 바닥 원점에 둔다 (새 URDF 에서 팔이 앞으로 나와도 자동).

실행 위치: [개발 PC]
    ~/isaacsim/python.sh ~/manipulator-isaac61-lyrical-practice/scripts/nz2_make_armonly_usd.py \
        ~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD/urdf/r12_arm_train_pos/r12_arm_train/r12_arm_train.usda
    (새 URDF 로 USD 를 다시 만들면 그 .usda 를 넣고 다시 실행)
결과
    <원본>_armonly.usda                       (원본 sublayer + over)
    <R12 폴더>/isaac/nz2_arm_armonly.json     (nz2_arm.json 의 usd_path 만 바꾼 사본)
검사: 숨긴 충돌 prim 수, 남은 충돌 prim 이 모두 arm_link1 아래인지 출력. 남은 차체 충돌이 있으면 종료 코드 1
"""
import json
import os
import sys

_APP = None
try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:                    # Isaac Sim 6.x python.sh: Kit 을 먼저 띄워야 pxr 를 불러올 수 있음
    from isaacsim import SimulationApp
    _APP = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

ARM_ROOT_NAME = "arm_link1"


def find_base_link(stage):
    for p in stage.Traverse():
        if p.GetName() == "base_link" and p.GetParent() and p.GetParent().GetName() == "Geometry":
            return p
    return None


def uninstance_to(stage, prim):
    """인스턴스 안 prim 을 고치려면 인스턴스 루트의 instanceable 을 꺼야 함"""
    root = prim
    while root and not root.IsInstance():
        root = root.GetParent()
    if root and root.IsValid():
        stage.OverridePrim(root.GetPath()).SetInstanceable(False)
        return True
    return False


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    src = os.path.abspath(os.path.expanduser(sys.argv[1]))
    if "_armonly" in os.path.basename(src):
        print("이미 _armonly 파일입니다. 원본 .usda 를 넣으세요")
        sys.exit(2)
    stem, ext = os.path.splitext(src)
    dst = stem + "_armonly" + ext
    if os.path.exists(dst):
        os.remove(dst)
    s0 = Usd.Stage.Open(src)
    layer = Sdf.Layer.CreateNew(dst)
    layer.subLayerPaths.append("./" + os.path.basename(src))
    layer.defaultPrim = s0.GetRootLayer().defaultPrim
    layer.Save()
    stage = Usd.Stage.Open(dst)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(s0))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(s0))
    stage.SetEditTarget(stage.GetRootLayer())

    base = find_base_link(stage)
    if base is None:
        os.remove(dst)
        print("[실패] Geometry/base_link 를 못 찾음")
        sys.exit(1)
    arm_prefix = str(base.GetPath().AppendChild(ARM_ROOT_NAME))
    if not stage.GetPrimAtPath(arm_prefix):
        os.remove(dst)
        print(f"[실패] {arm_prefix} 없음")
        sys.exit(1)

    # 1) arm_link1 이 아닌 base_link 자식: 숨김
    hidden = []
    for c in base.GetChildren():
        if c.GetName() == ARM_ROOT_NAME:
            continue
        UsdGeom.Imageable(stage.OverridePrim(c.GetPath())).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        hidden.append(c.GetName())
    # 2) arm 밖 충돌 prim: 충돌 끔 (인스턴스 안이면 인스턴스 해제 후 다시 찾음)
    def cols():
        it = Usd.PrimRange(base, Usd.TraverseInstanceProxies())
        return [p for p in it if p.HasAPI(UsdPhysics.CollisionAPI)]
    for p in cols():
        if str(p.GetPath()).startswith(arm_prefix):
            continue
        if p.IsInstanceProxy():
            uninstance_to(stage, p)
    off = 0
    for p in cols():
        if str(p.GetPath()).startswith(arm_prefix):
            continue
        UsdPhysics.CollisionAPI(stage.OverridePrim(p.GetPath())).CreateCollisionEnabledAttr().Set(False)
        off += 1
    stage.GetRootLayer().Save()

    # 3) 검사: 켜진 충돌은 모두 팔 아래여야 함
    stage = Usd.Stage.Open(dst)
    base = find_base_link(stage)
    left = []
    for p in Usd.PrimRange(base, Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        en = UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
        if en is not False and not str(p.GetPath()).startswith(arm_prefix):
            left.append(str(p.GetPath()))
    arm_cols = sum(1 for p in Usd.PrimRange(stage.GetPrimAtPath(arm_prefix), Usd.TraverseInstanceProxies())
                   if p.HasAPI(UsdPhysics.CollisionAPI))
    print(f"숨긴 base_link 자식 {len(hidden)}개: {hidden[:12]}{' …' if len(hidden) > 12 else ''}")
    print(f"충돌 끈 차체 prim {off}개, 남은 팔 충돌 prim {arm_cols}개")
    if left:
        print(f"[실패] 차체 충돌이 남음 {len(left)}개 예: {left[:5]}")
        sys.exit(1)
    print(f"저장: {dst}")

    # 4) arm_joint1 위치 (base_link 기준)
    j1 = None
    for p in stage.Traverse():
        if p.GetName() == "arm_joint1" and p.HasAttribute("physics:localPos0"):
            v = p.GetAttribute("physics:localPos0").Get()
            j1 = [round(float(v[0]), 6), round(float(v[1]), 6), round(float(v[2]), 6)]
            break
    if j1 is None:
        print("[실패] arm_joint1 의 physics:localPos0 를 못 찾음")
        sys.exit(1)
    print(f"arm_joint1 위치 (base_link 기준) = {j1} m")

    root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(dst))))   # R12 폴더
    js = os.path.join(root_dir, "isaac", "nz2_arm.json")
    if os.path.exists(js):
        c = json.load(open(js, encoding="utf-8"))
        c["usd_path"] = os.path.relpath(dst, root_dir)
        c["arm_j1_xyz"] = j1
        out = os.path.join(root_dir, "isaac", "nz2_arm_armonly.json")
        json.dump(c, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
        print(f"저장: {out}  (usd_path = {c['usd_path']})")
    else:
        print(f"[주의] {js} 없음 → nz2_arm.json 사본의 usd_path 를 {dst} 로 직접 바꾸세요")


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
