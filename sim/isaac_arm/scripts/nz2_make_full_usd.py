#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nz2_make_full_usd.py — 최종 URDF 로봇(assembly.usd, swerve_manipulator)을 Isaac Lab 학습용으로 고치는 덮어쓰기 레이어 생성

위치: ~/manipulator-isaac61-lyrical-practice/scripts/nz2_make_full_usd.py
실행 위치: [개발 PC] 또는 [클라우드] — pxr(usd-core 또는 Isaac Sim python) 만 있으면 됨
    ~/isaacsim/python.sh ~/manipulator-isaac61-lyrical-practice/scripts/nz2_make_full_usd.py \
        ~/Downloads/assembly_URDF/assembly.usd ~/manipulator-isaac61-lyrical-practice/models/nz2_full

원본 assembly.usd 는 건드리지 않고, 그것을 sublayer 로 깔고 'over' 만 덧씌운다.
  1) 질량·관성 × 0.1  (CAD 42.96 kg → 실제 4.296 kg, 사용자 확인 11/12)
  2) 팔 관절 5개 부호를 R12(펌웨어) 규약으로 뒤집음: 관절 프레임 두 개를 관절 축에 수직인 z 로 180° 돌림
       → q_USD = q_R12 (영점 동일). 클라우드 FK 대조: 무작위 200 자세 TCP 오차 0.0 mm, 집게 축 방향 일치
  3) 관절 한계 = 펌웨어/R12 값 (j1 ±175°, j2 −95~113°, j3 −111~82°, j4 −99~117°, j5 ±170°)
  4) 집게: jaw_left·jaw_right 모두 '+ = 벌림'(0 ~ 0.027686 m)으로 뒤집음. 오른쪽은 행동 클래스가 왼쪽 목표를 복사해 구동
     충돌 상자는 실제 손가락판·캐리지 치수로 교체
  5) 뿌리 고정 관절 root_joint (/Robot → base_link) 추가 (R12 USD 와 같은 방식, fix_root_link 대신)
  6) _light 판: 스워브 모듈 시각 메시(로봇당 약 370만 면)를 비활성 → 환경 수십 개 복제 시 메모리 절약 (충돌은 유지)
결과
  <출력폴더>/nz2_full_robot.usda        (시각 메시 전부, GUI 확인용)
  <출력폴더>/nz2_full_robot_light.usda  (학습·expert-check 기본)
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# ===== 상수 (한곳에서 수정) =====
MASS_SCALE = 0.1
ARM_JOINTS = ["arm_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
# R12 / 펌웨어 한계 [rad] (nz2_arm_armonly.json 과 같음)
ARM_LOWER = [-3.0543, -1.6581, -1.9373, -1.7279, -2.9671]
ARM_UPPER = [3.0543, 1.9722, 1.4312, 2.042, 2.9671]
JAW_DRIVER, JAW_FOLLOWER = "jaw_left_joint", "jaw_right_joint"
JAW_OPEN_M = 0.027686130404472348
MIMIC_INSTANCE = "rotX"          # 단일 자유도 관절: 형식상 값 (R12 에서 프리즘 관절에 같은 값으로 동작 확인)
# 집게 충돌 상자 (jaw_left 몸체 좌표 [m]: 중심, 크기). jaw_left_0 메시 단면 (영점, 몸체 원점 = wrist_roll + (−0.085, −0.0189, 0))
#   손가락판: 끝 쪽 56 mm, 두께 18.5 mm (안쪽 면 = 영점에서 중심선 9.5 mm → 패드 간격 19 mm), 폭 33.5 mm
#   캐리지: 손목 쪽 32 mm, 두께 36.7 mm, 폭 58 mm (안쪽 끝을 중심선 2 mm 밖으로 → 좌우 캐리지 겹침 방지)
JAW_FINGER_BOX = ((-0.01181, 0.00012, -0.00015), (0.056, 0.0185, 0.0335))
JAW_CARRIAGE_BOX = ((0.03219, -0.00148, 0.0001), (0.032, 0.0367, 0.058))
STEER_BODIES =["rear_left_steer", "front_right_steer", "front_left_steer", "rear_right_steer"]
ROBOT = "/Robot"
JOINTS = ROBOT + "/joints"


def qmul(a: Gf.Quatf, b: Gf.Quatf) -> Gf.Quatf:
    return Gf.Quatf((Gf.Quatd(a) * Gf.Quatd(b)).GetNormalized())


QZ180 = Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.0, 1.0))      # z 축 180° (w=0)


def build(src: str, dst: str, light: bool):
    if os.path.exists(dst):
        os.remove(dst)
    base = Usd.Stage.Open(src)
    layer = Sdf.Layer.CreateNew(dst)
    layer.subLayerPaths.append(src)
    layer.defaultPrim = base.GetRootLayer().defaultPrim or "Robot"
    layer.Save()
    st = Usd.Stage.Open(dst)
    st.SetEditTarget(st.GetRootLayer())
    UsdGeom.SetStageUpAxis(st, UsdGeom.GetStageUpAxis(base))
    UsdGeom.SetStageMetersPerUnit(st, UsdGeom.GetStageMetersPerUnit(base))

    # 1) 질량·관성
    total = 0.0
    for p in st.Traverse():
        if p.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(p)
            mv = m.GetMassAttr().Get()
            if mv:
                m.GetMassAttr().Set(float(mv) * MASS_SCALE)
                total += float(mv) * MASS_SCALE
            di = m.GetDiagonalInertiaAttr().Get()
            if di is not None and any(abs(x) > 0 for x in di):
                m.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(x) * MASS_SCALE for x in di]))

    # 2)·3) 팔 관절 부호·한계
    for name, lo, hi in zip(ARM_JOINTS, ARM_LOWER, ARM_UPPER):
        j = st.GetPrimAtPath(f"{JOINTS}/{name}")
        if not j:
            raise RuntimeError(f"관절 없음: {JOINTS}/{name}")
        for a in ("physics:localRot0", "physics:localRot1"):
            j.GetAttribute(a).Set(qmul(j.GetAttribute(a).Get(), QZ180))
        j.GetAttribute("physics:lowerLimit").Set(math.degrees(lo))
        j.GetAttribute("physics:upperLimit").Set(math.degrees(hi))

    # 4) 집게
    for name in (JAW_DRIVER, JAW_FOLLOWER):
        j = st.GetPrimAtPath(f"{JOINTS}/{name}")
        for a in ("physics:localRot0", "physics:localRot1"):
            j.GetAttribute(a).Set(qmul(j.GetAttribute(a).Get(), QZ180))
        j.GetAttribute("physics:lowerLimit").Set(0.0)
        j.GetAttribute("physics:upperLimit").Set(JAW_OPEN_M)
    # (11/12 v3) PhysX mimic 대신 오른쪽 집게도 위치 구동 (목표 = 왼쪽 목표, nz2_smooth_action.apply_actions)
    #   v2 실험(060): mimic 설정에서 집기 0.4 % — 오른쪽 손가락이 따라오지 않은 것으로 판단
    # 4-b) 집게 충돌형상: CAD 거친 상자(87×39×58 mm, 열면 바깥면 ±67 mm → 옆 퍽을 침, 057 실험) 를
    #      실제 메시 단면에서 잰 손가락판 + 캐리지 상자 2개로 교체 (jaw 몸체 좌표, 오른쪽은 y 대칭)
    for body, sy in (("jaw_left", 1.0), ("jaw_right", -1.0)):
        col = st.GetPrimAtPath(f"{ROBOT}/{body}/collision")
        (fx, fy, fz), fsz = JAW_FINGER_BOX
        col.GetAttribute("xformOp:translate").Set(Gf.Vec3d(fx, sy * fy, fz))
        col.GetAttribute("xformOp:scale").Set(Gf.Vec3f(*fsz))
        car = UsdGeom.Cube.Define(st, f"{ROBOT}/{body}/collision_carriage")
        car.CreateSizeAttr(1.0)
        car.CreatePurposeAttr(UsdGeom.Tokens.guide)
        (cx, cy, cz), csz = JAW_CARRIAGE_BOX
        car.AddTranslateOp().Set(Gf.Vec3d(cx, sy * cy, cz))
        car.AddScaleOp().Set(Gf.Vec3f(*csz))
        UsdPhysics.CollisionAPI.Apply(car.GetPrim())

    # 5) 뿌리 고정
    rj = UsdPhysics.FixedJoint.Define(st, f"{JOINTS}/root_joint")
    rj.CreateBody0Rel().SetTargets([Sdf.Path(ROBOT)])
    rj.CreateBody1Rel().SetTargets([Sdf.Path(ROBOT + "/base_link")])
    rj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    rj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    rj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    rj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # 6) 가벼운 판
    hidden = 0
    if light:
        for b in STEER_BODIES:
            body = st.GetPrimAtPath(f"{ROBOT}/{b}")
            for c in body.GetChildren():
                if c.GetTypeName() == "Mesh" and c.GetName().startswith("visual"):
                    c.SetActive(False)
                    hidden += 1
    st.GetRootLayer().customLayerData = {"nz2_note": "assembly.usd over: mass x0.1, R12 joint signs, firmware limits, jaw mirror drive, root_joint",
                                         "nz2_light": light}
    st.GetRootLayer().Save()
    print(f"[NZ2-USD] {dst}: 총질량 {total:.3f} kg, 팔 관절 부호 뒤집음 {len(ARM_JOINTS)}, 숨긴 시각 메시 {hidden}")
    return total


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    src = os.path.abspath(os.path.expanduser(sys.argv[1]))
    out = os.path.abspath(os.path.expanduser(sys.argv[2]))
    os.makedirs(out, exist_ok=True)
    for light, name in ((False, "nz2_full_robot.usda"), (True, "nz2_full_robot_light.usda")):
        t = build(src, os.path.join(out, name), light)
        if not 4.0 < t < 4.6:
            print(f"[NZ2-USD] 경고: 총질량 {t:.3f} kg (기대 4.296)")
            sys.exit(1)


if __name__ == "__main__":
    main()
