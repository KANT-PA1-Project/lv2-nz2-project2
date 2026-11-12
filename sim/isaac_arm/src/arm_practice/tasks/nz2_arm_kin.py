"""NZ2 팔 기구학 (순수 torch, Isaac 의존 없음) — 해석적 FK / IK / 하향각 자동 선택 / 충돌 근사 검사

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_arm_kin.py
치수 출처: URDF 자동 읽기 (load_urdf_dims, 2026-11-10) — 아래는 r12_arm_train.urdf (2026-11-03 v8) 값이자 URDF 를 못 찾을 때 기본값
  arm_joint1  base_link → (0, 0, 0.275)       축 +Z (yaw)
  arm_joint2  arm_link1 → (0, 0, 0.0595)      축 +Y (양수 = 아래로 숙임)
  arm_joint3  arm_link2 → (0.024, 0, 0.1854)  축 +Y
  arm_joint4  arm_link3 → (0.149, 0, 0)       축 +Y
  arm_joint5  arm_link4 → (0.06635, 0, 0)     축 +X (손목 롤, TCP 는 롤 축 위)
  TCP         arm_link5 + (0.105, 0, 0)
  손가락 끝    TCP + 0.020 (도구축 방향), 손가락 두께 ±0.013 (link5 z 방향)
좌표: base_link 원점 = 차체 중심 바닥. r = 수평 거리, z = 높이, P = 도구 하향각(0 = 수평, π/2 = 수직 아래)

모든 함수는 배치 텐서 (N,) 를 받는다. 실기 arm_ik_planner(KDL)와 같은 기하이므로 결과를 실기 계획 검증에도 쓸 수 있다.
"""
from __future__ import annotations

import math
import os

import torch

# ===== 기구 치수 [m] =====
# 팔 베이스(arm_joint1) 높이 [m]: 차체 장착 0.275(기본, 11/9 실험) / 바닥 고정 환경은 NZ2_ARM_Z 와 같게 NZ2_KIN_J1_Z=0.0
J1_Z = float(os.environ.get("NZ2_KIN_J1_Z", "0.275"))
# ----- 아래 치수·한계는 '기본값'. 모듈을 불러올 때 URDF·ARM_CONFIG 에서 다시 읽어 덮어쓴다 (load_urdf_dims) -----
SHOULDER_DZ = 0.0595                  # arm_joint1 → arm_joint2 높이
L2_A, L2_B = 0.024, 0.1854            # j2 → j3 (x, z)
L3 = 0.149                            # j3 → j4
J4_TO_J5 = 0.06635                    # j4 → j5 (손목 롤 축)
TCP_X = 0.105                         # j5(arm_link5) → TCP (gripper_tcp)
Q_LOWER = (-3.0543, -1.6581, -1.9373, -1.7279, -2.9671)
Q_UPPER = (3.0543, 1.9722, 1.4312, 2.042, 2.9671)
DIMS_SOURCE = "기본값(코드)"
TIP_BEYOND_TCP = 0.020                # [URDF] 손가락 끝 상자 x 0.1248 − TCP 0.105
FINGER_HALF_T = 0.013                 # [URDF] 손가락 끝 상자 z 두께 0.026 / 2
# ===== 차체 앞 충돌 근사 (D435 + 마운트 제거 후, 팔 통로 |y| < 0.06) =====
# (r 시작, r 끝, 윗면 z): URDF 충돌상자 최대 높이. 팔 각 부위는 반지름만큼 떨어져야 함
CHASSIS_KEEPOUT = ((0.10, 0.14, 0.268), (0.14, 0.20, 0.288), (0.20, 0.225, 0.188))
CHASSIS_KEEPOUT_WITH_D435 = ((0.10, 0.12, 0.268), (0.12, 0.14, 0.288), (0.14, 0.20, 0.308), (0.20, 0.225, 0.228))
# 도구축 위 구간 (link5 원점에서 도구축 방향 시작, 끝 [m], 도구축에 수직인 아래쪽 두께 [m])  [URDF 충돌상자]
TOOL_SEGMENTS = ((0.0, 0.036, 0.029),       # arm_link5 몸체 (z −0.029)
                 (0.036, 0.065, 0.029),     # 손가락 뿌리 (z ±0.029)
                 (0.065, 0.090, 0.017),     # 손가락 중간 (z ±0.017)
                 (0.090, 0.125, FINGER_HALF_T))  # 손가락 끝 (z ±0.013), TCP = 0.105
R_LINK3, R_LINK4 = 0.015, 0.012            # link3 (z ±0.014), link4 (아래 −0.011)
LIMIT_MARGIN = math.radians(3.0)

# ---------------------------------------------------------------------------
# URDF 자동 반영 (튜터 r12/src/arm_practice/learned_ik.py 의 ArmFK 방식: URDF 관절 사슬을 직접 읽음)
#   1) URDF 에서 arm_joint2~5 원점·축, gripper_tcp_joint 원점을 읽어 위 치수를 덮어씀 → URDF 가 바뀌면 자동으로 따라감
#   2) ARM_CONFIG json 의 lower/upper 로 관절 한계를 덮어씀
#   3) 해석적 IK 의 가정(팔 평면 구조: j2~j4 축 +Y, j5 축 +X, 모든 rpy = 0, 옆(y) 오프셋 0)을 검사 → 어긋나면 중단
#   4) URDF 사슬로 만든 범용 FK 와 해석 FK 를 무작위 관절 2000개에서 비교 → 0.1 mm 넘게 다르면 중단
#   (Isaac 실행 시에는 행동 클래스가 시뮬 TCP 와 한 번 더 비교: 20 mm 넘으면 중단)
# 환경 변수: NZ2_URDF (기본: ARM_CONFIG 의 R12 폴더/urdf/r12_arm_train.urdf), NZ2_KIN_URDF=0 이면 기본값 사용
# ---------------------------------------------------------------------------
ARM_CHAIN = ("arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5")
TCP_JOINT = "gripper_tcp_joint"
URDF_CHECK_MM = 0.1


def _rot_np(axis, angle):
    import numpy as np
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _rpy_np(rpy):
    return _rot_np((0, 0, 1), rpy[2]) @ _rot_np((0, 1, 0), rpy[1]) @ _rot_np((1, 0, 0), rpy[0])


def read_urdf_chain(urdf_path: str) -> dict:
    """URDF → {관절이름: (xyz, rpy, axis)} (팔 5축 + TCP)"""
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    out = {}
    for j in root.findall("joint"):
        n = j.get("name")
        if n not in ARM_CHAIN and n != TCP_JOINT:
            continue
        o = j.find("origin")
        xyz = tuple(float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split())
        rpy = tuple(float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split())
        a = j.find("axis")
        axis = tuple(float(v) for v in a.get("xyz").split()) if a is not None else None
        out[n] = (xyz, rpy, axis)
    miss = [n for n in (*ARM_CHAIN, TCP_JOINT) if n not in out]
    if miss:
        raise RuntimeError(f"[NZ2-KIN] URDF {urdf_path} 에 관절 없음: {miss}")
    return out


def urdf_fk_np(chain: dict, q):
    """범용 FK (URDF 관절 사슬을 그대로 곱함): q (5,) → TCP 위치, arm_joint1 원점 기준 [m]"""
    import numpy as np
    T = np.eye(4)
    for i, n in enumerate(ARM_CHAIN):
        xyz, rpy, axis = chain[n]
        A = np.eye(4)
        A[:3, :3] = _rpy_np(rpy)
        A[:3, 3] = xyz if i > 0 else (0.0, 0.0, 0.0)          # arm_joint1 원점 = 팔 베이스 (높이는 J1_Z 로 따로)
        R = np.eye(4)
        R[:3, :3] = _rot_np(axis, q[i])
        T = T @ A @ R
    xyz, rpy, _ = chain[TCP_JOINT]
    return (T @ np.array([*xyz, 1.0]))[:3]


def _check_planar(chain: dict):
    """해석적 IK 가 성립하는 구조인지 (어긋나면 해석 IK 로는 못 풂 → 수치 IK 필요)"""
    tol = 1e-6
    bad = []
    expect_axis = {"arm_joint1": (0, 0, 1), "arm_joint2": (0, 1, 0), "arm_joint3": (0, 1, 0), "arm_joint4": (0, 1, 0),
                   "arm_joint5": (1, 0, 0)}
    for n, ax in expect_axis.items():
        xyz, rpy, axis = chain[n]
        if any(abs(v) > tol for v in rpy):
            bad.append(f"{n} rpy={rpy}")
        if axis is None or any(abs(a - b) > tol for a, b in zip(axis, ax)):
            bad.append(f"{n} axis={axis} (기대 {ax})")
        if n != "arm_joint1" and abs(xyz[1]) > tol:
            bad.append(f"{n} y 오프셋 {xyz[1]}")
    if abs(chain["arm_joint2"][0][0]) > tol:
        bad.append(f"arm_joint2 x 오프셋 {chain['arm_joint2'][0][0]}")
    for n in ("arm_joint4", "arm_joint5"):
        if abs(chain[n][0][2]) > tol:
            bad.append(f"{n} z 오프셋 {chain[n][0][2]}")
    txyz, trpy, _ = chain[TCP_JOINT]
    if abs(txyz[1]) > tol or abs(txyz[2]) > tol or any(abs(v) > tol for v in trpy):
        bad.append(f"{TCP_JOINT} xyz={txyz} rpy={trpy} (TCP 는 손목 롤 축 위여야 함)")
    if bad:
        raise RuntimeError("[NZ2-KIN] URDF 가 해석적 IK 가정(팔 평면 구조)과 다름 → 수치 IK(learned_ik 방식) 필요: " + "; ".join(bad))


def _default_urdf() -> str | None:
    p = os.environ.get("NZ2_URDF")
    if p:
        return os.path.expanduser(p)
    cfg = os.environ.get("ARM_CONFIG")
    if cfg:
        cand = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(os.path.expanduser(cfg)))), "urdf", "r12_arm_train.urdf")
        if os.path.isfile(cand):
            return cand
    return None


def load_urdf_dims(urdf_path: str | None = None, config_path: str | None = None, verbose: bool = True) -> dict:
    """URDF·json 에서 치수·한계를 읽어 이 모듈의 상수를 덮어쓰고, 범용 FK 와 비교 검사한다"""
    global SHOULDER_DZ, L2_A, L2_B, L3, J4_TO_J5, TCP_X, Q_LOWER, Q_UPPER, DIMS_SOURCE
    import json
    import numpy as np
    urdf_path = urdf_path or _default_urdf()
    info = {}
    if urdf_path:
        if not os.path.isfile(urdf_path):
            raise RuntimeError(f"[NZ2-KIN] URDF 없음: {urdf_path} (NZ2_URDF 확인)")
        chain = read_urdf_chain(urdf_path)
        _check_planar(chain)
        SHOULDER_DZ = chain["arm_joint2"][0][2]
        L2_A, L2_B = chain["arm_joint3"][0][0], chain["arm_joint3"][0][2]
        L3 = chain["arm_joint4"][0][0]
        J4_TO_J5 = chain["arm_joint5"][0][0]
        TCP_X = chain[TCP_JOINT][0][0]
        DIMS_SOURCE = urdf_path
        info["urdf_j1_xyz"] = chain["arm_joint1"][0]
    cfg = config_path or os.environ.get("ARM_CONFIG")
    if cfg and os.path.isfile(os.path.expanduser(cfg)):
        c = json.load(open(os.path.expanduser(cfg), encoding="utf-8"))
        names = list(c.get("joint_names", []))
        if all(n in names for n in ARM_CHAIN):
            idx = [names.index(n) for n in ARM_CHAIN]
            Q_LOWER = tuple(float(c["lower"][i]) for i in idx)
            Q_UPPER = tuple(float(c["upper"][i]) for i in idx)
        j1 = c.get("arm_j1_xyz")
        if j1 is not None and "urdf_j1_xyz" in info:
            d = max(abs(a - b) for a, b in zip(j1, info["urdf_j1_xyz"]))
            info["usd_vs_urdf_j1_mm"] = round(d * 1000, 2)
            if d > 0.001:
                raise RuntimeError(f"[NZ2-KIN] USD(arm_j1_xyz {j1}) 와 URDF(arm_joint1 {info['urdf_j1_xyz']}) 가 {d * 1000:.1f} mm 다름 "
                                   f"→ USD 를 이 URDF 로 다시 만들었는지 확인 (bash nz2_vispick_run.sh usd), 또는 NZ2_URDF 경로 확인")
    _derive()
    if urdf_path:
        rng = np.random.default_rng(0)
        lo, hi = np.array(Q_LOWER), np.array(Q_UPPER)
        qs = rng.uniform(lo, hi, size=(2000, 5))
        ana = fk_tcp(torch.tensor(qs, dtype=torch.float64)).numpy()
        ana[:, 2] -= J1_Z
        gen = np.stack([urdf_fk_np(chain, q) for q in qs])
        err = float(np.abs(ana - gen).max()) * 1000
        info["fk_vs_urdf_max_mm"] = round(err, 4)
        if err > URDF_CHECK_MM:
            raise RuntimeError(f"[NZ2-KIN] 해석 FK 와 URDF 범용 FK 가 {err:.3f} mm 다름 (> {URDF_CHECK_MM} mm) → nz2_arm_kin 수식 확인")
    info.update(source=DIMS_SOURCE, shoulder_dz=SHOULDER_DZ, l2=(L2_A, L2_B), l3=L3, j4_j5=J4_TO_J5, tcp_x=TCP_X,
                q_lower=Q_LOWER, q_upper=Q_UPPER, j1_z=J1_Z)
    if verbose:
        print(f"[NZ2-KIN] 치수 출처 {DIMS_SOURCE}: 어깨 {SHOULDER_DZ:.4f}, L2 ({L2_A:.4f}, {L2_B:.4f}), L3 {L3:.4f}, "
              f"j4→j5 {J4_TO_J5:.5f}, TCP {TCP_X:.4f} m / URDF FK 대조 최대 {info.get('fk_vs_urdf_max_mm', '-')} mm")
    return info


def _derive():
    """치수 → 파생 상수"""
    global SHOULDER_Z, L2, ALPHA0, WRIST_TO_TCP
    SHOULDER_Z = J1_Z + SHOULDER_DZ
    L2 = math.hypot(L2_A, L2_B)
    ALPHA0 = math.atan2(L2_B, L2_A)
    WRIST_TO_TCP = J4_TO_J5 + TCP_X


_derive()
# ===== 차체 앞 충돌 근사 (D435 + 마운트 제거 후, 팔 통로 |y| < 0.06) =====
# (r 시작, r 끝, 윗면 z): URDF 충돌상자 최대 높이. 팔 각 부위는 반지름만큼 떨어져야 함
CHASSIS_KEEPOUT = ((0.10, 0.14, 0.268), (0.14, 0.20, 0.288), (0.20, 0.225, 0.188))
CHASSIS_KEEPOUT_WITH_D435 = ((0.10, 0.12, 0.268), (0.12, 0.14, 0.288), (0.14, 0.20, 0.308), (0.20, 0.225, 0.228))
# 차체가 바뀌면 환경 변수로 덮어씀 (11/10, R20D): NZ2_CHASSIS_KEEPOUT="r0,r1,top;r0,r1,top" [m]  (위 기본값은 R12)
_CK = os.environ.get("NZ2_CHASSIS_KEEPOUT", "").strip()
if _CK:
    CHASSIS_KEEPOUT = tuple(tuple(float(v) for v in seg.split(",")) for seg in _CK.split(";") if seg.strip())
    if any(len(k) != 3 or k[0] >= k[1] for k in CHASSIS_KEEPOUT):
        raise ValueError(f"NZ2_CHASSIS_KEEPOUT 형식 오류 (r0<r1, 'r0,r1,top;…'): {_CK}")
    if J1_Z > 0.1 and any(k[2] >= J1_Z for k in CHASSIS_KEEPOUT):
        print(f"[NZ2-KIN] 경고: 차체 충돌 윗면이 팔 베이스(J1_Z {J1_Z}) 이상 → 계획이 대부분 막힐 수 있음: {CHASSIS_KEEPOUT}")
    print(f"[NZ2-KIN] 차체 충돌 근사 (NZ2_CHASSIS_KEEPOUT): {CHASSIS_KEEPOUT}")
# 도구축 위 구간 (link5 원점에서 도구축 방향 시작, 끝 [m], 도구축에 수직인 아래쪽 두께 [m])  [URDF 충돌상자]
TOOL_SEGMENTS = ((0.0, 0.036, 0.029),       # arm_link5 몸체 (z −0.029)
                 (0.036, 0.065, 0.029),     # 손가락 뿌리 (z ±0.029)
                 (0.065, 0.090, 0.017),     # 손가락 중간 (z ±0.017)
                 (0.090, 0.125, FINGER_HALF_T))  # 손가락 끝 (z ±0.013), TCP = 0.105
R_LINK3, R_LINK4 = 0.015, 0.012            # link3 (z ±0.014), link4 (아래 −0.011)


def _rot_pitch(x: torch.Tensor, z: torch.Tensor, th: torch.Tensor):
    """+Y 축 회전(양수 = 아래로): (x, z) → (x cos + z sin, −x sin + z cos)"""
    c, s = torch.cos(th), torch.sin(th)
    return x * c + z * s, -x * s + z * c


def planar_points(q2, q3, q4):
    """팔 평면 위 주요 점 (r, z) 들: 어깨 S, 팔꿈치 J3, 손목 W, TCP, 손가락 끝"""
    zero = torch.zeros_like(q2)
    s = (zero, zero + SHOULDER_Z)
    ax, az = _rot_pitch(zero + L2_A, zero + L2_B, q2)
    j3 = (s[0] + ax, s[1] + az)
    a23 = q2 + q3
    w = (j3[0] + L3 * torch.cos(a23), j3[1] - L3 * torch.sin(a23))
    p = a23 + q4
    tcp = (w[0] + WRIST_TO_TCP * torch.cos(p), w[1] - WRIST_TO_TCP * torch.sin(p))
    tip = (tcp[0] + TIP_BEYOND_TCP * torch.cos(p), tcp[1] - TIP_BEYOND_TCP * torch.sin(p))
    return {"S": s, "J3": j3, "W": w, "TCP": tcp, "TIP": tip, "P": p}


def fk_tcp(q: torch.Tensor) -> torch.Tensor:
    """q (N,5) → TCP 위치 (N,3) [base_link]"""
    pts = planar_points(q[:, 1], q[:, 2], q[:, 3])
    r, z = pts["TCP"]
    return torch.stack([r * torch.cos(q[:, 0]), r * torch.sin(q[:, 0]), z], dim=1)


def fk_link5_origin(q: torch.Tensor) -> torch.Tensor:
    """arm_link5 원점 (= j5) 위치 (N,3). 시뮬 body_pos 와 비교해 부호·치수 자기검사용"""
    pts = planar_points(q[:, 1], q[:, 2], q[:, 3])
    p = pts["P"]
    r = pts["W"][0] + J4_TO_J5 * torch.cos(p)
    z = pts["W"][1] - J4_TO_J5 * torch.sin(p)
    return torch.stack([r * torch.cos(q[:, 0]), r * torch.sin(q[:, 0]), z], dim=1)


def ik(pos: torch.Tensor, pitch: torch.Tensor, roll: torch.Tensor | None = None):
    """TCP 위치 (N,3) + 하향각 P (N,) → q (N,5), ok (N,) bool (도달·관절한계)
    팔꿈치 위(elbow-up) 해. j5 = roll (기본 0)"""
    x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]
    q1 = torch.atan2(y, x)
    r = torch.sqrt(x * x + y * y)
    wr = r - WRIST_TO_TCP * torch.cos(pitch)
    wz = z + WRIST_TO_TCP * torch.sin(pitch)
    dx, dz = wr, wz - SHOULDER_Z
    d2 = dx * dx + dz * dz
    c = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    reach_ok = c.abs() <= 1.0
    c = c.clamp(-1.0, 1.0)
    delta = -torch.acos(c)                                   # 팔꿈치 위: link3 이 link2 보다 아래로 꺾임
    q3 = -delta - ALPHA0
    beta = torch.atan2(dz, dx) - torch.atan2(L3 * torch.sin(delta), L2 + L3 * torch.cos(delta))
    q2 = ALPHA0 - beta
    q4 = pitch - q2 - q3
    q5 = torch.zeros_like(q1) if roll is None else roll
    q = torch.stack([q1, q2, q3, q4, q5], dim=1)
    q = torch.atan2(torch.sin(q), torch.cos(q))              # (−π, π] 로 정리
    lo = torch.tensor(Q_LOWER, device=q.device) + LIMIT_MARGIN
    hi = torch.tensor(Q_UPPER, device=q.device) - LIMIT_MARGIN
    lim_ok = ((q >= lo) & (q <= hi)).all(dim=1)
    return q, reach_ok & lim_ok


def _seg_samples(a, b, n=6):
    t = torch.linspace(0.0, 1.0, n, device=a[0].device).view(1, n)
    return a[0].unsqueeze(1) + (b[0] - a[0]).unsqueeze(1) * t, a[1].unsqueeze(1) + (b[1] - a[1]).unsqueeze(1) * t


def clearance(q: torch.Tensor, box_near: torch.Tensor, box_far: torch.Tensor, box_top: float,
              keepout=CHASSIS_KEEPOUT, margin: float = 0.005):
    """충돌 근사 여유 [m] (N,) — 음수면 충돌 예상.
    팔 평면(r, z) 위에서 link3·link4·link5·손가락 구간의 아래쪽 면이 차체 앞 상자와 선반 윗면보다 위에 있는지 본다.
    각 구간은 기울기에 따라 아래쪽 두께가 rad·cos(기울기) 만큼 내려온다.
    j1 이 작을 때(|y| ≲ 0.1 m) 유효한 근사. 최종 판정은 시뮬 접촉 센서가 한다."""
    pts = planar_points(q[:, 1], q[:, 2], q[:, 3])
    a23, p = q[:, 1] + q[:, 2], pts["P"]
    w = pts["W"]
    j5 = (w[0] + J4_TO_J5 * torch.cos(p), w[1] - J4_TO_J5 * torch.sin(p))
    segs = [(pts["J3"], w, R_LINK3, a23), (w, j5, R_LINK4, p)]
    for s0, s1, rad in TOOL_SEGMENTS:
        a = (j5[0] + s0 * torch.cos(p), j5[1] - s0 * torch.sin(p))
        b = (j5[0] + s1 * torch.cos(p), j5[1] - s1 * torch.sin(p))
        segs.append((a, b, rad, p))
    worst = torch.full_like(q[:, 0], 1.0)
    for a, b, rad, ang in segs:
        rr, zz = _seg_samples(a, b)
        down = (rad * torch.cos(ang).abs()).unsqueeze(1)      # 아래쪽으로 내려오는 두께
        side = (rad * torch.sin(ang).abs()).unsqueeze(1)      # 수평으로 퍼지는 두께
        low = zz - down
        for r0, r1, top in keepout:
            inside = (rr + side >= r0) & (rr - side <= r1)
            gap = torch.where(inside, low - top, torch.full_like(zz, 1.0))
            worst = torch.minimum(worst, gap.amin(dim=1))
        over_box = (rr + side >= box_near.unsqueeze(1)) & (rr - side <= box_far.unsqueeze(1))
        gap = torch.where(over_box, low - box_top, torch.full_like(zz, 1.0))
        worst = torch.minimum(worst, gap.amin(dim=1))
    return worst - margin


def choose_pitch(pos: torch.Tensor, box_near: torch.Tensor, box_far: torch.Tensor, box_top: float,
                 pitch_max_deg: float = 90.0, pitch_min_deg: float = 0.0, step_deg: float = 5.0,
                 keepout=CHASSIS_KEEPOUT):
    """가장 수직에 가까운(위에서 잡는) 하향각부터 내려가며 IK·한계·충돌근사를 통과하는 첫 값을 고른다.
    반환: pitch (N,) [rad], ok (N,) bool. 실패한 환경은 pitch = NaN"""
    n = pos.shape[0]
    out = torch.full((n,), float("nan"), device=pos.device)
    found = torch.zeros(n, dtype=torch.bool, device=pos.device)
    deg = pitch_max_deg
    while deg >= pitch_min_deg - 1e-6:
        p = torch.full((n,), math.radians(deg), device=pos.device)
        q, ok = ik(pos, p)
        ok = ok & (clearance(q, box_near, box_far, box_top, keepout) >= 0.0)
        new = ok & ~found
        out[new] = p[new]
        found |= ok
        deg -= step_deg
    return out, found


def tool_dir(q1: torch.Tensor, pitch: torch.Tensor) -> torch.Tensor:
    """도구축 단위벡터 (N,3): 수평 성분은 j1 방향, 아래로 pitch"""
    return torch.stack([torch.cos(pitch) * torch.cos(q1), torch.cos(pitch) * torch.sin(q1), -torch.sin(pitch)], dim=1)


def plan_grasp(puck: torch.Tensor, grasp_z: torch.Tensor, pitch: torch.Tensor, box_near: torch.Tensor,
               box_far: torch.Tensor, box_top: float, back_m: float = 0.05, up_m: float = 0.06, lift_m: float = 0.04,
               keepout=CHASSIS_KEEPOUT):
    """집기 경유점 4개 (N,4,3): via(접근 위) → pre(도구축 뒤로 back_m) → grasp → lift(위로 lift_m)
    반환: wps, q_grasp (N,5), ok (N,) — 4점 모두 IK·한계 통과 + grasp·pre 충돌근사 통과"""
    g = torch.stack([puck[:, 0], puck[:, 1], grasp_z], dim=1)
    q1 = torch.atan2(g[:, 1], g[:, 0])
    t = tool_dir(q1, pitch)
    pre = g - back_m * t
    via = pre.clone()
    via[:, 2] += up_m
    lift = g.clone()
    lift[:, 2] += lift_m
    wps = torch.stack([via, pre, g, lift], dim=1)
    ok = torch.ones_like(pitch, dtype=torch.bool)
    q_grasp = None
    for k in range(4):
        q, okk = ik(wps[:, k], pitch)
        ok &= okk
        if k in (1, 2):
            ok &= clearance(q, box_near, box_far, box_top, keepout) >= 0.0
        if k == 2:
            q_grasp = q
    return wps, q_grasp, ok


def choose_plan(puck, grasp_z, box_near, box_far, box_top, pitch_req=None, pitch_max_deg=90.0,
                pitch_min_deg=0.0, step_deg=5.0, keepout=CHASSIS_KEEPOUT):
    """pitch_req (N,) 가 NaN 인 환경은 수직(90°)부터 내려가며 plan_grasp 가 통과하는 첫 하향각을 고른다.
    pitch_req 가 숫자인 환경은 그 각으로만 계획 (충돌근사 실패여도 IK 가 되면 실행해 시뮬로 확인).
    반환: pitch (N,), wps (N,4,3), ok (N,), approx_clear (N,) bool"""
    n = puck.shape[0]
    dev = puck.device
    if pitch_req is None:
        pitch_req = torch.full((n,), float("nan"), device=dev)
    auto = torch.isnan(pitch_req)
    pitch = torch.where(auto, torch.zeros(n, device=dev), pitch_req)
    wps = torch.zeros(n, 4, 3, device=dev)
    ok = torch.zeros(n, dtype=torch.bool, device=dev)
    clear = torch.zeros(n, dtype=torch.bool, device=dev)
    # 고정 각
    if (~auto).any():
        p = pitch_req.clone()
        p[auto] = math.radians(90.0)
        w, _, okc = plan_grasp(puck, grasp_z, p, box_near, box_far, box_top, keepout=keepout)
        ik_ok = torch.ones(n, dtype=torch.bool, device=dev)
        for k in range(4):
            ik_ok &= ik(w[:, k], p)[1]
        m = ~auto
        wps[m], ok[m], clear[m] = w[m], ik_ok[m], okc[m]
    # 자동
    if auto.any():
        found = ~auto
        deg = pitch_max_deg
        while deg >= pitch_min_deg - 1e-6 and not bool(found.all()):
            p = torch.full((n,), math.radians(deg), device=dev)
            w, _, okc = plan_grasp(puck, grasp_z, p, box_near, box_far, box_top, keepout=keepout)
            new = okc & ~found
            pitch[new], wps[new] = p[new], w[new]
            ok[new], clear[new] = True, True
            found |= okc
            deg -= step_deg
        pitch[auto & ~ok] = float("nan")
    return pitch, wps, ok, clear


# 모듈을 불러올 때 URDF 자동 반영 (URDF 를 못 찾으면 기본값 + 경고)
if os.environ.get("NZ2_KIN_URDF", "1") != "0":
    if _default_urdf():
        KIN_INFO = load_urdf_dims()
    else:
        KIN_INFO = {"source": DIMS_SOURCE}
        print("[NZ2-KIN][경고] URDF 를 못 찾아 코드 기본 치수 사용 (NZ2_URDF 또는 ARM_CONFIG 지정 권장)")
else:
    KIN_INFO = {"source": DIMS_SOURCE}
