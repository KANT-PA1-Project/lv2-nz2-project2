"""최종 조립 URDF(assembly_puck_slots) 설정·URDF 정리·순수 numpy 정기구학.

Isaac/ROS 를 import 하지 않는다. prepare.py, plan_poses.py, run.py 가 같이 쓴다.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
PERCEPTION_ROOT = HERE.parent                      # r12/experiments/barcode_signal
def _find_r12():
    """저장소 안(r12/experiments/...)에 있으면 그 저장소의 r12, 아니면 기본 tutor 체크아웃."""
    for parent in HERE.parents:
        if parent.name == 'r12' and (parent / 'src/arm_practice/mission.py').is_file():
            return parent
    return Path('~/git/lv2-nz2-project2/r12').expanduser()


R12 = _find_r12()  # 장면 USD·SignalRecorder 읽기 전용

ARM_JOINTS = ['arm_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
JAW_JOINTS = ['jaw_left_joint', 'jaw_right_joint']   # right 는 URDF 에서 left 의 mimic(배율1)
ARM_LINKS = ['arm_yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'jaw_left', 'jaw_right']
CHASSIS_LINKS = ['base_link'] + [f'{c}_{p}' for c in ('front_left', 'front_right', 'rear_left', 'rear_right')
                                 for p in ('steer', 'wheel')]
WRIST_LINK = 'wrist_roll'
CAMERA_FRAME = 'wrist_d435_color_optical_frame'
MAST_PARENT = 'base_link'
MAST_FRAME = 'mast_d435_color_optical_frame'
# 바퀴 중심 z(-0.003-0.145) - 타이어 반지름 0.033 = -0.181 → base_link 원점은 지면 위 0.181 m.
# README 의 gz 생성 높이 0.182 를 그대로 쓴다(1 mm 낙하 후 정착).
BASE_HEIGHT_M = 0.182


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _vec(value, n, name):
    if (not isinstance(value, list) or len(value) != n
            or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value)):
        raise ValueError(f'{name}: 유한한 {n}개 숫자 배열 필요 (현재 {value!r})')
    return value


def load_config(path, check_poses=True):
    cfg = json.loads(Path(path).read_text(encoding='utf-8'))
    # 저장소 밖 외부 자산은 환경변수로 덮어쓸 수 있다(설정 파일 값은 원래 PC 경로).
    #   NZ2_ASSEMBLY_DIR: 최종 조립 URDF 폴더(assembly.urdf, meshes/, sensors.json)
    #   NZ2_FLOOR_USD   : 바닥 visual USD(scene.usda). 빈 값이면 바닥 교체 안 함
    if os.environ.get('NZ2_ASSEMBLY_DIR'):
        cfg['assembly_dir'] = os.environ['NZ2_ASSEMBLY_DIR']
    if 'NZ2_FLOOR_USD' in os.environ:
        cfg['floor_visual_usd'] = os.environ['NZ2_FLOOR_USD'] or None
    cfg['assembly_dir'] = str(Path(cfg['assembly_dir']).expanduser().resolve())
    urdf = Path(cfg['assembly_dir']) / 'assembly.urdf'
    if not urdf.is_file():
        raise FileNotFoundError(urdf)
    model = Model(urdf)
    lower, upper = arm_limits(cfg, model)
    _vec(cfg['barcode_base_xyz_yaw'], 4, 'barcode_base_xyz_yaw')
    _vec(cfg['tower_base_xyz_yaw'], 4, 'tower_base_xyz_yaw')
    poses = [('barcode_arm_q_rad', cfg['barcode_arm_q_rad'])]
    if not isinstance(cfg['tower_arm_waypoints_rad'], list) or not cfg['tower_arm_waypoints_rad']:
        raise ValueError('tower_arm_waypoints_rad: 5관절 경유점 1개 이상 필요')
    poses += [(f'tower_arm_waypoints_rad[{i}]', q) for i, q in enumerate(cfg['tower_arm_waypoints_rad'])]
    for name, q in poses:
        _vec(q, 5, name)
        if check_poses and any(not lo <= x <= hi for x, lo, hi in zip(q, lower, upper)):
            raise ValueError(f'{name}: URDF 관절 범위 밖')
    from signal_fallback import validate_fallback, wall_budget
    validate_fallback(cfg, lower, upper)
    wall_budget(cfg)
    if cfg.get('tower_scene') is not None:
        tower = cfg['tower_scene']
        _vec(tower['xy_m'], 2, 'tower_scene.xy_m')
        z = tower.get('base_z_m', 0.)
        if isinstance(z, bool) or not isinstance(z, (int, float)) or not math.isfinite(z):
            raise ValueError('tower_scene.base_z_m: finite height required')
    _vec(cfg['barcode']['center_m'], 3, 'barcode.center_m')
    if cfg['barcode']['code'] not in {f'P2-0{i}' for i in range(1, 5)}:
        raise ValueError('barcode.code 는 P2-01..04')
    seq = cfg['sequence']
    if (not isinstance(seq, str) or len(seq) != 12 or set(seq) - set('RYG')
            or any(not 1 <= seq.count(c) <= 6 for c in 'RYG')):
        raise ValueError('sequence: R/Y/G 12회, 색별 1~6회')
    if cfg.get('trial_mode', 'barcode_then_tower') not in ('barcode_then_tower', 'tower_only'):
        raise ValueError('trial_mode 는 barcode_then_tower 또는 tower_only')
    cam = cfg['camera']
    if cam['width'] < 320 or cam['height'] < 240 or not 30 <= cam['hfov_deg'] <= 100:
        raise ValueError('camera 해상도/화각 확인')
    if cfg.get('camera_fps', 30) not in (20, 30) or cfg.get('physics_hz', 960) not in (120, 240, 480, 960):
        raise ValueError('camera_fps 20/30, physics_hz 120/240/480/960')
    for key in ('dome', 'emissive', 'max_seconds', 'prepare_timeout_s', 'barcode_timeout_s',
                'motion_timeout_s', 'arm_command_velocity_rad_s'):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'{key}: 양수 필요')
    drive = cfg['arm_drive']
    for key in ('stiffness_per_deg', 'damping_per_deg', 'max_force_nm'):
        if not drive[key] > 0:
            raise ValueError(f'arm_drive.{key}: 양수 필요')
    motion = cfg['base_motion']
    if not (0 < motion['speed_mps'] <= 0.5 and 0 < motion['yaw_rate_rad_s'] <= 1.5):
        raise ValueError('base_motion: 0<speed_mps<=0.5, 0<yaw_rate_rad_s<=1.5')
    if cfg.get('approach'):
        ap = cfg['approach']
        _vec(ap['start_base_xyz_yaw'], 4, 'approach.start_base_xyz_yaw')
        _vec(ap['start_arm_q_rad'], 5, 'approach.start_arm_q_rad')
        if check_poses and any(not lo <= x <= hi for x, lo, hi in zip(ap['start_arm_q_rad'], lower, upper)):
            raise ValueError('approach.start_arm_q_rad: 관절 범위 밖')
    if cfg.get('stow_arm_after_barcode_q_rad') is not None:
        _vec(cfg['stow_arm_after_barcode_q_rad'], 5, 'stow_arm_after_barcode_q_rad')
        if check_poses and any(not lo <= x <= hi for x, lo, hi in zip(cfg['stow_arm_after_barcode_q_rad'], lower, upper)):
            raise ValueError('stow_arm_after_barcode_q_rad: 관절 범위 밖')
    if cfg.get('barcode_seek') is not None and not isinstance(cfg['barcode_seek'], dict):
        raise ValueError('barcode_seek 는 설정 dict 또는 null')
    if cfg.get('order', 'barcode_first') not in ('barcode_first', 'tower_first'):
        raise ValueError('order 는 barcode_first 또는 tower_first')
    if cfg.get('order') == 'tower_first' and not cfg.get('approach'):
        raise ValueError('order tower_first 는 approach(시작 위치) 필요')
    if cfg.get('signal_camera', 'wrist') not in ('wrist', 'mast'):
        raise ValueError('signal_camera 는 wrist 또는 mast')
    pitch = cfg.get('mast_pitch_down_deg')
    if pitch is not None and not 0 < pitch < 90:
        raise ValueError('mast_pitch_down_deg: 0~90 또는 null(URDF 60°)')
    if cfg['barcode_upscale'] not in (1, 2):
        raise ValueError('barcode_upscale 은 1 또는 2')
    if cfg.get('floor_visual_usd'):
        cfg['floor_visual_usd'] = str(Path(cfg['floor_visual_usd']).expanduser().resolve())
    for file in (PERCEPTION_ROOT / 'scripts/live_mission_optimized.py',
                 R12 / 'models/navigation/mission_scene.usda', R12 / 'src/arm_practice/mission.py'):
        if not file.is_file():
            raise FileNotFoundError(file)
    return cfg, model


def arm_limits(cfg, model):
    """팔 5축 한계. config arm_limits_rad 가 있으면 그것(실측), 없으면 URDF 값."""
    own = cfg.get('arm_limits_rad')
    if not own:
        return model.limits(ARM_JOINTS)
    lower = np.array(_vec(own['lower'], 5, 'arm_limits_rad.lower'), float)
    upper = np.array(_vec(own['upper'], 5, 'arm_limits_rad.upper'), float)
    if np.any(lower >= upper):
        raise ValueError('arm_limits_rad: lower < upper 필요')
    return lower, upper


# ---------------------------------------------------------------- 정기구학
def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF: R = Rz(yaw) Ry(pitch) Rx(roll)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k @ k)


def homogeneous(rot=None, xyz=None):
    t = np.eye(4)
    if rot is not None:
        t[:3, :3] = rot
    if xyz is not None:
        t[:3, 3] = xyz
    return t


def base_transform(xyz_yaw):
    x, y, z, yaw = xyz_yaw
    return homogeneous(rpy_matrix([0, 0, yaw]), [x, y, z])


def _floats(text, n=3):
    return [float(v) for v in (text or ' '.join(['0']*n)).split()]


class Model:
    """URDF 관절 트리. 손목 카메라·관절 위치·질량중심 계산용."""
    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        self.joints = {}
        self.by_child = {}
        for j in root.findall('joint'):
            origin = j.find('origin')
            axis = j.find('axis')
            limit = j.find('limit')
            info = dict(name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                        child=j.find('child').get('link'),
                        origin=homogeneous(rpy_matrix(_floats(origin.get('rpy') if origin is not None else None)),
                                           _floats(origin.get('xyz') if origin is not None else None)),
                        axis=_floats(axis.get('xyz')) if axis is not None else [1, 0, 0],
                        limit=None if limit is None or limit.get('lower') is None else
                        (float(limit.get('lower')), float(limit.get('upper'))))
            self.joints[info['name']] = info
            self.by_child[info['child']] = info
        self.inertial = {}
        for link in root.findall('link'):
            i = link.find('inertial')
            if i is not None and i.find('mass') is not None:
                o = i.find('origin')
                self.inertial[link.get('name')] = (float(i.find('mass').get('value')),
                                                   np.array(_floats(o.get('xyz') if o is not None else None)))

    def limits(self, names):
        lo, hi = zip(*(self.joints[n]['limit'] for n in names))
        return np.array(lo), np.array(hi)

    def chain(self, link):
        out = []
        while link in self.by_child:
            j = self.by_child[link]
            out.append(j)
            link = j['parent']
        return out[::-1]

    def link_transform(self, link, q, base=np.eye(4)):
        """q: {관절이름: 값}. 지정 안 한 가동관절은 0."""
        t = base.copy()
        for j in self.chain(link):
            t = t @ j['origin']
            if j['type'] in ('revolute', 'continuous'):
                t = t @ homogeneous(axis_angle(j['axis'], q.get(j['name'], 0.)))
            elif j['type'] == 'prismatic':
                t = t @ homogeneous(xyz=np.asarray(j['axis']) / np.linalg.norm(j['axis']) * q.get(j['name'], 0.))
        return t

    def static_frame(self, parent, child):
        """parent→child 고정 관절 연쇄 변환(카메라 광학 프레임용)."""
        chain = self.chain(child)
        names = [j['parent'] for j in chain]
        if parent not in names:
            raise ValueError(f'{child} 는 {parent} 아래가 아님')
        t = np.eye(4)
        for j in chain[names.index(parent):]:
            if j['type'] != 'fixed':
                raise ValueError(f'{parent}→{child} 사이에 가동 관절 {j["name"]}')
            t = t @ j['origin']
        return t

    def arm_q(self, q5):
        return dict(zip(ARM_JOINTS, map(float, q5)))

    def camera_world(self, q5, base_xyz_yaw):
        """ROS 광학 프레임(+Z 전방, +X 오른쪽, +Y 아래)의 월드 4x4."""
        wrist = self.link_transform(WRIST_LINK, self.arm_q(q5), base_transform(base_xyz_yaw))
        return wrist @ self.static_frame(WRIST_LINK, CAMERA_FRAME)

    def mast_to_optical(self, pitch_down_deg=None):
        """base_link → 마스트 D435 컬러 광학 프레임. pitch_down_deg 를 주면 하향각만 바꾼다(URDF 60°)."""
        frame = self.joints['mast_d435_color_frame_fixed']['origin'].copy()
        if pitch_down_deg is not None:
            frame[:3, :3] = rpy_matrix([0., math.radians(pitch_down_deg), math.pi])
        return frame @ self.joints['mast_d435_color_optical_frame_fixed']['origin']

    def gravity_torque(self, q5, base_xyz_yaw, g=9.81):
        """정지 자세에서 팔 관절별 중력 토크(N·m). 구동 한계 검토용."""
        q = self.arm_q(q5)
        base = base_transform(base_xyz_yaw)
        torques = []
        for name in ARM_JOINTS:
            j = self.joints[name]
            tj = self.link_transform(j['child'], q, base)
            axis_w = tj[:3, :3] @ (np.asarray(j['axis']) / np.linalg.norm(j['axis']))
            tau = 0.
            for link in ARM_LINKS:
                if name not in [c['name'] for c in self.chain(link)]:
                    continue
                m, com = self.inertial[link]
                tl = self.link_transform(link, q, base)
                r = tl[:3, :3] @ com + tl[:3, 3] - tj[:3, 3]
                tau += float(np.cross(r, [0, 0, -m*g]) @ axis_w)
            torques.append(tau)
        return torques


# ---------------------------------------------------------------- URDF 정리
def sanitize_urdf(source, destination, limits=None):
    """시뮬용 사본: 메시 절대경로, 조향·바퀴 고정, 센서/슬롯 프레임·gazebo 블록 제거.

    센서 프레임은 질량 없는 링크라 Isaac articulation 에 넣지 않고, run.py 가
    wrist_roll 아래에 같은 변환으로 USD 카메라를 직접 붙인다.
    """
    source = Path(source)
    tree = ET.parse(source)
    robot = tree.getroot()
    robot.set('name', 'assembly_perception')
    removed_links = set()
    for j in list(robot.findall('joint')):
        child = j.find('child').get('link')
        if j.get('type') == 'fixed' and (child.startswith(('wrist_d435', 'mast_d435', 'lidar_', 'puck_slot_'))):
            robot.remove(j)
            removed_links.add(child)
        elif j.get('name').endswith(('_steer_joint', '_wheel_joint')):
            j.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                for item in j.findall(tag):
                    j.remove(item)
    for link in list(robot.findall('link')):
        if link.get('name') in removed_links:
            robot.remove(link)
    if limits is not None:  # 실측 한계를 시뮬 관절 한계에도 반영
        for name, lo, hi in zip(ARM_JOINTS, *limits):
            limit = robot.find(f"joint[@name='{name}']/limit")
            limit.set('lower', repr(float(lo)))
            limit.set('upper', repr(float(hi)))
    for g in list(robot.findall('gazebo')):
        robot.remove(g)
    meshes = set()
    for mesh in robot.iter('mesh'):
        name = mesh.get('filename')
        if '://' in name:
            raise ValueError(f'예상 밖 메시 URI: {name}')
        path = (source.parent / name).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', str(path))
        meshes.add(str(path))
    ET.indent(tree)
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return dict(source_urdf_sha256=sha256(source), sim_urdf_sha256=sha256(destination),
                removed_frame_links=sorted(removed_links), mesh_sha256={p: sha256(p) for p in sorted(meshes)},
                chassis_fixed_joints='steer/wheel -> fixed')
