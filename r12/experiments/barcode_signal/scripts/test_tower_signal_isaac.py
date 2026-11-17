"""신호등 색 판정(tower_signal) 단독 Isaac Sim 렌더 시험.

빈 장면에 바닥·조명·실측 치수의 세로 3단 신호등과 카메라 1대만 놓고, 미션 v5 규칙대로
12회 점멸(1 s 켜짐 + 1 s 소등, 끝 3 s 소등)을 렌더한 뒤 매 프레임 tower_signal_color 로 판정하고
SignalRecorder 로 순서·횟수를 기록해 정답과 비교한다.

  - 로봇·주행·R12 미션 런타임·ROS 제어를 쓰지 않는 영상 시험이며 물리 성능 평가는 하지 않는다.
  - 카메라 자세는 시험 조건으로 정한 값이다. --pose-error-m 로 "로봇이 아는 자세"에 일부러 오차를 넣어
    위치 추정 오차에 얼마나 견디는지 본다 (판정에 정답 자세를 쓰는 것이 아니라 오차 민감도 시험)
  - 결과: <out>/summary.json, 색별 첫 켜짐 프레임 사진(칸 표시) <out>/frame_<거리>_<색>.png

실행 위치: Isaac Sim 이 설치된 개발 PC (headless). GPU 여유 1 GB 이상 확인 후.
tutor 는 읽기만 한다 (SignalRecorder 를 r12/src 에서 import). 이 스크립트가 있는 폴더에서:
  # 동작 확인 (1조건 30프레임): 
  /home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --out ~/nz2_runs/tower_signal_smoke --smoke
  # 본 시험:
  /home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py \
      --out ~/nz2_runs/tower_signal_isaac_1 --emissive 0.5 1 2 --distances 0.6 1.0 1.5 2.0
  결과는 조건(발광 × 거리)별 성공·실패 기록이다. 특정 발광값에서 통과한 것은 인식 성능의 증거가 아니며,
  실물 신호등 영상 시험은 별도로 해야 한다.
"""
import argparse
import time
import json
import sys
from pathlib import Path

import numpy as np

import importlib.util  # noqa: E402
import os  # noqa: E402


def _load_dependencies():
    """tutor 를 수정하지 않고 읽어오는 구성.

    - SignalRecorder (arm_practice.mission): tutor src 에서 읽음. 위치 = --r12-src 또는 환경변수 R12_SRC,
      없으면 이 스크립트가 tutor r12/scripts 안에 있을 때의 ../src, 마지막으로 ~/git/lv2-nz2-project2/r12/src
    - tower_signal: tutor 에 이미 있으면 그것, 없으면 이 스크립트 옆 폴더의 ../src/arm_practice/tower_signal.py
    """
    argv_src = None
    if '--r12-src' in sys.argv:
        argv_src = sys.argv[sys.argv.index('--r12-src') + 1]
    here = Path(__file__).resolve().parent
    candidates = [argv_src, os.environ.get('R12_SRC'), here.parent / 'src', Path.home() / 'git/lv2-nz2-project2/r12/src']
    r12_src = next((Path(c).expanduser() for c in candidates if c and (Path(c).expanduser() / 'arm_practice/mission.py').is_file()), None)
    if r12_src is None:
        raise SystemExit('tutor r12/src 를 찾지 못함 (arm_practice/mission.py 필요). --r12-src 또는 R12_SRC 로 지정')
    sys.path.insert(0, str(r12_src))
    from arm_practice.mission import SignalRecorder
    try:
        import arm_practice.tower_signal as ts
        origin = 'tutor'
    except ModuleNotFoundError:
        p = here.parent / 'src/arm_practice/tower_signal.py'
        if not p.is_file():
            raise SystemExit('tower_signal.py 를 찾지 못함: ' + str(p))
        spec = importlib.util.spec_from_file_location('tower_signal', p)
        ts = importlib.util.module_from_spec(spec); spec.loader.exec_module(ts)
        origin = str(p)
    return r12_src, SignalRecorder, ts, origin


R12_SRC, SignalRecorder, _ts, TOWER_SIGNAL_ORIGIN = _load_dependencies()
TOWER, PARAMS, tower_signal_color, tier_boxes = _ts.TOWER, _ts.PARAMS, _ts.tower_signal_color, _ts.tier_boxes

# ---------------------------------------------------------------------------
# 시험 상수 (여기서만 바꾼다)
# ---------------------------------------------------------------------------
WIDTH, HEIGHT = 640, 480
FOCAL_MM, H_APERTURE_MM = 13.096875, 20.955          # fx = 640 × 13.096875 / 20.955 = 400 (스키마 r07 값)
K = np.array([[400., 0, 319.5], [0, 400., 239.5], [0, 0, 1]])
FRAME_DT_S = 0.05                                     # 20 Hz (R12 패킷 주기와 같음)
# 프레임 수 = (시작 전 소등 1 s + 12회 × 2 s + 마지막 소등 3.5 s) / 0.05 s = 28.5 / 0.05 = 570
#   (시작 전 소등은 SignalRecorder 가 OFF 를 먼저 봐야 기록을 시작하므로 넣음. 빼면 550)
PRE_OFF_S, ON_S, OFF_S, END_OFF_S = 1.0, 1.0, 1.0, 3.5
DEFAULT_SEQUENCE = 'RGYRGYGRGRGG'                     # 12회, R4 Y2 G6 (각 1~6회)
TOWER_CENTER_Z = TOWER['base_z_m'] + (min(a for a, _ in TOWER['tiers'].values()) + max(b for _, b in TOWER['tiers'].values())) / 2


# ---------------------------------------------------------------------------
# Isaac 없이 시험 가능한 순수 함수
# ---------------------------------------------------------------------------
def look_at(eye, target):
    """카메라 광학 프레임(x 오른쪽, y 아래, z 앞)의 월드 자세 4×4."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.])
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('카메라가 수직으로 내려다보면 look_at 기준축이 정의되지 않음')
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


START_POSE = (0.734, 0.950, 0.008, 180.0)   # tutor r12_sensor_sim.py 117줄 /World/Spawn (x, y, z, yaw°)
FIELD_SCENE = 'models/navigation/mission_scene.usda'    # tutor 경기장 장면 (벽·박스·패널), 읽기만 함


def start_camera_pose(r12_root, camera='r07_camera1_left_optical'):
    """tutor 시작 자세에서 로봇 카메라 광학 프레임의 월드 자세 (팔 관절 0, 카메라 리그·URDF 그대로)."""
    p = Path(r12_root) / 'scripts/sensor_runtime/camera_rig.py'
    spec = importlib.util.spec_from_file_location('camera_rig', p)
    rig = importlib.util.module_from_spec(spec); spec.loader.exec_module(rig)
    T_base_cam = rig.transforms(Path(r12_root), {f'arm_joint{i}': 0.0 for i in range(1, 6)})[camera]
    x, y, z, yaw = START_POSE
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    W = np.eye(4); W[:2, :2] = [[c, -s], [s, c]]; W[:3, 3] = [x, y, z]
    return W @ np.asarray(T_base_cam, float)


RANDOM_VIEW_RANGES = {
    'forward_m': (0.10, 0.35), 'lateral_m': (-0.10, 0.10), 'height_m': (0.25, 0.70),
    'yaw_deg': (-8.0, 8.0), 'pitch_deg': (-8.0, 8.0), 'roll_deg': (-10.0, 10.0),
}


def random_camera_pose(seed, view_index):
    """로봇 시작 위치는 고정. 카메라 구도만 샘플링하며 팔 도달성은 검증하지 않는다."""
    rng = np.random.default_rng(np.random.SeedSequence([seed, view_index]))
    sample = {key: float(rng.uniform(*bounds)) for key, bounds in RANDOM_VIEW_RANGES.items()}
    bx, by, _, heading = START_POSE
    c, s = np.cos(np.radians(heading)), np.sin(np.radians(heading))
    f, lateral = sample['forward_m'], sample['lateral_m']
    eye = [bx + c * f - s * lateral, by + s * f + c * lateral, sample['height_m']]
    T = look_at(eye, [*TOWER['xy_m'], TOWER_CENTER_Z])
    yaw, pitch, roll = np.radians([sample['yaw_deg'], sample['pitch_deg'], sample['roll_deg']])
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    T[:3, :3] = T[:3, :3] @ Ry @ Rx @ Rz
    return T, sample


def optical_to_usd(T_opt):
    """광학 프레임 → USD 카메라 프레임(-Z 앞, +Y 위): x 축 기준 180° 회전."""
    return np.asarray(T_opt, float) @ np.diag([1., -1., -1., 1.])


def timeline(sequence):
    """프레임별 정답 색 목록 ('R'/'Y'/'G'/'OFF')."""
    n = int(round((PRE_OFF_S + len(sequence) * (ON_S + OFF_S) + END_OFF_S) / FRAME_DT_S))
    truth = []
    for i in range(n):
        t = i * FRAME_DT_S - PRE_OFF_S
        k = int(t // (ON_S + OFF_S)) if t >= 0 else -1
        on = 0 <= k < len(sequence) and (t - k * (ON_S + OFF_S)) < ON_S - 1e-9
        truth.append(sequence[k] if on else 'OFF')
    return truth


def score(truth, predicted, sequence):
    """프레임 정확도(색 바뀌는 순간 ±1프레임 제외)와 SignalRecorder 결과."""
    edge = {i for i in range(1, len(truth)) if truth[i] != truth[i - 1]}
    edge |= {i - 1 for i in edge} | {i + 1 for i in edge}
    counted = [i for i in range(len(truth)) if i not in edge]
    wrong = [i for i in counted if predicted[i] != truth[i]]
    confusion = {}
    for i in counted:
        key = truth[i] + '->' + predicted[i]
        confusion[key] = confusion.get(key, 0) + 1
    rec = SignalRecorder()
    armed, rec_error = False, None
    for i, c in enumerate(predicted):
        if not armed:
            armed = c == 'OFF'          # r12_sensor_ros.py 와 같음: OFF 를 본 뒤부터 기록
            if not armed:
                continue
        try:
            rec.observe(i * FRAME_DT_S, c)
        except ValueError as e:
            rec_error = str(e)
            break
    return dict(frames=len(truth), frames_scored=len(counted), frame_errors=len(wrong),
                frame_accuracy=1 - len(wrong) / max(1, len(counted)), first_wrong_frames=wrong[:10],
                confusion=confusion, recorder_complete=rec.complete, recorder_error=rec_error,
                recorded_sequence=''.join(rec.sequence), sequence_match=rec.complete and ''.join(rec.sequence) == sequence)


def perturb(T, error_m, rng):
    """로봇이 아는 자세에 수평 위치 오차 error_m 를 임의 방향으로 넣음."""
    if error_m <= 0:
        return T
    a = rng.uniform(0, 2 * np.pi)
    T = T.copy()
    T[:2, 3] += error_m * np.array([np.cos(a), np.sin(a)])
    return T


# ---------------------------------------------------------------------------
# Isaac 장면·렌더
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True, help='결과 폴더 (없어야 함)')
    ap.add_argument('--field', action='store_true',
                    help='tutor 미로 벽·박스를 불러옴. 임시 회색 판·바코드·구 가로 램프는 제외')
    ap.add_argument('--random-views', type=int, default=5, help='camera=random의 시점 수. seed와 시점 번호로 재현')
    ap.add_argument('--barcode', choices=['P2-01', 'P2-02', 'P2-03', 'P2-04'], help='Add the supplied photographic 75x25 mm label')
    ap.add_argument('--barcode-center', type=float, nargs=3, default=[0.15, 1.03, 0.14], metavar=('X', 'Y', 'Z'), help='Assumed barcode center in world meters; upright facing +X')
    ap.add_argument('--barcode-upscale', type=int, choices=[1, 2], default=1, help='Optional whole-frame 2x cubic fallback after no raw read; records both')
    ap.add_argument('--barcode-sweep', action='store_true', help='Before signal test, run all four upright labels through 48 camera/placement cases')
    ap.add_argument('--camera', choices=['look', 'start', 'random', 'barcode'], default='look',
                    help="look: 거리별 정면 / start: tutor 상단 카메라 시작 자세 / random: 로봇 고정, 카메라 구도만 샘플링")
    ap.add_argument('--distances', type=float, nargs='+', default=[0.6, 1.0, 2.0], help='신호등까지 수평 거리 (m)')
    ap.add_argument('--camera-height', type=float, default=0.30, help='카메라 높이 (m) [가정: 손목/상단 카메라 관측 자세]')
    ap.add_argument('--off-reference', action='store_true', help='Calibrate three commanded-OFF renders per fixed camera/exposure before the truth timeline')
    ap.add_argument('--locate-body', action='store_true', help='Experimental local HSV body registration on confirmed OFF frames; requires --off-reference')
    ap.add_argument('--hide-tower-test', action='store_true', help='Negative test: hide generated tower before OFF acquisition')
    ap.add_argument('--search-radius-px', type=int, default=60)
    ap.add_argument('--pose-error-m', type=float, default=0.0, help='판정에 쓰는 자세에 넣을 수평 오차 (m)')
    ap.add_argument('--sequence', default=DEFAULT_SEQUENCE)
    ap.add_argument('--emissive', type=float, nargs='+', default=[1.0],
                    help='켜진 렌즈 발광 세기 배율 목록. 여러 값을 주면 전부 기록 (R12 시뮬 기본 1.0). '
                         '통과하는 값을 찾아 맞추는 용도가 아니라 밝기별 성공·실패 기록용')
    ap.add_argument('--dome', type=float, default=1800., help='주변광 세기 (R12 시뮬 기본 1800)')
    ap.add_argument('--subframes', type=int, default=2, help='색 바꾼 뒤 읽기 전 렌더 갱신 횟수')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--r12-src', help='tutor r12/src 경로 (읽기만 함). 기본: R12_SRC 또는 ~/git/lv2-nz2-project2/r12/src')
    ap.add_argument('--max-frames', type=int, default=None,
                    help='조건마다 앞에서부터 이 프레임 수만 렌더 (import·렌더 동작 확인용). 지정하면 순서 채점은 하지 않음')
    ap.add_argument('--smoke', action='store_true',
                    help='동작 확인: 첫 거리·첫 발광 1조건, 30프레임(시작 소등 20 + 첫 색 켜짐 10)만 렌더')
    ap.add_argument('--save-scene', action='store_true', help='장면을 한 번만 scene.usdc로 저장 (기본 저장 안 함)')
    ap.add_argument('--renderer', choices=['RaytracedLighting', 'PathTracing'], default='RaytracedLighting',
                    help='렌더 지연 분리 시험용. 사용한 모드는 summary에 기록')
    ap.add_argument('--gui', action='store_true', help='독립 PreviewCam으로 GUI 표시. 화면 조작은 판독 카메라에 영향 없음')
    ap.add_argument('--pace', action='store_true', help='실제 시간 속도(20 Hz)로 진행. --gui 와 함께 쓰면 점멸을 눈으로 따라갈 수 있음')
    ap.add_argument('--hold', action='store_true', help='--gui 에서 시험이 끝난 뒤 창을 닫을 때까지 유지')
    a = ap.parse_args()
    if a.hide_tower_test and not a.locate_body:
        raise SystemExit('--hide-tower-test requires --locate-body')
    if a.locate_body and not a.off_reference:
        raise SystemExit('--locate-body requires --off-reference')
    if not 1 <= a.search_radius_px <= 256:
        raise SystemExit('--search-radius-px must be 1..256')
    search_module = None
    if a.locate_body:
        search_path = Path(__file__).resolve().parents[1] / 'src/arm_practice/tower_signal_search.py'
        spec = importlib.util.spec_from_file_location('tower_signal_search', search_path)
        search_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(search_module)
    seq = a.sequence.upper()
    if len(seq) != 12 or set(seq) - set('RYG') or any(not 1 <= seq.count(c) <= 6 for c in 'RYG'):
        raise SystemExit('sequence 는 R/Y/G 12개, 색별 1~6회여야 함')
    if a.random_views < 1 or (a.max_frames is not None and a.max_frames < 1):
        raise SystemExit('random-views와 max-frames는 양수여야 함')
    if a.smoke:
        a.distances, a.emissive, a.max_frames = a.distances[:1], a.emissive[:1], (a.max_frames or 30)
        a.random_views = 1
    out = a.out.expanduser()
    if out.exists():
        raise SystemExit(f'이미 존재: {out} (덮어쓰지 않음)')

    out.mkdir(parents=True)
    log = (out / 'run.log').open('a', buffering=1)

    def note(msg):
        print(msg); log.write(time.strftime('%H:%M:%S ') + msg + '\n')

    note('시작: ' + ' '.join(sys.argv))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not a.gui, 'renderer': a.renderer, 'anti_aliasing': 0,
                         'samples_per_pixel_per_frame': 8,
                         'width': WIDTH, 'height': HEIGHT})
    try:
        import omni.usd
        import omni.replicator.core as rep
        rep.orchestrator.set_capture_on_play(False)
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

        omni.usd.get_context().new_stage()
        app.update()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(a.dome)

        def material(name, rgb):
            mat = UsdShade.Material.Define(stage, '/World/Looks/' + name)
            sh = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Shader')
            sh.CreateIdAttr('UsdPreviewSurface')
            sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0))
            sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.5)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
            return mat, sh

        def bind(geom, mat):
            UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)

        if a.field:
            scene = R12_SRC.parent / FIELD_SCENE
            if not scene.is_file():
                raise SystemExit('경기장 장면 없음: ' + str(scene))
            stage.DefinePrim('/World/Mission', 'Xform').GetReferences().AddReference(str(scene))
            app.update()
            # 미로 벽은 유지하고 임시 회색 판·바코드만 이번 시험 장면에서 제외한다.
            removed_panel_paths = []
            for path in ('/World/Mission/StartPanel/Board', '/World/Mission/StartPanel/Barcode'):
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    prim.SetActive(False)
                    removed_panel_paths.append(path)
            note('시험에서 제외한 임시 패널: ' + ', '.join(removed_panel_paths))
            for c in 'RYG':                                  # 장면에 구워진 예전 가로 램프(구 3개)는 끔
                old = stage.GetPrimAtPath('/World/Mission/StartPanel/Lamp' + c)
                if old.IsValid():
                    old.SetActive(False)
            note('경기장 장면 불러옴: ' + str(scene))
        floor = UsdGeom.Cube.Define(stage, '/World/Floor')
        floor.CreateSizeAttr(1.)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.005))
        floor.AddScaleOp().Set(Gf.Vec3f(10, 10, .01))
        bind(floor, material('Floor', (.35, .35, .35))[0])

        tx, ty = TOWER['xy_m']
        look = _ts.TOWER_LOOK
        lin = _ts.srgb_to_linear
        bz, r = TOWER['base_z_m'], TOWER['lens_diameter_m'] / 2
        lens_lo = min(z0 for z0, _ in TOWER['tiers'].values())
        lens_hi = max(z1 for _, z1 in TOWER['tiers'].values())
        base_h = _ts._BASE_H
        # 파란 받침 (가로 막대 2개를 십자로 근사)
        for i, (sx, sy) in enumerate([look['base_size_m'], look['base_size_m'][::-1]]):
            b = UsdGeom.Cube.Define(stage, f'/World/Tower/Base{i}')
            b.CreateSizeAttr(1.); b.AddTranslateOp().Set(Gf.Vec3d(tx, ty, bz + base_h / 2))
            b.AddScaleOp().Set(Gf.Vec3f(sx, sy, base_h))
            bind(b, material(f'Base{i}', lin(look['base_srgb']))[0])
        pole = UsdGeom.Cylinder.Define(stage, '/World/Tower/Pole')
        pole_h = lens_lo - base_h
        pole.CreateRadiusAttr(look['pole_radius_m']); pole.CreateHeightAttr(pole_h); pole.CreateAxisAttr('Z')
        pole.AddTranslateOp().Set(Gf.Vec3d(tx, ty, bz + base_h + pole_h / 2))
        bind(pole, material('Pole', lin(look['pole_srgb']))[0])
        cap = UsdGeom.Cylinder.Define(stage, '/World/Tower/Cap')
        cap.CreateRadiusAttr(r); cap.CreateHeightAttr(look['cap_h_m']); cap.CreateAxisAttr('Z')
        cap.AddTranslateOp().Set(Gf.Vec3d(tx, ty, bz + lens_hi + look['cap_h_m'] / 2))
        bind(cap, material('Cap', lin(look['cap_srgb']))[0])
        lamp_geoms, off_materials, on_materials = {}, {}, {}
        for c in 'RYG':
            z0, z1 = TOWER['tiers'][c]
            lamp = UsdGeom.Cylinder.Define(stage, '/World/Tower/Lamp' + c)
            lamp.CreateRadiusAttr(r); lamp.CreateHeightAttr(z1 - z0); lamp.CreateAxisAttr('Z')
            lamp.AddTranslateOp().Set(Gf.Vec3d(tx, ty, bz + (z0 + z1) / 2))
            mat, _ = material('LampOff' + c, lin(look['lens_off_srgb'][c]))
            bind(lamp, mat)
            lamp_geoms[c], off_materials[c] = lamp, mat
            for level_index, level in enumerate(a.emissive):
                on_lin = np.asarray(lin(look['lens_on_srgb'][c]))
                on_mat, on_shader = material(f'LampOn{c}_{level_index}', tuple(on_lin))
                on_shader.GetInput('emissiveColor').Set(Gf.Vec3f(*(on_lin * level)))
                on_materials[(c, float(level))] = on_mat

        if a.hide_tower_test:
            # Visibility keeps lamp handles valid for the timeline material updates.
            UsdGeom.Xform.Define(stage, '/World/Tower').MakeInvisible()
        cam = UsdGeom.Camera.Define(stage, '/World/Cam')
        cam.CreateFocalLengthAttr(FOCAL_MM)
        cam.CreateHorizontalApertureAttr(H_APERTURE_MM)
        cam.CreateVerticalApertureAttr(H_APERTURE_MM * HEIGHT / WIDTH)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 30.))
        cam_op = cam.AddTransformOp()
        rp = rep.create.render_product('/World/Cam', (WIDTH, HEIGHT))
        preview_op = None
        if a.gui:
            preview = UsdGeom.Camera.Define(stage, '/World/PreviewCam')
            preview.CreateFocalLengthAttr(FOCAL_MM)
            preview.CreateHorizontalApertureAttr(H_APERTURE_MM)
            preview.CreateVerticalApertureAttr(H_APERTURE_MM * HEIGHT / WIDTH)
            preview.CreateClippingRangeAttr(Gf.Vec2f(.01, 30.))
            preview_op = preview.AddTransformOp()
            try:
                from omni.kit.viewport.utility import get_active_viewport
                get_active_viewport().camera_path = '/World/PreviewCam'  # GUI 조작은 판독 카메라에 영향 없음
            except Exception as e:                                   # 판정에는 영향 없음
                note('뷰포트 카메라 전환 실패 (판정과 무관): ' + repr(e))
        rgb_ann = rep.AnnotatorRegistry.get_annotator('rgb'); rgb_ann.attach(rp)
        dep_ann = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane'); dep_ann.attach(rp)

        applied_lamp_states = {}

        def set_lamps(active, emissive):
            changed = False
            for c, lamp in lamp_geoms.items():
                on = active == c
                state = (on, float(emissive) if on else 0.0)
                if applied_lamp_states.get(c) == state:
                    continue
                # 재질 입력을 매번 바꾸는 대신 준비된 점등/소등 재질을 전환한다.
                bind(lamp, on_materials[(c, float(emissive))] if on else off_materials[c])
                applied_lamp_states[c] = state
                changed = True
            return changed

        def _array(ann):
            d = ann.get_data()
            if isinstance(d, dict):                     # 버전에 따라 {'data': ...} 형태
                d = d.get('data')
            return np.asarray(d)

        def render():
            # USD 재질 변경을 반영하고, 각 캡처의 시간을 전진시켜 이전 프레임 재사용을 피한다.
            app.update()
            rep.orchestrator.step(rt_subframes=max(1, a.subframes),
                                  delta_time=FRAME_DT_S, pause_timeline=True)
            rgb, depth = _array(rgb_ann), _array(dep_ann)
            if rgb.ndim != 3 or rgb.shape[:2] != (HEIGHT, WIDTH) or depth.shape != (HEIGHT, WIDTH):
                raise RuntimeError(f'렌더 결과 형식 오류: rgb {rgb.shape} {rgb.dtype}, depth {depth.shape}')
            return rgb[:, :, :3].astype(np.uint8).copy(), depth.astype(np.float32).copy()

        barcode_meta = None
        if a.barcode or a.barcode_sweep:
            from barcode_support import add_label, decode_rgb, StableCode, projected_bars_px
            def move_camera(T):
                matrix = Gf.Matrix4d(*optical_to_usd(T).T.flatten().tolist())
                cam_op.Set(matrix)
                if preview_op is not None:
                    preview_op.Set(matrix)
                for _ in range(20):
                    app.update()
            if a.barcode_sweep:
                from barcode_sweep import run_sweep
                set_lamps(None, a.emissive[0])
                run_sweep(stage, render, move_camera, look_at, K, out, note, upscale=a.barcode_upscale)
                stage.RemovePrim('/World/TestBarcode')
            if a.barcode:
                barcode_meta = add_label(stage, a.barcode, a.barcode_center)
        # Initialize the render product before the first scored camera transform.
        # These initial captures are unscored and are not assigned signal truth.
        for _ in range(4):
            render()
        note('장면·카메라 준비 완료 — 후보 규칙, 실사 정답 영상 검증 전')
        rng = np.random.default_rng(a.seed)
        truth_full = timeline(seq)
        truth = truth_full[:a.max_frames] if a.max_frames else truth_full
        center = np.array([tx, ty, TOWER_CENTER_Z])
        results = []          # (발광, 거리) 조건마다 1행. 실패도 그대로 남김
        for emissive in a.emissive:
            for view_index, dist in enumerate(a.distances if a.camera == 'look' else [None] * (a.random_views if a.camera == 'random' else 1)):
                view_parameters = None
                if a.camera == 'barcode':
                    if not a.barcode:
                        raise ValueError('--camera barcode requires --barcode')
                    T_true = look_at(np.asarray(a.barcode_center) + [.20, -.03, .07], center)
                    dist = float(np.linalg.norm(T_true[:2, 3] - center[:2]))
                elif a.camera == 'random':
                    T_true, view_parameters = random_camera_pose(a.seed, view_index)
                    dist = float(np.linalg.norm(T_true[:2, 3] - np.asarray(TOWER['xy_m'])))
                elif dist is None:
                    T_true = start_camera_pose(R12_SRC.parent)
                    dist = float(np.hypot(*(T_true[:2, 3] - np.array([tx, ty]))))   # 기록용 수평 거리
                else:
                    T_true = look_at([tx, ty - dist, a.camera_height], center)
                m = optical_to_usd(T_true)
                cam_op.Set(Gf.Matrix4d(*m.T.flatten().tolist()))   # USD 는 행벡터 규약 → 전치
                if preview_op is not None:
                    preview_op.Set(Gf.Matrix4d(*m.T.flatten().tolist()))
                if a.save_scene and not (out / 'scene.usdc').exists():
                    stage.Export(str((out / 'scene.usdc').resolve()))
                for _ in range(20):
                    app.update()                                     # 카메라 이동 후 안정화
                T_known = perturb(T_true, a.pose_error_m, rng)
                label = f'e{emissive:g}_v{view_index}_{dist:.1f}m'
                # Wait for valid RGB in this pose before calibration or scoring.
                for warm_count in range(32):
                    ready_rgb, _ = render()
                    if warm_count >= 3 and np.max(ready_rgb) > np.min(ready_rgb):
                        break
                else:
                    raise RuntimeError('Invalid uniform RGB before signal condition')
                off_reference = None
                registration, aligned_reference, registered_context = None, None, None
                exposure_token = f'simulation_dome_{a.dome}'
                if a.off_reference:
                    set_lamps(None, emissive)
                    for _ in range(4):
                        app.update()
                        render()
                    off_frames = []
                    for _ in range(3):
                        off_rgb, _ = render()
                        if np.max(off_rgb) <= np.min(off_rgb):
                            raise RuntimeError('Invalid RGB in OFF reference capture')
                        off_frames.append(off_rgb)
                    ref_boxes, ref_polygons, ref_reason = _ts.tier_rois(T_known, K, off_frames[0].shape)
                    if ref_reason:
                        raise RuntimeError('OFF reference ROI invalid: ' + ref_reason)
                    off_reference = _ts.make_off_reference(off_frames, ref_boxes,
                        roi_polygons=ref_polygons, confirmed_off=True,
                        context_key=_ts.camera_context(T_known,K,off_frames[0].shape,exposure_token))
                    _save_png(out / f'off_reference_{label}.png', off_frames[0], ref_boxes, ref_polygons)
                    np.savez_compressed(out / f'off_reference_{label}.npz', rgb=off_reference.rgb,
                                        camera=T_known, context_key=off_reference.context_key)
                if a.locate_body:
                    # OFF is explicitly commanded by this harness, never inferred from a prediction.
                    registration = search_module.locate_body(
                        np.median(off_frames, axis=0).astype(np.uint8), ref_boxes, ref_polygons,
                        confirmed_off=True, params={'radius_px': a.search_radius_px})
                    if registration['found']:
                        registered_context = _ts.camera_context(T_known,K,off_frames[0].shape,exposure_token) + registration['key']
                        aligned_reference = _ts.make_off_reference(off_frames, registration['boxes'],
                            roi_polygons=registration['polygons'], confirmed_off=True, context_key=registered_context)
                        _save_png(out / f'body_off_reference_{label}.png', off_frames[0], registration['boxes'], registration['polygons'])
                        np.savez_compressed(out / f'body_off_reference_{label}.npz', rgb=aligned_reference.rgb, camera=T_known, context_key=registered_context)
                    note('몸통 검색: ' + json.dumps(registration, ensure_ascii=False))
                projection_only_predicted, registration_times = [], []
                no_reference_predicted, prior_candidate_predicted = [], []
                predicted, reasons, saved, lit_ratio = [], {}, set(), {c: [] for c in 'RYG'}
                legacy_predicted, decision_counts, frame_rows = [], {}, []
                debug_saved = set()
                barcode_rows = []
                stable_barcode = StableCode() if a.barcode else None
                note(f'조건 시작: 발광 {emissive:g}, 거리 {dist:.1f} m, {len(truth)}프레임')
                t_start = time.monotonic()
                for i, color in enumerate(truth):
                    if a.pace:
                        time.sleep(max(0.0, t_start + i * FRAME_DT_S - time.monotonic()))
                    changed = set_lamps(None if color == 'OFF' else color, emissive)
                    if changed:
                        for _ in range(4):
                            app.update()  # 재질 전환 반영. 실시간 처리 성능을 측정하는 시험은 아님
                    rgb, depth = render()
                    if np.max(rgb) <= np.min(rgb):
                        raise RuntimeError(f'Invalid uniform signal RGB at frame {i}')
                    actual_camera = np.asarray(cam_op.Get(), dtype=float).T @ np.diag([1., -1., -1., 1.])
                    if not np.allclose(actual_camera, T_true, rtol=0, atol=1e-6):
                        raise RuntimeError('시험 카메라 자세가 렌더 중 변경됨; 이 조건은 무효')
                    if a.barcode:
                        decoded = decode_rgb(rgb, upscale=a.barcode_upscale)
                        decoded.update(frame=i, stable=stable_barcode.observe(decoded['reads']))
                        barcode_rows.append(decoded)
                    res = tower_signal_color(rgb, T_known, K, depth=depth,
                                             off_reference=off_reference, exposure_token=exposure_token)
                    projection_only_predicted.append(res['color'])
                    if a.locate_body:
                        res = search_module.read_registered(_ts, rgb, registration,
                            context_key=registered_context, off_reference=aligned_reference, depth=depth)
                        if 'registration_read_ms' in res:
                            registration_times.append(res['registration_read_ms'])
                    unref = _ts.color_from_boxes(rgb, res['boxes'], roi_polygons=res.get('roi_polygons')) if res['ratios'] else res
                    prior = _ts.color_from_boxes(rgb, res['boxes'], dict(PARAMS,white_unique_enabled=False),
                                                roi_polygons=res.get('roi_polygons')) if res['ratios'] else res
                    no_reference_predicted.append(unref['color'])
                    prior_candidate_predicted.append(prior['color'])
                    predicted.append(res['color'])
                    # 이전 규칙도 같은 RGB·영역으로 계산해 입력 차이 없이 비교한다.
                    old_lit = [c for c in 'RYG' if res['ratios'].get(c, 0) >= PARAMS['on_ratio']]
                    old_color = ('OFF' if not old_lit else old_lit[0] if len(old_lit) == 1 else 'UNKNOWN') if res['ratios'] else 'UNKNOWN'
                    legacy_predicted.append(old_color)
                    decision = res.get('decision', 'invalid')
                    decision_counts[decision] = decision_counts.get(decision, 0) + 1
                    frame_rows.append(dict(frame=i, time_s=i * FRAME_DT_S, truth=color,
                                           predicted=res['color'], legacy=old_color, reason=res['reason'],
                                           ratios=res['ratios'], white_ratios=res.get('white_ratios', {}),
                                           scores=res.get('scores', {}), decision=decision,
                                           raw_ratios=res.get('raw_ratios'), delta_ratios=res.get('delta_ratios'),
                                           without_off_reference=unref['color'], prior_candidate=prior['color'],
                                           projection_only=projection_only_predicted[-1],
                                           body_support=res.get('body_support'),
                                           registration_read_ms=res.get('registration_read_ms')))
                    if res['reason']:
                        reasons[res['reason']] = reasons.get(res['reason'], 0) + 1
                    if res['color'] != color and str(res['reason']) not in debug_saved:
                        debug_saved.add(str(res['reason']))
                        _save_png(out / f'debug_{label}_f{i}.png', rgb, res['boxes'], res.get('roi_polygons'))
                        np.savez_compressed(out / f'debug_{label}_f{i}.npz',
                                            rgb=rgb, depth=depth, camera=T_known)
                    if color != 'OFF' and res['ratios']:
                        lit_ratio[color].append(res['ratios'][color])   # 켜진 칸의 밝은 픽셀 비율 (기준 on_ratio 와 비교용)
                    if color not in saved and i > 0 and truth[i - 1] == color:
                        saved.add(color)
                        _save_png(out / f'frame_{label}_{color}.png', rgb, res['boxes'], res.get('roi_polygons'))
                if a.max_frames:
                    # 짧은 확인: 순서 채점 없이 프레임별 일치만 기록
                    s = dict(frames=len(truth), frame_matches=sum(p == t for p, t in zip(predicted, truth)),
                             truth=''.join(c[0] for c in truth), predicted=''.join(p[0] for p in predicted),
                             sequence_match=None, recorded_sequence=None, recorder_error=None, frame_accuracy=None)
                else:
                    s = score(truth, predicted, seq)
                trace_path = out / f'frames_{label}.jsonl'
                with trace_path.open('w') as trace:
                    for row in frame_rows:
                        trace.write(json.dumps(row) + '\n')
                s['body_registration'] = registration
                s['registration_read_ms'] = (dict(median=float(np.median(registration_times)),p95=float(np.percentile(registration_times,95))) if registration_times else None)
                s['projection_only'] = (dict(frame_matches=sum(p == t for p,t in zip(projection_only_predicted,truth))) if a.max_frames else score(truth,projection_only_predicted,seq))
                s['off_reference_used'] = (aligned_reference is not None) if a.locate_body else off_reference is not None
                s['off_reference_frames'] = off_reference.frame_count if off_reference is not None else 0
                if not a.max_frames:
                    s['without_off_reference'] = score(truth, no_reference_predicted, seq)
                    s['prior_candidate_same_rois'] = score(truth, prior_candidate_predicted, seq)
                s['legacy'] = (dict(frame_matches=sum(p == t for p, t in zip(legacy_predicted, truth)))
                               if a.max_frames else score(truth, legacy_predicted, seq))
                s['decision_counts'] = decision_counts
                s['frame_trace'] = trace_path.name
                s['view_index'], s['view_parameters'] = view_index, view_parameters
                s['camera_world'] = T_true.tolist()
                s['camera_projection_used'] = T_known.tolist()
                s.update(emissive=emissive, distance_m=dist, unknown_reasons=reasons,
                         lit_ratio_min={c: (min(v) if v else None) for c, v in lit_ratio.items()},
                         lens_px=_lens_px(T_known))
                if a.barcode:
                    barcode_trace = out / f'barcodes_{label}.jsonl'
                    barcode_trace.write_text(''.join(json.dumps(row) + '\n' for row in barcode_rows))
                    s['barcode'] = dict(expected=a.barcode, metadata=barcode_meta,
                        bars_projection=projected_bars_px(T_true, K, a.barcode_center),
                        frames=len(barcode_rows), upscale=a.barcode_upscale,
                        raw_read_frames=sum(row['raw_read_count'] > 0 for row in barcode_rows),
                        correct_frames=sum({r['text'] for r in row['reads']} == {a.barcode} for row in barcode_rows),
                        wrong_frames=sum(any(r['text'] != a.barcode for r in row['reads']) for row in barcode_rows),
                        stable_correct_frames=sum(row['stable'] == a.barcode for row in barcode_rows),
                        trace=barcode_trace.name)
                results.append(s)
                print(json.dumps({k: s[k] for k in ('emissive', 'distance_m', 'lens_px', 'frame_accuracy', 'sequence_match',
                                                    'recorded_sequence', 'recorder_error')}, ensure_ascii=False))
        summary = dict(tower_hidden_negative_test=a.hide_tower_test, body_search_enabled=a.locate_body, body_search_params=(dict(search_module.SEARCH, radius_px=a.search_radius_px) if a.locate_body else None), off_reference_enabled=a.off_reference, off_reference_scope='Simulation commanded-OFF calibration only; real matched OFF video not available', rule_status='후보 규칙, 실사 정답 영상 검증 전', barcode=barcode_meta, sequence=seq, sequence_counts={c: seq.count(c) for c in 'RYG'}, frames=len(truth),
                       full_frames=len(truth_full), smoke_or_truncated=bool(a.max_frames),
                       r12_src=str(R12_SRC), tower_signal_source=TOWER_SIGNAL_ORIGIN,
                       field_scene_source=str(scene) if a.field else None,
                       excluded_field_prims=removed_panel_paths if a.field else [],
                       maze_prim_count=sum(1 for prim in stage.Traverse() if str(prim.GetPath()).startswith('/World/Mission/Maze/')) if a.field else 0,
                       renderer=a.renderer, scene_saved=a.save_scene,
                       fixed_robot_pose=list(START_POSE), random_view_ranges=RANDOM_VIEW_RANGES if a.camera == 'random' else None,
                       seed=a.seed,
                       camera_mode=a.camera, field=a.field, camera_height_m=a.camera_height if a.camera == 'look' else None, pose_error_m=a.pose_error_m, dome=a.dome,
                       subframes=a.subframes, resolution=[WIDTH, HEIGHT], K=K.tolist(), tower=TOWER, params=PARAMS,
                       results=results, passed=None if a.max_frames else sum(bool(r['sequence_match']) for r in results),
                       conditions=len(results),
                       scope=('단독 렌더 시험의 조건별 성공·실패 기록. 이 해상도·화각·렌더 설정에서만 유효. '
                              '로봇·위치추정·ROS·실물 카메라·실물 신호등 판정 근거가 아님'))
        (out / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        if a.max_frames:
            print(f"짧은 확인 {len(truth)}프레임 렌더 완료 (순서 채점 안 함) ->", out / 'summary.json')
        else:
            print(f"조건 {summary['conditions']}개 중 순서 일치 {summary['passed']}개 ->", out / 'summary.json')
        code = 0          # 조건별 결과 기록이 목적이므로 실패가 있어도 정상 종료. 판단은 summary.json 으로
    except BaseException:
        import traceback
        (out / 'error.txt').write_text(traceback.format_exc())
        note('오류로 중단 → error.txt')
        raise
    finally:
        if a.gui and a.hold:
            note('창을 닫으면 종료됩니다')
            while app.is_running():
                app.update()
        log.close()
        app.close()
    sys.exit(code)


def _lens_px(T_cam):
    """이 카메라 자세에서 렌즈 한 칸이 화면에 보이는 크기 (판정 칸 축소 전, 가로×세로 px)."""
    boxes, _ = tier_boxes(T_cam, K, (HEIGHT, WIDTH), TOWER, dict(PARAMS, inner=1.0, min_tier_px=0))
    if not boxes:
        return None
    b = boxes['Y']
    return [b[2] - b[0], b[3] - b[1]]


def _save_png(path, rgb, boxes, polygons=None):
    """확인용 사진: 칸 사각형 표시 (빨/노/초 테두리)."""
    from PIL import Image, ImageDraw
    im = Image.fromarray(np.asarray(rgb, np.uint8))
    d = ImageDraw.Draw(im)
    for c, b in (boxes or {}).items():
        color={'R':(255,0,0),'Y':(255,255,0),'G':(0,255,0)}[c]
        if polygons and c in polygons:
            points=[tuple(p) for p in polygons[c]]
            d.line(points+[points[0]],fill=color,width=1)
        else:
            d.rectangle(b[:4],outline=color)
    im.save(path)


if __name__ == '__main__':
    main()
