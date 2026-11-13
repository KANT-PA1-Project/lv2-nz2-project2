#!/usr/bin/env python3
"""최종 조립 URDF(assembly_puck_slots) 로봇, 손목 D435 RGB → 바코드 → 팔 이동 → 신호등 시험.

r20d_perception/run.py 를 바탕으로 로봇 모델만 바꾼 사본:
- 로봇 = ~/Desktop/assembly_puck_slots./assembly.urdf (차체+팔+손목 카메라 일체). 조향·바퀴는 고정.
- 카메라 = URDF wrist_d435_color_optical_frame 변환을 wrist_roll 아래 USD 카메라로 붙임.
- 차체는 물리 스텝마다 지정 자세로 붙잡는다(운동학적, 바퀴 접촉 떨림 회피). barcode_base 와 tower_base 가 다르면
  바코드 확정 후 차체를 base_motion 속도로 연속 이동(물리 스텝마다 자세 보간, 바퀴 구동 아님)하면서
  팔도 동시에 신호등 자세로 움직인다.
- 팔 자기충돌 끔: 조립본 검증(verification.json)도 링크 간 충돌을 끈 상태.

ROS·실물 카메라·주행 정책은 시작하지 않는다. 인지는 렌더 픽셀만 사용한다.
"""
import argparse
import json
import math
import os
import shutil
import time
from pathlib import Path
import sys
from types import SimpleNamespace

from common import (ARM_JOINTS, ARM_LINKS, JAW_JOINTS, PERCEPTION_ROOT, R12, WRIST_LINK,
                    arm_limits, load_config, sha256)
from appearance import asset_paths, apply_floor, apply_gripper
from signal_fallback import SignalFallback, wall_budget


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    ap.add_argument('--prepared', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path, help='새 결과 폴더')
    args = ap.parse_args()
    cfg, model = load_config(args.config)
    tower_only = cfg.get('trial_mode') == 'tower_only'
    route = SignalFallback(cfg['signal_fallback']) if cfg.get('signal_fallback') else None
    wall_budget_s = wall_budget(cfg)
    prepared = args.prepared.expanduser().resolve()
    manifest = json.loads((prepared / 'manifest.json').read_text())
    if manifest['config'] != cfg or manifest['sim_urdf_sha256'] != sha256(prepared / 'assembly_sim.urdf'):
        raise ValueError('준비 자료와 설정이 다름: 새 폴더로 prepare 다시 실행')
    if manifest['source_urdf_sha256'] != sha256(Path(cfg['assembly_dir']) / 'assembly.urdf'):
        raise ValueError('원본 URDF 가 준비 후 바뀜')
    if any(sha256(path) != digest for path, digest in manifest['mesh_sha256'].items()):
        raise ValueError('원본 메시가 준비 후 바뀜')
    os.environ['R12_SRC'] = str(R12 / 'src')
    sys.path.insert(0, str(PERCEPTION_ROOT / 'scripts'))
    from live_mission_optimized import Preparation, load_color_rule, decode_bgr
    from test_mission_camera_isaac import VideoSink
    from mission_summary import MissionSession
    import tower_live_hsv
    if cfg.get('signal_patch') == 'oblique':  # 원본 파일 수정 없이 실행 중에만 판정 함수 교체
        import tower_oblique
        tower_oblique.install()
    import cv2
    import numpy as np
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    provenance = {str(p): sha256(p) for p in Path(__file__).parent.glob('*.py')}
    provenance.update({str(p): sha256(p) for p in asset_paths(cfg)})
    for p in (PERCEPTION_ROOT / 'scripts').glob('*.py'):
        provenance[str(p)] = sha256(p)
    for p in (PERCEPTION_ROOT / 'src/arm_practice').glob('*.py'):
        provenance[str(p)] = sha256(p)
    for p in (R12 / 'models/navigation/mission_scene.usda', R12 / 'src/arm_practice/mission.py',
              Path(cfg['assembly_dir']) / 'assembly.urdf', Path(cfg['assembly_dir']) / 'sensors.json',
              PERCEPTION_ROOT / 'configs/barcodes.json',
              PERCEPTION_ROOT / 'references' / (cfg['barcode']['code'] + '.png')):
        provenance[str(p)] = sha256(p)
    (out / 'inputs.json').write_text(json.dumps(dict(config=cfg, prepared=manifest, source_hashes=provenance),
                                                indent=2, ensure_ascii=False))
    snapshots = out / 'sources'
    snapshots.mkdir()
    for source, digest in provenance.items():
        path = Path(source)
        if path.suffix in ('.py', '.json'):
            shutil.copy2(path, snapshots / (digest[:12] + '-' + path.name))
    result = dict(status='not_completed', scope=cfg['scope'], rule_status='후보 규칙, 실사 정답 영상 검증 전',
                  robot_model='assembly_puck_slots/assembly.urdf', driving_tested=False,
                  learned_policy_tested=False, physical_robot_tested=False, arm_self_collision=False,
                  trial_mode=cfg.get('trial_mode', 'barcode_then_tower'), barcode_tested=not tower_only)
    result['wall_budget_s'] = wall_budget_s
    app = video = world = None
    camera_fps = cfg.get('camera_fps', 30)
    physics_hz = cfg.get('physics_hz', 960)
    width, height = cfg['camera']['width'], cfg['camera']['height']
    fx = width / 2 / math.tan(math.radians(cfg['camera']['hfov_deg']) / 2)
    cmd_velocity = float(cfg['arm_command_velocity_rad_s'])
    lower, upper = arm_limits(cfg, model)
    wall_start = time.monotonic()
    log = (out / 'frames.jsonl').open('w', buffering=1)
    physics_log = (out / 'physics.jsonl').open('w', buffering=1)
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'renderer': cfg['renderer'],
                             'samples_per_pixel_per_frame': int(cfg.get('render', {}).get('spp', 8)),
                             'width': width, 'height': height})
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
        from scene import build_fixture, ClearanceVeto
        import carb
        carb.settings.get_settings().set_bool('/rtx/post/histogram/enabled', False)
        # 렌더 선명도(B04: 막대 경계 흐림). 디노이저 끄기·픽셀당 샘플 수.
        render_cfg = cfg.get('render', {})
        rs = carb.settings.get_settings()
        rs.set_int('/rtx/pathtracing/spp', int(render_cfg.get('spp', 8)))
        rs.set_int('/rtx/pathtracing/totalSpp', int(render_cfg.get('spp', 8)))
        rs.set_bool('/rtx/pathtracing/optixDenoiser/enabled', bool(render_cfg.get('denoiser', True)))
        result['render'] = dict(spp=int(render_cfg.get('spp', 8)), denoiser=bool(render_cfg.get('denoiser', True)))
        signal_camera = cfg.get('signal_camera', 'wrist')
        result['signal_camera'] = signal_camera

        usd = URDFImporter(URDFImporterConfig(
            urdf_path=str(prepared / 'assembly_sim.urdf'), usd_path=str(out / 'imported'),
            fix_base=False, merge_fixed_joints=False, allow_self_collision=False,
            joint_drive_type='force', joint_target_type='position',
            override_joint_stiffness=cfg['arm_drive']['stiffness_per_deg'],
            override_joint_damping=cfg['arm_drive']['damping_per_deg'])).import_urdf()
        omni.usd.get_context().new_stage()
        app.update()
        world = World(stage_units_in_meters=1., physics_dt=1/physics_hz, rendering_dt=1/camera_fps)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, 'Z')
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateSolverTypeAttr('PGS')
        result['physics_solver'] = 'PGS'
        if cfg.get('neutral_floor', False):
            world.scene.add_ground_plane(size=20., color=np.array([.35, .35, .35]), restitution=0.)
            result['ground_visual'] = 'neutral_0.35_no_default_blue_grid'
        else:
            ground = world.scene.add_default_ground_plane()
            result['ground_visual'] = 'default_grid'
            if cfg.get('floor_visual_usd'):
                result['ground_visual'] = apply_floor(stage, ground, cfg['floor_visual_usd'])
        rule = load_color_rule()
        set_signal = build_fixture(stage, cfg, rule)
        start_base = cfg['tower_base_xyz_yaw'] if tower_only else cfg['barcode_base_xyz_yaw']
        approach = None if tower_only else cfg.get('approach')
        # 신호 먼저: 시작 위치(규정 관측 위치)에서 상단으로 12회 기록 → 바코드로 접근해 해독
        tower_first = bool(approach) and cfg.get('order') == 'tower_first'
        result['order'] = 'tower_first' if tower_first else 'barcode_first'
        if approach:  # 시작 위치(팔 접음)에서 출발해 바코드 앞으로 접근
            start_base = approach['start_base_xyz_yaw']
        x, y, z, yaw = start_base
        spawn = UsdGeom.Xform.Define(stage, '/World/Spawn')
        spawn.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        spawn.AddRotateZOp().Set(float(np.degrees(yaw)))
        robot_path = '/World/Spawn/Robot'
        robot_prim = stage.DefinePrim(robot_path, 'Xform')
        robot_prim.GetReferences().AddReference(usd)
        variants = robot_prim.GetVariantSet('Physics')
        if 'physx' not in variants.GetVariantNames():
            raise ValueError('가져온 자산에 PhysX payload variant 없음')
        variants.SetVariantSelection('physx')
        app.update()
        if cfg.get('black_gripper', False):
            result['gripper_visual'] = apply_gripper(stage, robot_prim)
        world_joints = [str(p.GetPath()) for p in Usd.PrimRange(robot_prim) if p.IsA(UsdPhysics.FixedJoint)
                        and not UsdPhysics.FixedJoint(p).GetBody0Rel().GetTargets()]
        if world_joints:
            raise ValueError(f'fix_base=False 인데 월드 고정 관절이 있음: {world_joints}')
        roots = [p for p in Usd.PrimRange(robot_prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise ValueError(f'articulation root 1개 예상, 실제 {[str(p.GetPath()) for p in roots]}')
        robot = world.scene.add(SingleArticulation(str(roots[0].GetPath()), name='assembly'))
        physx = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
        physx.CreateSolverPositionIterationCountAttr(32)
        physx.CreateSolverVelocityIterationCountAttr(16)
        # 손목 카메라: URDF wrist_roll → wrist_d435_color_optical_frame (ROS 광학 → USD 카메라 축)
        links = [p for p in Usd.PrimRange(robot_prim) if p.GetName() == WRIST_LINK]
        if len(links) != 1:
            raise ValueError(f'{WRIST_LINK} 링크 1개 예상, 실제 {len(links)}')
        def define_camera(path, parent_to_optical):
            camera = UsdGeom.Camera.Define(stage, path)
            transform = np.array(parent_to_optical, dtype=float)
            transform[:3, :3] = transform[:3, :3] @ np.diag([1., -1., -1.])
            camera.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
            camera.CreateFocalLengthAttr(20.)
            camera.CreateHorizontalApertureAttr(20. * width / fx)
            camera.CreateVerticalApertureAttr(20. * height / fx)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 30.))
            return camera

        cam_path = str(links[0].GetPath()) + '/perception_camera'
        cam = define_camera(cam_path, manifest['wrist_camera']['parent_to_optical'])
        # 상단(마스트) D435: URDF base_link → mast_d435_color_optical_frame (하향각은 config 로 바꿀 수 있음)
        bases = [p for p in Usd.PrimRange(robot_prim) if p.GetName() == 'base_link']
        if len(bases) != 1:
            raise ValueError(f'base_link 1개 예상, 실제 {len(bases)}')
        mast_path = str(bases[0].GetPath()) + '/mast_camera'
        mast_cam = define_camera(mast_path, manifest['mast_camera']['parent_to_optical'])
        result['mast_camera'] = dict(pitch_down_deg=manifest['mast_camera']['pitch_down_deg'],
                                     frame=manifest['mast_camera']['frame'])
        result['camera'] = dict(width=width, height=height, fx_px=fx, hfov_deg=cfg['camera']['hfov_deg'],
                                parent=WRIST_LINK, frame=manifest['wrist_camera']['frame'])
        rep.orchestrator.set_capture_on_play(False)
        product = rep.create.render_product(cam_path, (width, height))
        rgb_ann = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb_ann.attach(product)
        mast_product = rep.create.render_product(mast_path, (width, height))
        mast_ann = rep.AnnotatorRegistry.get_annotator('rgb')
        mast_ann.attach(mast_product)

        def grab(annotator):
            data = annotator.get_data()
            rgb = np.asarray(data.get('data') if isinstance(data, dict) else data)
            if rgb.dtype != np.uint8 or rgb.shape[:2] != (height, width) or rgb.ndim != 3:
                raise ValueError(f'렌더 RGB 비정상: {rgb.shape} / {rgb.dtype}')
            return cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)

        def side_by_side(wrist, mast):
            return np.hstack([wrist, mast])
        drive_cfg = cfg['arm_drive']
        for prim in Usd.PrimRange(robot_prim):
            if prim.GetName() in ARM_JOINTS:
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                drive.CreateMaxForceAttr(float(drive_cfg['max_force_nm']))
                drive.CreateStiffnessAttr(float(drive_cfg['stiffness_per_deg']))
                drive.CreateDampingAttr(float(drive_cfg['damping_per_deg']))
        world.reset()
        result['dof_names'] = list(robot.dof_names)
        contact_probes = []
        for prim in Usd.PrimRange(robot_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in ARM_LINKS:
                probe = RigidPrim(prim_paths_expr=str(prim.GetPath()), name='contact_'+prim.GetName(),
                                  track_contact_forces=True, reset_xform_properties=False)
                probe.initialize()
                contact_probes.append((prim.GetName(), probe))
        indices = np.array([robot.get_dof_index(n) for n in ARM_JOINTS])
        initial = np.array(cfg['tower_arm_waypoints_rad'][-1] if tower_only else
                           (approach['start_arm_q_rad'] if approach else cfg['barcode_arm_q_rad']), dtype=float)
        robot.set_joint_positions(initial, joint_indices=indices)  # 초기 자세만 직접 지정
        robot.set_joint_velocities(np.zeros(5), joint_indices=indices)
        jaw_present = [n for n in JAW_JOINTS if n in robot.dof_names]
        result['jaw_dofs'] = jaw_present
        for name in jaw_present:  # mimic 이 DOF 로 남으면 양쪽에 같은 열림 목표
            gi = np.array([robot.get_dof_index(name)])
            value = np.array([float(cfg['jaw_open_m'])])
            robot.set_joint_positions(value, joint_indices=gi)
            robot.set_joint_velocities(np.zeros(1), joint_indices=gi)
            robot.apply_action(ArticulationAction(joint_positions=value, joint_indices=gi))

        def base_pose():
            p, quat = robot.get_world_pose()  # quat = w,x,y,z
            w, qx, qy, qz = map(float, quat)
            return np.array(p, float), math.atan2(2*(w*qz+qx*qy), 1-2*(qy*qy+qz*qz))

        def base_speed():
            return (float(np.linalg.norm(robot.get_linear_velocity())),
                    float(np.linalg.norm(robot.get_angular_velocity())))

        def place(xyz_yaw, linear=(0., 0., 0.), yaw_rate=0.):
            bx, by, bz, byaw = xyz_yaw
            robot.set_world_pose(position=np.array([bx, by, bz]),
                                 orientation=np.array([math.cos(byaw/2), 0., 0., math.sin(byaw/2)]))
            robot.set_linear_velocity(np.array(linear, float))
            robot.set_angular_velocity(np.array([0., 0., yaw_rate]))

        motion_goal = list(cfg['tower_base_xyz_yaw'])
        tower_waypoints = cfg['tower_arm_waypoints_rad']

        def start_base_path(t0):
            # 연속 이동(바퀴 구동 아님): 물리 스텝마다 직선·등속으로 차체 자세를 옮긴다.
            current_pos, current_yaw = base_pose()
            start = np.r_[current_pos, current_yaw]
            end = np.array(motion_goal, float)
            dyaw = math.atan2(math.sin(end[3]-start[3]), math.cos(end[3]-start[3]))
            motion = cfg['base_motion']
            duration = max(np.linalg.norm(end[:2]-start[:2]) / motion['speed_mps'],
                           abs(dyaw) / motion['yaw_rate_rad_s'], 1e-6)
            return dict(start=start, delta=np.r_[end[:3]-start[:3], dyaw], t0=t0, duration=duration)

        def step_base_path(path, t):
            frac = min(1., (t-path['t0']) / path['duration'])
            pose = path['start'] + path['delta']*frac
            if frac < 1.:
                v = path['delta'] / path['duration']
                place(pose, (v[0], v[1], 0.), v[3])
                return path
            place(pose)
            return None

        target, command = initial.copy(), initial.copy()
        previous = initial.copy()
        veto = ClearanceVeto(stage, robot_path)
        for _ in range(5):
            world.render()
        if cfg['record_video']:
            video = VideoSink(out, width*2, height, camera_fps)  # 왼쪽 그리퍼, 오른쪽 마스트
        session = MissionSession()
        def new_preparation():
            timeout = route.timeout_s if route else cfg['prepare_timeout_s']
            return Preparation(rule, SimpleNamespace(auto_start=True, prepare_timeout=timeout,
                search_hz=10., exposure='sim_fixed', wb='sim_fixed'))

        prep = new_preparation()
        seeker = None
        if cfg.get('barcode_seek'):  # 바코드 미확정 시 라벨 후보 방향으로 팔 보정(barcode_seek.py)
            from barcode_seek import BarcodeSeek
            seeker = BarcodeSeek(model, cfg, lower, upper, fx, width, height)
        barcode_first_s = barcode_since = None
        phase, phase_start, stable_since = ('settle_tower' if tower_only or tower_first else
                                            'settle_start' if approach else 'settle_barcode'), 0., None
        patch, waypoint, started_signal = None, 0, None
        last_decode = -1.
        sequence = cfg['sequence']  # 장면 생성·채점 전용. 인식에 넘기지 않음
        terminal_hold = None
        classified = correct = 0
        last_notice = time.monotonic()
        peak_velocity = 0.
        anchor = None  # READY 직전 차체 위치 (기록 중 밀림 검사)
        result['base_moved'] = False
        base_path = None
        # 차체는 운동학적으로 붙잡는다: 고정 바퀴 원통 접촉 떨림(B02: 5~10 mm/s)으로 정지 판정이 안 됨.
        hold_pose = list(start_base)
        for frame in range(int(cfg['max_seconds'] * camera_fps)):
            if time.monotonic()-wall_start > wall_budget_s:
                raise ValueError(f'{wall_budget_s:g} s 실행 시간 예산 초과')
            for substep in range(physics_hz // camera_fps):
                if base_path is not None:
                    base_path = step_base_path(base_path, float(world.current_time))
                    if base_path is None:
                        result['base_motion_end_s'] = float(world.current_time)
                        hold_pose = list(motion_goal)
                else:
                    place(hold_pose)
                q = np.asarray(robot.get_joint_positions(joint_indices=indices), dtype=float)
                native = np.asarray(robot.get_joint_velocities(joint_indices=indices), dtype=float)
                if not np.isfinite(q).all() or not np.isfinite(native).all():
                    raise ValueError('팔 관절 피드백 비정상')
                # 시작 정착 중 착지 충격(생성 높이 ~1 mm 낙하)은 속도 검사에서 제외. 정지 판정은 그대로.
                initial_settle = phase in ('settle_barcode', 'settle_tower', 'settle_start')
                if (np.any(q < lower-.01) or np.any(q > upper+.01)
                        or (not initial_settle and np.any(np.abs(native) > cmd_velocity*1.5 + .05))):
                    raise ValueError(f'관절 위치/속도 범위 초과: q={q.tolist()}, native_velocity={native.tolist()}')
                fd = (q - previous) * physics_hz
                peak_velocity = max(peak_velocity, float(np.abs(native).max()))
                previous = q.copy()
                t = float(world.current_time)
                lin, ang = base_speed()
                settled = (base_path is None and np.max(np.abs(q-target)) < .01 and np.max(np.abs(native)) < .02
                           and np.max(np.abs(fd)) < .02 and lin < .005 and ang < .01)
                if settled:
                    if stable_since is None:
                        stable_since = t
                else:
                    stable_since = None
                if (phase in ('prepare', 'record') or (route and phase == 'barcode')) and not settled:
                    raise ValueError('소등 기준 확보 후 팔/차체가 움직임')
                veto.check()
                command += np.clip(target-command, -cmd_velocity/physics_hz, cmd_velocity/physics_hz)
                robot.apply_action(ArticulationAction(joint_positions=command.copy(), joint_indices=indices))
                physics_log.write(json.dumps(dict(t_s=t, phase=phase, q=q.tolist(), target=target.tolist(),
                    native_velocity=native.tolist(), fd_velocity=fd.tolist(), base_lin=lin, base_ang=ang,
                    settled=bool(settled)))+'\n')
                world.step(render=False)
            t = float(world.current_time)
            pos, heading = base_pose()
            if phase in ('prepare', 'record') and anchor is not None:
                dyaw = math.atan2(math.sin(heading-anchor[1]), math.cos(heading-anchor[1]))
                if np.linalg.norm(pos[:2]-anchor[0][:2]) > .005 or abs(dyaw) > .01:
                    raise ValueError('기록 중 차체 밀림 >5 mm 또는 >0.01 rad')
            age = None if started_signal is None else t-started_signal
            truth = 'OFF' if age is None or age < 1. else (
                sequence[int((age-1.) // 2)] if age < 25. and (age-1.) % 2 < 1. else 'OFF')
            if phase == 'saved' and t-terminal_hold >= .2:
                truth = 'R'  # 음성 시험: 저장 후 점등이 기록을 지우면 안 됨
            set_signal(truth)
            world.render()
            physics_time_before_capture = float(world.current_time)
            settings = carb.settings.get_settings()
            settings.set_bool('/app/player/playSimulations', False)
            try:
                rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=False)
            finally:
                settings.set_bool('/app/player/playSimulations', True)
            if abs(float(world.current_time)-physics_time_before_capture) > 1e-9:
                raise ValueError('영상 캡처가 물리 시간을 진행시킴')
            bgr = grab(rgb_ann)          # 그리퍼: 바코드
            mast_bgr = grab(mast_ann)    # 마스트
            sig = mast_bgr if signal_camera == 'mast' else bgr  # 신호등 판독 입력
            if frame == 0:
                cv2.imwrite(str(out/'first_frame.png'), side_by_side(bgr, mast_bgr))
            color, reason, reads, box = None, None, [], None
            if phase in ('settle_barcode', 'settle_tower', 'move', 'move_base', 'move_transport',
                         'settle_start', 'approach_base', 'approach_arm', 'seek_move', 'stow_arm'):
                if t-phase_start > cfg['motion_timeout_s']:
                    raise ValueError('팔 이동/정착 시간 초과')
                if stable_since is not None and t-stable_since >= .5:
                    if phase == 'seek_move':
                        phase, barcode_since = 'barcode', t   # 보정 자세 정착 → 다시 해독
                    elif phase == 'stow_arm':
                        # 준비 시작(route.begin)·anchor·arrival_view 는 아래 공통 'prepare' 처리에서 한 번만
                        result['stow_after_barcode_settled_s'] = t
                        phase, phase_start = 'prepare', t
                    elif phase == 'settle_start':
                        # ① 팔을 접은 채 차체만 바코드 앞으로 (동시 이동은 B12 에서 관절 속도 초과)
                        motion_goal = list(cfg['barcode_base_xyz_yaw'])
                        base_path = start_base_path(t)
                        result.update(approach_start_s=t, approach_base_duration_s=base_path['duration'],
                                      approach_start_base=list(approach['start_base_xyz_yaw']))
                        phase, phase_start, stable_since = 'approach_base', t, None
                    elif phase == 'approach_base':
                        # ② 도착·정지 후 팔을 바코드 자세로
                        motion_goal = list(cfg['tower_base_xyz_yaw'])
                        target = np.array(cfg['barcode_arm_q_rad'], dtype=float)
                        result['approach_base_arrived_s'] = t
                        phase, phase_start, stable_since = 'approach_arm', t, None
                    elif phase == 'approach_arm':
                        result['approach_arm_settled_s'] = t
                        phase, phase_start = 'barcode', t
                    elif phase == 'move_transport':
                        base_path = start_base_path(t)
                        result.update(transport_pose_reached_s=t, base_moved=True,
                                      base_motion_start_s=t, base_motion_duration_s=base_path['duration'])
                        phase, phase_start, stable_since = 'move_base', t, None
                    elif phase == 'move_base':
                        if route and route.camera == 'mast':
                            # Base-only relocation: hold the original arm target throughout.
                            phase, phase_start = 'prepare', t
                        else:
                            target = np.array(tower_waypoints[0], dtype=float)
                            phase, phase_start, stable_since = 'move', t, None
                    elif phase == 'settle_tower':
                        phase, phase_start = 'prepare', t
                    elif phase == 'settle_barcode':
                        phase, phase_start = 'barcode', t
                    elif waypoint + 1 < len(tower_waypoints):
                        waypoint += 1
                        target = np.array(tower_waypoints[waypoint], dtype=float)
                        stable_since = None
                    else:
                        phase, phase_start = 'prepare', t
                    if phase == 'prepare':
                        if route:
                            route.begin(t)
                        anchor = base_pose()
                        cv2.imwrite(str(out / 'arrival_view.png'), side_by_side(bgr, mast_bgr))
            elif phase == 'barcode':
                if barcode_first_s is None:
                    barcode_first_s = barcode_since = t
                budget = cfg['barcode_seek'].get('total_timeout_s', 60.) if seeker else cfg['barcode_timeout_s']
                if t-barcode_first_s > budget:
                    if seeker:
                        result['barcode_seek'] = dict(attempts=seeker.attempts, log=seeker.log)
                    raise ValueError('바코드 확정 시간 초과')
                if seeker and session.barcode is None and seeker.due(t, barcode_since):
                    pos_now, yaw_now = base_pose()
                    new_q = seeker.propose(bgr, q, [float(pos_now[0]), float(pos_now[1]), float(pos_now[2]), yaw_now], t)
                    result['barcode_seek'] = dict(attempts=seeker.attempts, log=seeker.log)
                    if new_q is not None:
                        target = np.asarray(new_q, float)
                        phase, stable_since = 'seek_move', None
                        cv2.imwrite(str(out / f'seek_{seeker.attempts:02d}.png'), side_by_side(bgr, mast_bgr))
                if t-last_decode >= .2-1e-9:
                    decoded = decode_bgr(bgr, (0, 0), cfg['barcode_upscale'], t, True)
                    reads, last_decode = decoded['reads'], t
                    session.observe_barcodes(reads, t)
                box = tower_live_hsv.detect(sig if route else bgr)
                if session.barcode is not None and tower_first:
                    cv2.imwrite(str(out / 'barcode_saved.png'), side_by_side(bgr, mast_bgr))
                    final = session.snapshot()
                    before = result['mission_before_barcode']
                    observed = ''.join(session.recorder.sequence)
                    preserved = (final.get('sequence') == before.get('sequence')
                                 and final.get('signal_complete') and not final.get('error'))
                    result['mission'] = final
                    (out / 'mission_summary.json').write_text(json.dumps(final, indent=2, ensure_ascii=False))
                    result.update(status='passed' if observed == sequence and session.barcode == cfg['barcode']['code']
                                  and preserved and final.get('status') == 'ready' else 'failed',
                        barcode_saved_at_s=t, observed_sequence=observed, expected_sequence_for_scoring=sequence,
                        expected_barcode_for_scoring=cfg['barcode']['code'], signal_saved=preserved,
                        signal_preserved_through_motion=preserved, later_lamp_ignored=preserved,
                        classified_frames=classified, correct_color_frames=correct,
                        measured_peak_arm_velocity_rad_s=peak_velocity,
                        measured_start_q=initial.tolist(), measured_final_q=q.tolist(),
                        frame_count=frame+1, wall_s=time.monotonic()-wall_start)
                    # 확정 프레임도 영상에 남기고 2 s 더 녹화한 뒤 종료(아래 'done' 처리)
                    phase, done_at = 'done', t
                elif session.barcode is not None:  # 바코드 먼저 순서만 (tower_first 는 위에서 'done')
                    cv2.imwrite(str(out / 'barcode_saved.png'), side_by_side(bgr, mast_bgr))
                    result['barcode_saved_before_motion'] = session.barcode
                    result['barcode_saved_at_s'] = t
                    result['tower_candidate_at_barcode_pose'] = None if box is None else list(map(float, box))
                    if not route and cfg['barcode_base_xyz_yaw'] != cfg['tower_base_xyz_yaw']:
                        base_path = start_base_path(t)  # 팔과 동시에 차체 연속 이동
                        result.update(base_moved=True, base_motion_start_s=t,
                                      base_motion_duration_s=base_path['duration'])
                    if route and cfg.get('stow_arm_after_barcode_q_rad'):
                        # 바코드 먼저: 보정으로 바뀐 팔이 상단 시야를 가리지 않게 접은 뒤 신호 준비
                        target = np.array(cfg['stow_arm_after_barcode_q_rad'], dtype=float)
                        result['stow_after_barcode_start_s'] = t
                        phase, phase_start, stable_since = 'stow_arm', t, None
                    elif route:
                        # No initial motion: keep barcode pose and prepare the mast image.
                        phase, phase_start = 'prepare', t
                        anchor = base_pose()
                        route.begin(t)
                        cv2.imwrite(str(out / 'arrival_view.png'), side_by_side(bgr, mast_bgr))
                    else:
                        target = np.array(tower_waypoints[0], dtype=float)
                        phase, phase_start, stable_since = 'move', t, None
            elif phase == 'prepare':
                action = route.timeout_action(t, prep.reason) if route else None
                if action == 'fallback':
                    if started_signal is not None or session.recorder.sequence:
                        raise ValueError('Camera fallback forbidden after recording started')
                    saved_barcode = session.barcode
                    patch, anchor, box = None, None, None
                    if cfg.get('signal_patch') == 'oblique':
                        tower_oblique.uninstall()  # Drop old-view blob/ROI cache.
                        if route.camera == 'mast':
                            tower_oblique.install()
                    signal_camera = route.camera
                    prep = new_preparation()  # Fresh OFF baseline after settling at the new view.
                    motion_goal = list(cfg['signal_fallback']['base_xyz_yaw'])
                    tower_waypoints = cfg['signal_fallback']['arm_waypoints_rad']
                    waypoint = 0
                    current_pos, current_yaw = base_pose()
                    need_base_move = np.max(np.abs(np.r_[current_pos, current_yaw] - motion_goal)) > 1e-6
                    transport_q = cfg['signal_fallback'].get('transport_arm_q_rad')
                    if signal_camera == 'mast':
                        # Validation forbids arm/transport changes in this strategy.
                        if need_base_move:
                            base_path = start_base_path(t)
                            result.update(base_moved=True, base_motion_start_s=t,
                                          base_motion_duration_s=base_path['duration'])
                        phase = 'move_base'
                    elif need_base_move and transport_q is not None:
                        target = np.array(transport_q, dtype=float)
                        phase = 'move_transport'
                    else:
                        if need_base_move:
                            base_path = start_base_path(t)
                            result.update(base_moved=True, base_motion_start_s=t,
                                          base_motion_duration_s=base_path['duration'])
                        staged_base = bool(base_path is not None and cfg['signal_fallback'].get('base_before_arm', True))
                        if not staged_base:
                            target = np.array(tower_waypoints[0], dtype=float)
                        phase = 'move_base' if staged_base else 'move'
                    phase_start, stable_since = t, None
                    reason = f'mast preparation timed out; relocating for {signal_camera} observation'
                    result.update(fallback_used=True, fallback_reason=route.events[-1]['reason'],
                                  barcode_preserved_on_fallback=session.barcode == saved_barcode,
                                  barcode_saved_before_fallback=saved_barcode)
                    cv2.imwrite(str(out / 'fallback_started.png'), side_by_side(bgr, mast_bgr))
                    print(f'[fallback] t={t:.3f}s barcode={session.barcode}; {signal_camera} observation', flush=True)
                elif action == 'fail':
                    raise ValueError(f'Fallback {signal_camera} preparation timeout: {prep.reason}')
                elif not route and t-phase_start > cfg['prepare_timeout_s']:
                    raise ValueError(f'자동 준비 시간 초과: {prep.reason}')
                if action != 'fallback':
                    box, acquired = prep.step(sig, t)
                    observed_tiers = None
                    if (route and signal_camera == 'mast' and cfg.get('signal_patch') == 'oblique'
                            and cfg['signal_fallback'].get('allow_partial_sides', True)):
                        observed_tiers = tower_oblique.observed_tiers(box)
                    if route and box is not None and not route.accepts_box(box, sig.shape, observed_tiers):
                        prep.clear()
                        prep.last_box = None
                        prep.reason = 'tower clipped or too little lens area; waiting for usable view'
                        acquired = None
                    reason = prep.reason
                    if acquired:
                        patch, evidence = acquired
                        if not session.arm(t, 'OFF', confirmed_off=True, fixed_settings=True):
                            raise ValueError('기록기 arm 거부')
                        if route:
                            route.lock(t)
                        if route:
                            evidence['horizontal_crop_allowed'] = bool(box[0] < 3 or box[2] > width-3)
                            evidence['observed_tiers'] = observed_tiers
                        result['ready_evidence'] = evidence
                        cv2.imwrite(str(out / 'ready.png'), side_by_side(bgr, mast_bgr))
                        started_signal = t
                        phase, phase_start = 'record', t
            elif phase == 'record':
                box = patch.bounds
                reading = patch.read_pixels(patch.pixels(sig))
                color, reason = reading['color'], reading.get('reason')
                classified += 1
                correct += color == truth
                session.observe(t, color)
                if session.error:
                    raise ValueError(f'신호 기록 실패: {session.error}; {reason}')
                if t-phase_start > 32.:
                    raise ValueError('12회 기록 시간 초과')
                if session.recorder.complete:
                    result['mission'] = session.snapshot()
                    (out / 'mission_summary.json').write_text(json.dumps(result['mission'], indent=2, ensure_ascii=False))
                    patch, phase, terminal_hold = None, 'saved', t
                    cv2.imwrite(str(out / 'signal_saved.png'), side_by_side(bgr, mast_bgr))
            log.write(json.dumps(dict(frame=frame, t_s=t, phase=phase, color=color,
                reason=reason, reads=reads, box=None if box is None else list(map(float, box)),
                barcode=session.barcode, sequence=list(session.recorder.sequence),
                signal_camera=signal_camera, signal_route=None if route is None else route.snapshot(),
                truth_for_scoring_only=truth, joint_q=q.tolist(), native_velocity=native.tolist(),
                fd_velocity=fd.tolist(), base_xy=pos[:2].tolist(), base_yaw=heading,
                camera_world_transform=np.array(
                    UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(0.)).tolist(),
                contacts_N={name: np.asarray(probe.get_net_contact_forces(dt=1/physics_hz)).tolist()
                            for name, probe in contact_probes}), ensure_ascii=False) + '\n')
            view = side_by_side(bgr, mast_bgr)
            if box is not None and phase in ('prepare', 'record'):
                x0, y0, x1, y1 = map(int, box)
                dx = width if signal_camera == 'mast' else 0
                cv2.rectangle(view, (x0+dx, y0), (x1+dx, y1), (0, 255, 255), 1)
            cv2.putText(view, f'{phase}  barcode={session.barcode}  n={len(session.recorder.sequence)} color={color}',
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
            cv2.putText(view, 'gripper (barcode)', (8, height-10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            cv2.putText(view, f'mast {manifest["mast_camera"]["pitch_down_deg"]:.0f}deg ({"signal" if signal_camera == "mast" else "-"})',
                        (width+8, height-10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            for read in reads:  # 확정 전 해독도 표시: 읽힌 위치 + 5 s 안 같은 코드 횟수
                pts = np.int32(np.round(read['corners']))
                cv2.polylines(view, [pts], True, (0, 255, 0), 2)
                votes = sum(1 for _, code in getattr(session, 'barcode_votes', []) if code == read['text'])
                cv2.putText(view, f"read {read['text']} ({min(votes, 3)}/3)", (8, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
            if video is not None:
                video.write(view)
            if phase == 'done' and t - done_at >= 2.:  # 동작 끝난 뒤 2 s 여유 녹화
                break
            if time.monotonic()-last_notice >= 20. or frame == 0:
                print(f'frame={frame} sim={t:.3f}s phase={phase} barcode={session.barcode} '
                      f'sequence={"".join(session.recorder.sequence)} wall={time.monotonic()-wall_start:.1f}s', flush=True)
                last_notice = time.monotonic()
            if route:
                result['signal_route'] = route.snapshot()
                result['signal_camera'] = signal_camera
            if tower_first and phase == 'saved' and t-terminal_hold >= 2.:
                # 12회 저장(추가 점등 무시 확인 2 s) 후 바코드로 접근: 팔 접은 채 차체 → 팔 뻗기
                result['mission_before_barcode'] = session.snapshot()
                result['signal_saved_at_s'] = terminal_hold
                motion_goal = list(cfg['barcode_base_xyz_yaw'])
                base_path = start_base_path(t)
                result.update(approach_start_s=t, approach_base_duration_s=base_path['duration'],
                              approach_start_base=list(approach['start_base_xyz_yaw']), base_moved=True)
                phase, phase_start, stable_since = 'approach_base', t, None
            elif phase == 'saved' and t-terminal_hold >= 2.:
                observed = ''.join(session.recorder.sequence)
                preserved = session.snapshot() == result['mission']
                result.update(status='passed' if observed == sequence and (tower_only or session.barcode == cfg['barcode']['code']) and preserved else 'failed',
                    observed_sequence=observed, expected_sequence_for_scoring=sequence,
                    expected_barcode_for_scoring=None if tower_only else cfg['barcode']['code'],
                    signal_saved=preserved, later_lamp_ignored=preserved,
                    classified_frames=classified, correct_color_frames=correct,
                    measured_peak_arm_velocity_rad_s=peak_velocity,
                    measured_start_q=initial.tolist(), measured_final_q=q.tolist(),
                    frame_count=frame+1, wall_s=time.monotonic()-wall_start,
                    initial_tower_absence_requires_video_review=True)
                break
        else:
            raise ValueError('전체 시뮬 시간 예산 소진')
    except BaseException as exc:
        if route:
            result['signal_route'] = route.snapshot()
            result['signal_camera'] = route.camera
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        if 'mast_ann' in locals():
            try:
                cv2.imwrite(str(out/'failure.png'), side_by_side(grab(rgb_ann), grab(mast_ann)))
            except Exception:
                pass
        raise
    finally:
        log.close()
        physics_log.close()
        if video is not None:
            video.release()
        (out / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        try:
            if world is not None:
                world.stop()
        finally:
            if app is not None:
                app.close(exit_code=0 if result['status'] == 'passed' else 1)
    if result['status'] != 'passed':
        raise SystemExit('채점 실패: result.json 과 camera.mp4 확인')


if __name__ == '__main__':
    main()
