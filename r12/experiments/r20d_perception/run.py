#!/usr/bin/env python3
"""Isolated R20D + R12 arm, wrist RGB -> barcode -> arm move -> tower trial.

No ROS, real camera, navigation or driving checkpoint is started. Mandatory
mount and observation poses are supplied through configuration. Local trials
are documented in README.md. All recognition consumes rendered pixels.
"""
import argparse
import json
import os
import shutil
import time
from pathlib import Path
import sys
from types import SimpleNamespace

from common import R12, load_config, sha256
from appearance import asset_paths, apply_floor, apply_gripper


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    ap.add_argument('--prepared', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path, help='New results directory')
    args = ap.parse_args()
    cfg, limits = load_config(args.config)
    tower_only = cfg.get('trial_mode') == 'tower_only'
    prepared = args.prepared.expanduser().resolve()
    manifest = json.loads((prepared / 'manifest.json').read_text())
    if manifest['config'] != cfg or manifest['composite_urdf_sha256'] != sha256(prepared / 'r20d_perception.urdf'):
        raise ValueError('Prepared assets/config differ; prepare again into a new directory')
    if (manifest['archive_sha256'] != sha256(cfg['r20d_archive'])
            or manifest['r12_arm_urdf_sha256'] != sha256(R12 / 'models/r12/r12_mobile_gripper.urdf')):
        raise ValueError('Source CAD/arm changed since preparation')
    if any(sha256(path) != digest for path, digest in manifest['mesh_sha256'].items()):
        raise ValueError('Prepared/source meshes changed since preparation')
    if manifest.get('appearance_sha256', {}) != {str(p): sha256(p) for p in asset_paths(cfg)}:
        raise ValueError('Appearance assets changed since preparation')
    # Import image-only helpers. These imports do not start RealSense/main().
    os.environ['R12_SRC'] = str(R12 / 'src')
    sys.path.insert(0, str(Path(cfg['perception_root']) / 'scripts'))
    from live_mission_optimized import Preparation, load_color_rule, decode_bgr
    from test_mission_camera_isaac import VideoSink
    from mission_summary import MissionSession
    import tower_live_hsv
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    provenance = {str(p): sha256(p) for p in Path(__file__).parent.glob('*.py')}
    provenance.update({str(p): sha256(p) for p in asset_paths(cfg)})
    for p in (Path(cfg['perception_root']) / 'scripts').glob('*.py'):
        provenance[str(p)] = sha256(p)
    for p in (Path(cfg['perception_root']) / 'src/arm_practice').glob('*.py'):
        provenance[str(p)] = sha256(p)
    for p in (R12 / 'configs/sensors/camera_rig.json', R12 / 'configs/r12_robot.json',
              R12 / 'models/navigation/mission_scene.usda', R12 / 'src/arm_practice/mission.py',
              Path(cfg['perception_root']) / 'configs/barcodes.json',
              Path(cfg['perception_root']) / 'references' / (cfg['barcode']['code'] + '.png')):
        provenance[str(p)] = sha256(p)
    (out / 'inputs.json').write_text(json.dumps(dict(config=cfg, prepared=manifest, source_hashes=provenance), indent=2))
    snapshots = out / 'sources'
    snapshots.mkdir()
    for source, digest in provenance.items():
        path = Path(source)
        if path.suffix in ('.py', '.json'):
            shutil.copy2(path, snapshots / (digest[:12] + '-' + path.name))
    result = dict(status='not_completed', scope=cfg['scope'], rule_status='후보 규칙, 실사 정답 영상 검증 전',
                  driving_tested=False, learned_policy_tested=False, physical_robot_tested=False,
                  trial_mode=cfg.get('trial_mode', 'barcode_then_tower'), barcode_tested=not tower_only)
    # Keep Isaac GUI/headless policy of the cloned repo: headless only.
    app = video = world = None
    camera_fps = cfg.get('camera_fps', 30)
    physics_hz = cfg.get('physics_hz', 120)
    wall_start = time.monotonic()
    log = (out / 'frames.jsonl').open('w', buffering=1)
    physics_log = (out / 'physics.jsonl').open('w', buffering=1)
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'renderer': cfg['renderer'],
                             'samples_per_pixel_per_frame': 8, 'width': 640, 'height': 480})
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
        from scene import build_fixture, ClearanceVeto
        import carb
        carb.settings.get_settings().set_bool('/rtx/post/histogram/enabled', False)

        usd = URDFImporter(URDFImporterConfig(
            urdf_path=str(prepared / 'r20d_perception.urdf'), usd_path=str(out / 'imported'),
            fix_base=True, merge_fixed_joints=False, allow_self_collision=True,
            joint_drive_type='force', joint_target_type='position',
            override_joint_stiffness=40., override_joint_damping=4.)).import_urdf()
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
        spawn = UsdGeom.Xform.Define(stage, '/World/Spawn')
        x, y, z, yaw = cfg['spawn_xyz_yaw_rad']
        spawn.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        spawn.AddRotateZOp().Set(float(np.degrees(yaw)))
        robot_path = '/World/Spawn/Robot'
        robot_prim = stage.DefinePrim(robot_path, 'Xform')
        robot_prim.GetReferences().AddReference(usd)
        # Isaac 6.1's new importer authors selectable physics payloads and
        # leaves the selection empty. Select PhysX before querying bodies.
        variants = robot_prim.GetVariantSet('Physics')
        if 'physx' not in variants.GetVariantNames():
            raise ValueError('Imported asset has no PhysX payload variant')
        variants.SetVariantSelection('physx')
        app.update()
        if cfg.get('black_gripper', False):
            result['gripper_visual'] = apply_gripper(stage, robot_prim)
        # Supplied R20D CAD uses coarse touching bracket/gear boxes and asks
        # for internal chassis collision filtering. Keep arm self collision
        # enabled, filtering only the nine supplied chassis links among themselves.
        chassis_names = {'base_link'} | {f'{m}_{part}' for m in ('lh1','rh1','lh2','rh2') for part in ('steer','wheel')}
        chassis_links = [p for p in Usd.PrimRange(robot_prim) if p.GetName() in chassis_names]
        for prim in chassis_links:
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
                [p.GetPath() for p in chassis_links if p != prim])
        # A world-to-base fixed joint uses world coordinates for its body0
        # frame. Move that anchor together with the configured spawn.
        for prim in Usd.PrimRange(robot_prim):
            if prim.IsA(UsdPhysics.FixedJoint):
                joint = UsdPhysics.FixedJoint(prim)
                if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                    joint.GetLocalPos0Attr().Set(Gf.Vec3f(x, y, z))
                    joint.GetLocalRot0Attr().Set(Gf.Quatf(float(np.cos(yaw/2)), Gf.Vec3f(0., 0., float(np.sin(yaw/2)))))
        roots = [p for p in Usd.PrimRange(robot_prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise ValueError(f'Expected one articulation root, got {[str(p.GetPath()) for p in roots]}')
        robot = world.scene.add(SingleArticulation(str(roots[0].GetPath()), name='r20d'))
        physx = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
        physx.CreateSolverPositionIterationCountAttr(32)
        physx.CreateSolverVelocityIterationCountAttr(16)
        # Provisional CAD camera transform, optical XYZ -> USD camera axes.
        rig = json.loads((R12 / 'configs/sensors/camera_rig.json').read_text())
        wrist = rig['arm_camera']
        links = [p for p in Usd.PrimRange(robot_prim) if p.GetName() == wrist['parent']]
        if len(links) != 1:
            raise ValueError('Expected one arm_link5 in imported robot')
        cam_path = str(links[0].GetPath()) + '/perception_camera'
        cam = UsdGeom.Camera.Define(stage, cam_path)
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', wrist['optical_rpy_rad']).as_matrix() @ np.diag([1., -1., -1.])
        transform[:3, 3] = wrist['optical_center_xyz_m']
        cam.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
        fx, fy, cx, cy = rig['intrinsics']
        width, height = rig['resolution']
        if (width, height) != (640, 480) or fx != fy or abs(cx-319.5) > .01 or abs(cy-239.5) > .01:
            raise ValueError('This trial currently supports only symmetric uniform640 wrist intrinsics')
        cam.CreateFocalLengthAttr(20.)
        cam.CreateHorizontalApertureAttr(20. * width / fx)
        cam.CreateVerticalApertureAttr(20. * height / fy)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 30.))
        rep.orchestrator.set_capture_on_play(False)
        product = rep.create.render_product(cam_path, (width, height))
        rgb_ann = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb_ann.attach(product)
        # Force limits/velocity limits originate from existing arm configuration.
        for prim in Usd.PrimRange(robot_prim):
            if prim.GetName() in limits['joint_names']:
                i = limits['joint_names'].index(prim.GetName())
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                drive.CreateMaxForceAttr(float(limits['effort'][i]))
                drive.CreateStiffnessAttr(float(limits['stiffness'][i]))
                drive.CreateDampingAttr(float(limits['damping'][i]))
        world.reset()
        result['dof_names'] = list(robot.dof_names)
        from isaacsim.core.prims import RigidPrim
        contact_probes = []
        for prim in Usd.PrimRange(robot_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName().startswith(('arm_link', 'gripper_')):
                probe = RigidPrim(prim_paths_expr=str(prim.GetPath()), name='contact_'+prim.GetName(),
                    track_contact_forces=True, reset_xform_properties=False)
                probe.initialize()
                contact_probes.append((prim.GetName(), probe))
        indices = np.array([robot.get_dof_index(n) for n in limits['joint_names']])
        initial = np.array(cfg['tower_arm_waypoints_rad'][-1] if tower_only else cfg['barcode_arm_q_rad'], dtype=float)
        robot.set_joint_positions(initial, joint_indices=indices)  # initial pose only
        robot.set_joint_velocities(np.zeros(5), joint_indices=indices)
        # Keep the jaws open within their URDF travel, rather than pressing
        # their default zero targets past the positive/negative hard stops.
        for name, value in [('gripper_left_joint', .020), ('gripper_right_joint', -.020)]:
            if name in robot.dof_names:
                gi = np.array([robot.get_dof_index(name)])
                robot.set_joint_positions(np.array([value]), joint_indices=gi)
                robot.set_joint_velocities(np.zeros(1), joint_indices=gi)
                robot.apply_action(ArticulationAction(joint_positions=np.array([value]), joint_indices=gi))
        target, command = initial.copy(), initial.copy()
        previous = initial.copy()
        veto = ClearanceVeto(stage, robot_path)
        for _ in range(5):
            world.render()
        if cfg['record_video']:
            video = VideoSink(out, width, height, camera_fps)
        session = MissionSession()
        prep = Preparation(rule, SimpleNamespace(auto_start=True,
            prepare_timeout=cfg['prepare_timeout_s'], search_hz=10., exposure='sim_fixed', wb='sim_fixed'))
        phase, phase_start, stable_since = ('settle_tower' if tower_only else 'settle_barcode'), 0., None
        patch, waypoint, started_signal = None, 0, None
        last_decode = -1.
        sequence = cfg['sequence']  # fixture ONLY; never passed into recognition
        terminal_hold = None
        classified = correct = 0
        last_notice = time.monotonic()
        peak_velocity = 0.
        for frame in range(int(cfg['max_seconds'] * camera_fps)):
            if time.monotonic()-wall_start > 900.:
                raise ValueError('900 s wall time budget exceeded')
            # Both native and same-step FD velocity
            # must remain below .02 rad/s continuously for .5s before capture.
            for substep in range(physics_hz // camera_fps):
                q = np.asarray(robot.get_joint_positions(joint_indices=indices), dtype=float)
                native = np.asarray(robot.get_joint_velocities(joint_indices=indices), dtype=float)
                if not np.isfinite(q).all() or not np.isfinite(native).all():
                    raise ValueError('Invalid arm feedback')
                if (np.any(q < np.asarray(limits['lower'])-.01)
                        or np.any(q > np.asarray(limits['upper'])+.01)
                        or np.any(np.abs(native) > np.asarray(limits['velocity']) * 1.1 + .02)):
                    raise ValueError(f'Measured arm position/velocity outside configured limits: q={q.tolist()}, native_velocity={native.tolist()}')
                fd = (q - previous) * physics_hz
                peak_velocity = max(peak_velocity, float(np.abs(native).max()))
                previous = q.copy()
                t = float(world.current_time)
                settled = (np.max(np.abs(q-target)) < .01
                           and np.max(np.abs(native)) < .02 and np.max(np.abs(fd)) < .02)
                if settled:
                    if stable_since is None:
                        stable_since = t
                else:
                    stable_since = None
                if phase in ('prepare', 'record') and not settled:
                    raise ValueError('Arm moved after OFF reference preparation')
                veto.check()
                command += np.clip(target-command, -np.asarray(limits['velocity']) / physics_hz,
                                   np.asarray(limits['velocity']) / physics_hz)
                robot.apply_action(ArticulationAction(joint_positions=command.copy(), joint_indices=indices))
                physics_log.write(json.dumps(dict(t_s=t, phase=phase, q=q.tolist(), target=target.tolist(),
                    native_velocity=native.tolist(), fd_velocity=fd.tolist(), settled=bool(settled)))+'\n')
                world.step(render=False)
            t = float(world.current_time)
            # Fixture waits for image-based preparation AND actual arm settling.
            age = None if started_signal is None else t-started_signal
            truth = 'OFF' if age is None or age < 1. else (
                sequence[int((age-1.) // 2)] if age < 25. and (age-1.) % 2 < 1. else 'OFF')
            if phase == 'saved' and t-terminal_hold >= .2:
                truth = 'R'  # negative test: a later lamp must not erase saved signal
            set_signal(truth)
            # World.render refreshes Fabric/RTX with physics explicitly paused
            # for that refresh. Do not let Replicator perform additional
            # zero-dt physics steps between native/FD feedback samples.
            world.render()
            physics_time_before_capture = float(world.current_time)
            settings = carb.settings.get_settings()
            settings.set_bool('/app/player/playSimulations', False)
            try:
                rep.orchestrator.step(rt_subframes=2, delta_time=0., pause_timeline=False)
            finally:
                settings.set_bool('/app/player/playSimulations', True)
            if abs(float(world.current_time)-physics_time_before_capture) > 1e-9:
                raise ValueError('Image capture unexpectedly advanced physics')
            data = rgb_ann.get_data()
            rgb = np.asarray(data.get('data') if isinstance(data, dict) else data)
            if rgb.dtype != np.uint8 or rgb.shape[:2] != (height, width) or rgb.ndim != 3:
                raise ValueError(f'Invalid rendered RGB: {rgb.shape} / {rgb.dtype}')
            bgr = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
            if frame == 0:
                cv2.imwrite(str(out/'first_frame.png'), bgr)
            color, reason, reads, box = None, None, [], None
            if phase in ('settle_barcode', 'settle_tower', 'move'):
                if t-phase_start > cfg['motion_timeout_s']:
                    raise ValueError('Arm motion/settling timeout')
                if stable_since is not None and t-stable_since >= .5:
                    if phase == 'settle_tower':
                        phase, phase_start = 'prepare', t
                    elif phase == 'settle_barcode':
                        phase, phase_start = 'barcode', t
                    elif waypoint + 1 < len(cfg['tower_arm_waypoints_rad']):
                        waypoint += 1
                        target = np.array(cfg['tower_arm_waypoints_rad'][waypoint], dtype=float)
                        stable_since = None
                    else:
                        phase, phase_start = 'prepare', t
            elif phase == 'barcode':
                if t-phase_start > cfg['barcode_timeout_s']:
                    raise ValueError('Barcode acquisition timeout')
                if t-last_decode >= .2-1e-9:
                    decoded = decode_bgr(bgr, (0, 0), cfg['barcode_upscale'], t, True)
                    reads, last_decode = decoded['reads'], t
                    session.observe_barcodes(reads, t)
                # Separate setup diagnostic; finding no body is not proof that
                # tower is physically out of view. Video requires review.
                box = tower_live_hsv.detect(bgr)
                if session.barcode is not None:
                    cv2.imwrite(str(out / 'barcode_saved.png'), bgr)
                    result['barcode_saved_before_motion'] = session.barcode
                    result['tower_candidate_at_barcode_pose'] = None if box is None else list(map(float, box))
                    target = np.array(cfg['tower_arm_waypoints_rad'][0], dtype=float)
                    phase, phase_start, stable_since = 'move', t, None
            elif phase == 'prepare':
                if t-phase_start > cfg['prepare_timeout_s']:
                    raise ValueError(f'Automatic preparation timeout: {prep.reason}')
                box, acquired = prep.step(bgr, t)
                reason = prep.reason
                if acquired:
                    patch, evidence = acquired
                    if not session.arm(t, 'OFF', confirmed_off=True, fixed_settings=True):
                        raise ValueError('Recorder arm rejected')
                    result['ready_evidence'] = evidence
                    cv2.imwrite(str(out / 'ready.png'), bgr)
                    # READY is external to the fixture; fixture starts one
                    # second later. No inferred-OFF shortcut or expected ROI.
                    started_signal = t
                    phase, phase_start = 'record', t
            elif phase == 'record':
                box = patch.bounds
                reading = patch.read_pixels(patch.pixels(bgr))
                color, reason = reading['color'], reading.get('reason')
                classified += 1
                correct += color == truth
                session.observe(t, color)
                if session.error:
                    raise ValueError(f'Signal recording failed: {session.error}; {reason}')
                if t-phase_start > 32.:
                    raise ValueError('12-flash recording timeout')
                if session.recorder.complete:
                    result['mission'] = session.snapshot()
                    (out / 'mission_summary.json').write_text(json.dumps(result['mission'], indent=2, ensure_ascii=False))
                    # Freeze the valid record, drop fixed pixel ROI, ignore
                    # further lamp activity. Keep video for another 2 seconds.
                    patch, phase, terminal_hold = None, 'saved', t
                    cv2.imwrite(str(out / 'signal_saved.png'), bgr)
            log.write(json.dumps(dict(frame=frame, t_s=t, phase=phase, color=color,
                reason=reason, reads=reads, box=None if box is None else list(map(float, box)),
                barcode=session.barcode, sequence=list(session.recorder.sequence),
                truth_for_scoring_only=truth, joint_q=q.tolist(), native_velocity=native.tolist(),
                fd_velocity=fd.tolist(), camera_world_transform=np.array(
                    UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(0.)).tolist(),
                contacts_N={name: np.asarray(probe.get_net_contact_forces(dt=1/physics_hz)).tolist()
                            for name, probe in contact_probes}), ensure_ascii=False) + '\n')
            cv2.putText(bgr, f'{phase}  barcode={session.barcode}  n={len(session.recorder.sequence)} color={color}',
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
            if video is not None:
                video.write(bgr)
            if time.monotonic()-last_notice >= 20. or frame == 0:
                print(f'frame={frame} sim={t:.3f}s phase={phase} barcode={session.barcode} sequence={"".join(session.recorder.sequence)} wall={time.monotonic()-wall_start:.1f}s', flush=True)
                last_notice = time.monotonic()
            if phase == 'saved' and t-terminal_hold >= 2.:
                observed = ''.join(session.recorder.sequence)
                preserved = session.snapshot() == result['mission']
                result.update(status='passed' if observed == sequence and (tower_only or session.barcode == cfg['barcode']['code']) and preserved else 'failed',
                    observed_sequence=observed, expected_sequence_for_scoring=sequence,
                    expected_barcode_for_scoring=None if tower_only else cfg['barcode']['code'],
                    signal_saved=preserved, barcode_preserved_through_motion=None if tower_only else True,
                    later_lamp_ignored=preserved,
                    classified_frames=classified, correct_color_frames=correct,
                    measured_peak_arm_velocity_rad_s=peak_velocity,
                    measured_start_q=initial.tolist(), measured_final_q=q.tolist(),
                    frame_count=frame+1, wall_s=time.monotonic()-wall_start,
                    initial_tower_absence_requires_video_review=True)
                break
        else:
            raise ValueError('Overall simulation time budget exhausted')
    except BaseException as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        if 'rgb_ann' in locals():
            raw = rgb_ann.get_data()
            raw = raw.get('data') if isinstance(raw, dict) else raw
            pixels = np.asarray(raw)
            if pixels.ndim == 3 and pixels.shape[2] >= 3:
                cv2.imwrite(str(out/'failure.png'), cv2.cvtColor(pixels[:,:,:3], cv2.COLOR_RGB2BGR))
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
                app.close(exit_code=0 if result['status']=='passed' else 1)
    if result['status'] != 'passed':
        raise SystemExit('Scoring failed; inspect result.json and camera.mp4')


if __name__ == '__main__':
    main()
