#!/usr/bin/env python3
"""Pixel-only configurable-resolution field diagnostic; no robot or physical camera.

Barcode native resolution, tower resized to signal-width, shared live helpers.
Camera viewpoints are fixture inputs, NOT physical arm motion. Exposure/WB are
recorded real-camera settings, not applied as equivalent RTX parameters.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess
import time
from types import SimpleNamespace

HERE = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class VideoSink:
    """System FFmpeg encoder; Isaac's OpenCV build may lack video codecs."""
    def __init__(self, out, width, height, fps):
        self.log = (out / 'encoder.log').open('wb')
        self.process = subprocess.Popen([
            '/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin',
            '-f', 'rawvideo', '-pixel_format', 'bgr24', '-video_size', f'{width}x{height}',
            '-framerate', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-threads', '2', '-preset', 'ultrafast', '-crf', '22', '-pix_fmt', 'yuv420p',
            str(out / 'camera.mp4')], stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=self.log)

    def write(self, frame):
        if self.process.poll() is not None:
            raise RuntimeError('FFmpeg stopped; inspect encoder.log')
        self.process.stdin.write(frame.tobytes())

    def release(self):
        try:
            if self.process.stdin and not self.process.stdin.closed:
                self.process.stdin.close()
            if self.process.wait(timeout=30) != 0:
                raise RuntimeError('FFmpeg encoding failed; inspect encoder.log')
        finally:
            self.log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--r12-root', type=Path, default=Path('~/git/lv2-nz2-project2').expanduser() / 'r12')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--signal-width', type=int, default=640)
    p.add_argument('--barcode-upscale', type=int, choices=[1, 2], default=2)
    p.add_argument('--barcode', choices=['P2-01', 'P2-02', 'P2-03', 'P2-04'], default='P2-02')
    p.add_argument('--barcode-distance', type=float, default=.15)
    p.add_argument('--tower-distance', type=float, default=.30)
    p.add_argument('--exposure', type=float, default=150.)
    p.add_argument('--wb', type=float, default=4600.)
    p.add_argument('--max-seconds', type=float, default=60.)
    p.add_argument('--smoke', action='store_true', help='30 render frames, no full sequence score')
    p.add_argument('--auto-start', action='store_true', help='Fixture keeps tower OFF until READY; always required')
    a = p.parse_args()
    if not a.auto_start:
        p.error('--auto-start required: fixture guarantees OFF until READY')
    if (a.width < 320 or a.height < 240 or a.fps not in (20, 30)
            or not 0 < a.signal_width <= a.width or not .05 <= a.barcode_distance <= 2.
            or not .1 <= a.tower_distance <= 2. or not 1 <= a.max_seconds <= 120.):
        p.error('Invalid resolution, FPS, distance or time budget')
    # One instance of this diagnostic at a time; lock held through app close.
    lock = Path('/tmp/tower-1080p-isaac.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    a.r12_root = a.r12_root.resolve()
    os.environ['R12_SRC'] = str(a.r12_root / 'src')
    sys.path.insert(0, str(a.r12_root / 'experiments/r20d_perception'))
    from live_mission_optimized import Preparation, load_color_rule, decode_bgr
    from mission_summary import MissionSession
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    out = a.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    fixture_cfg = dict(
        barcode=dict(code=a.barcode, center_m=[.08, .65, .45]), dome=400., emissive=1.)
    sequence = 'RGYRGYGRGRGG'
    source_paths = [
        Path(__file__), Path(__file__).with_name('live_mission_optimized.py'),
        Path(__file__).with_name('tower_live_hsv.py'),
        Path(__file__).with_name('mission_summary.py'),
        Path(__file__).with_name('barcode_support.py'),
        HERE / 'src/arm_practice/tower_signal.py',
        HERE / 'configs/barcodes.json', HERE / 'references' / (a.barcode + '.png'),
        a.r12_root / 'experiments/r20d_perception/scene.py',
        a.r12_root / 'models/navigation/mission_scene.usda',
        a.r12_root / 'src/arm_practice/mission.py']
    inputs = dict(settings={k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()},
                  fixture=fixture_cfg, source_sha256={str(x): digest(x) for x in source_paths},
                  truth_sequence_for_scoring=sequence,
                  renderer='PathTracing', subframes=2, samples_per_pixel_per_frame=8,
                  camera_model='nominal horizontal FOV from 640/fx400; NOT measured D435 1080p intrinsics',
                  real_exposure_wb_applied_to_rtx=False,
                  scope='field_camera_view_change_no_robot_no_physical_motion',
                  wall_budget_s=900, source_frozen=True)
    (out / 'inputs.json').write_text(json.dumps(inputs, indent=2))
    import shutil
    snapshots = out / 'sources'
    snapshots.mkdir()
    for source in source_paths:
        if source.suffix in ('.py', '.json'):
            shutil.copy2(source, snapshots / (digest(source)[:12] + '-' + source.name))
    result = dict(status='not_completed', scope=inputs['scope'], robot_tested=False,
                  real_camera_equivalence=False, exposure_wb_equivalence=False,
                  rule_status='후보 규칙, 실사 정답 영상 검증 전')
    log = (out / 'frames.jsonl').open('w', buffering=1)
    app = video = None
    start_wall = time.monotonic()
    phase = 'startup'
    try:
        from isaacsim import SimulationApp
        app = SimulationApp(dict(headless=True, renderer='PathTracing', anti_aliasing=0,
                                  samples_per_pixel_per_frame=8, width=640, height=360, fast_shutdown=True))
        import omni.usd
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        from scene import build_fixture
        import carb
        # Fixed renderer tonemapping; real D435 exposure/WB units do not map.
        settings = carb.settings.get_settings()
        settings.set_bool('/rtx/post/histogram/enabled', False)
        result['histogram_enabled_readback'] = settings.get('/rtx/post/histogram/enabled')
        omni.usd.get_context().new_stage()
        app.update()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, 'Z')
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        rule = load_color_rule()
        set_signal = build_fixture(stage, fixture_cfg, rule)
        cam = UsdGeom.Camera.Define(stage, '/World/DiagnosticCamera')
        cam.CreateFocalLengthAttr(13.096875)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(20.955 * a.height / a.width)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 30.))
        camera_op = cam.AddTransformOp()

        def point_camera(eye, target):
            eye, target = np.asarray(eye, float), np.asarray(target, float)
            forward = target-eye
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0., 0., 1.])
            right /= np.linalg.norm(right)
            up = np.cross(right, forward)
            T = np.eye(4)
            T[:3, :3] = np.column_stack([right, up, -forward])
            T[:3, 3] = eye
            camera_op.Set(Gf.Matrix4d(T.T.tolist()))

        label = np.array(fixture_cfg['barcode']['center_m'])
        point_camera(label + [a.barcode_distance, 0., 0.], label)
        rep.orchestrator.set_capture_on_play(False)
        product = rep.create.render_product('/World/DiagnosticCamera', (a.width, a.height))
        ann = rep.AnnotatorRegistry.get_annotator('rgb')
        ann.attach(product)
        video = VideoSink(out, a.width, a.height, a.fps)
        session = MissionSession()
        prep = Preparation(rule, SimpleNamespace(auto_start=True, prepare_timeout=15.,
                           search_hz=10., exposure=a.exposure, wb=a.wb))
        phase, phase_started = 'barcode', 0.
        signal_start = saved_at = patch = None
        last_decode = -1.
        barcode_attempts = barcode_hit_frames = 0
        color_checked = color_correct = 0
        unknown_reasons = {}
        frame_total = 30 if a.smoke else int(a.max_seconds*a.fps)
        last_notice = time.monotonic()
        for frame in range(frame_total):
            if time.monotonic()-start_wall > 900:
                raise RuntimeError('900 s wall budget exceeded')
            t = frame/a.fps
            age = None if signal_start is None else t-signal_start
            truth = 'OFF'
            if age is not None and 1. <= age < 25. and (age-1.) % 2. < 1.:
                truth = sequence[min(11, int((age-1.) // 2))]
            if phase == 'saved' and t-saved_at >= .2:
                truth = 'R'  # Must not reset an already saved sequence.
            set_signal(truth)
            tick = time.monotonic()
            rep.orchestrator.step(rt_subframes=2, delta_time=1/a.fps, pause_timeline=True)
            data = ann.get_data()
            rgb = np.asarray(data.get('data') if isinstance(data, dict) else data)
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[:2] != (a.height, a.width):
                raise RuntimeError(f'Unexpected RGB {rgb.shape}/{rgb.dtype}')
            native = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
            if frame == 0:
                cv2.imwrite(str(out/'first_frame.png'), native)
            scale = min(1., a.signal_width/a.width)
            work = cv2.resize(native, (round(a.width*scale), round(a.height*scale)),
                              interpolation=cv2.INTER_AREA) if scale < 1 else native
            reads, color, reason, box = [], None, None, None
            if phase == 'barcode':
                if t-phase_started > 15.:
                    raise ValueError('Barcode acquisition timeout')
                if t-last_decode >= .2-1e-9:
                    decoded = decode_bgr(native, (0, 0), a.barcode_upscale, t, True)
                    reads, last_decode = decoded['reads'], t
                    barcode_attempts += 1
                    barcode_hit_frames += bool(reads)
                    session.observe_barcodes(reads, t)
                if session.barcode is not None:
                    cv2.imwrite(str(out/'barcode_saved.png'), native)
                    result.update(barcode_saved=session.barcode, barcode_saved_at_s=t)
                    center_z = rule.TOWER['base_z_m'] + sum(
                        [min(x[0] for x in rule.TOWER['tiers'].values()),
                         max(x[1] for x in rule.TOWER['tiers'].values())])/2
                    tx, ty = rule.TOWER['xy_m']
                    point_camera([tx+a.tower_distance, ty, center_z+.08], [tx, ty, center_z])
                    phase, phase_started = 'settle_view', t
            elif phase == 'settle_view':
                # Fixed-camera render settle ONLY, not actual arm settling.
                if t-phase_started >= .5:
                    phase, phase_started = 'prepare', t
            elif phase == 'prepare':
                box, acquired = prep.step(work, t)
                reason = prep.reason
                if acquired:
                    patch, evidence = acquired
                    if not session.arm(t, 'OFF', confirmed_off=True, fixed_settings=True):
                        raise ValueError('Recorder arm rejected')
                    result.update(ready_at_s=t, ready_evidence=evidence)
                    cv2.imwrite(str(out/'ready.png'), native)
                    phase, phase_started, signal_start = 'record', t, t
            elif phase == 'record':
                reading = patch.read_pixels(patch.pixels(work))
                color, reason = reading['color'], reading.get('reason')
                box = patch.bounds
                color_checked += 1
                color_correct += color == truth
                if color == 'UNKNOWN':
                    unknown_reasons[reason] = unknown_reasons.get(reason, 0)+1
                session.observe(t, color)
                if session.error:
                    cv2.imwrite(str(out/'failure.png'), native)
                    raise ValueError(f'{session.error}; classifier reason={reason}')
                if t-phase_started > 32.:
                    raise ValueError('Signal recording timeout')
                if session.recorder.complete:
                    result['mission'] = session.snapshot()
                    (out/'mission_summary.json').write_text(json.dumps(result['mission'], indent=2, ensure_ascii=False))
                    cv2.imwrite(str(out/'signal_saved.png'), native)
                    patch, phase, saved_at = None, 'saved', t
            elapsed = time.monotonic()-tick
            row = dict(frame=frame, t_s=t, phase=phase, color=color, reason=reason,
                       reads=reads, barcode=session.barcode, sequence=list(session.recorder.sequence),
                       truth_for_scoring_only=truth, frame_wall_s=elapsed,
                       box=None if box is None else list(map(float, box)))
            log.write(json.dumps(row)+'\n')
            if box is not None:
                x0, y0, x1, y1 = map(lambda v: int(round(v/scale)), box[:4])
                cv2.rectangle(native, (x0, y0), (x1, y1), (0, 220, 255), 2)
            cv2.putText(native, f'{phase} barcode={session.barcode} n={len(session.recorder.sequence)} color={color}',
                        (12, 35), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
            cv2.putText(native, f'SIM {a.fps}Hz | WALL {1/max(elapsed,1e-6):.1f}fps | NO ROBOT',
                        (12, 70), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            video.write(native)
            if time.monotonic()-last_notice > 20 or frame == 0:
                print(f'frame={frame} sim={t:.2f}s phase={phase} barcode={session.barcode} sequence={"".join(session.recorder.sequence)} wall={time.monotonic()-start_wall:.1f}s', flush=True)
                last_notice = time.monotonic()
            if phase == 'saved' and t-saved_at >= 2.:
                preserved = session.snapshot() == result['mission']
                observed = ''.join(session.recorder.sequence)
                result.update(status='passed' if observed == sequence and session.barcode == a.barcode and preserved else 'failed',
                              observed_sequence=observed, expected_sequence_for_scoring=sequence,
                              saved_record_preserved_after_relight=preserved)
                break
        else:
            result.update(status='smoke_only' if a.smoke else 'failed',
                          error=None if a.smoke else 'Simulation time budget exhausted')
        result.update(barcode_attempts=barcode_attempts, barcode_hit_frames=barcode_hit_frames,
                      classified_frames=color_checked, correct_color_frames=color_correct,
                      unknown_reasons=unknown_reasons, rendered_frames=frame+1,
                      wall_s=time.monotonic()-start_wall,
                      achieved_wall_fps=(frame+1)/(time.monotonic()-start_wall))
    except BaseException as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}', phase=phase)
        raise
    finally:
        log.close()
        if video is not None:
            video.release()
        (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
        if app is not None:
            try:
                result.update(shutdown_mode='Kit default fast shutdown with cleanup', shutdown_requested=True)
                (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
                app.close(exit_code=0 if result['status'] in ('passed', 'smoke_only') else 1)
                result['shutdown_returned'] = True
            except BaseException as exc:
                result.update(status='failed', shutdown_complete=False, shutdown_error=repr(exc))
                raise
            finally:
                (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
        lock.close()
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if result['status'] not in ('passed', 'smoke_only'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
