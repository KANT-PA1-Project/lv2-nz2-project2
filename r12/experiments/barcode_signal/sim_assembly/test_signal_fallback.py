"""CPU-only routing tests: execute actual run.py phase blocks with mock motion/IO."""
import ast
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
HERE_DATA = Path(__file__).resolve().parent / 'test_data'  # reports/ 대신 저장소에 넣은 시험용 렌더 화면 4장
sys.path[:0] = [str(HERE), str(ROOT / 'scripts')]
from common import load_config
from signal_fallback import SignalFallback, validate_fallback, wall_budget
import tower_oblique as oblique
import tower_live_hsv as finder
import live_mission_optimized as live
from mission_summary import MissionSession


def phase_block(name):
    tree = ast.parse((HERE / 'run.py').read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        if name == 'prepare' and isinstance(node.test, ast.Compare):
            if (isinstance(node.test.left, ast.Name) and node.test.left.id == 'phase'
                    and isinstance(node.test.comparators[0], ast.Constant)
                    and node.test.comparators[0].value == 'prepare'
                    and any(isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'action'
                            for t in n.targets) for n in node.body)):
                return compile(ast.Module(body=node.body, type_ignores=[]), str(HERE / 'run.py'), 'exec')
        if name == 'move' and isinstance(node.test, ast.Compare):
            if isinstance(node.test.ops[0], ast.In) and isinstance(node.test.left, ast.Name) and node.test.left.id == 'phase':
                if isinstance(node.test.comparators[0], ast.Tuple) and 'settle_barcode' in ast.literal_eval(node.test.comparators[0]):
                    return compile(ast.Module(body=node.body, type_ignores=[]), str(HERE / 'run.py'), 'exec')
    raise AssertionError('Runtime phase block not found: ' + name)


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.cfg, _ = load_config(HERE / 'config_auto_signal_fallback.json')
        self.route = SignalFallback(self.cfg['signal_fallback'])

    def tearDown(self):
        oblique.uninstall()

    def environment(self):
        route = self.route
        session = MissionSession()
        for t in (0., .2, .4):
            session.observe_barcodes([dict(text='P2-02')], t)
        rule = live.load_color_rule()
        oblique.install()
        def new_preparation():
            return live.Preparation(rule, SimpleNamespace(auto_start=True, prepare_timeout=route.timeout_s,
                search_hz=10., exposure='sim_fixed', wb='sim_fixed'))
        route.begin(0.)
        b = np.zeros((480, 640, 3), np.uint8)
        return dict(route=route, cfg=self.cfg, t=0., phase='prepare', phase_start=0.,
            prep=new_preparation(), new_preparation=new_preparation, session=session,
            started_signal=None, patch=object(), anchor=object(), box=None, result={},
            signal_camera='mast', tower_oblique=oblique, np=np, cv2=SimpleNamespace(imwrite=Mock()),
            side_by_side=np.hstack, out=Path('/tmp'), bgr=b, mast_bgr=b,
            base_pose=lambda: (np.array(self.cfg['barcode_base_xyz_yaw'][:3]), 0.),
            start_base_path=lambda t: dict(duration=3.), base_path=None,
            target=np.array(self.cfg['barcode_arm_q_rad']), stable_since=None,
            motion_goal=self.cfg['tower_base_xyz_yaw'], tower_waypoints=self.cfg['tower_arm_waypoints_rad'],
            waypoint=0, sig=b, rule=rule, reason=None, width=640)

    def execute(self, env, name='prepare'):
        # Runtime overlay helper has two arguments, unlike np.hstack.
        env['side_by_side'] = lambda a, b: np.hstack([a, b])
        with contextlib.redirect_stdout(io.StringIO()):
            exec(phase_block(name), env)

    def test_existing_configs_still_validate(self):
        for p in HERE.glob('config*.json'):
            load_config(p)

    def test_reject_invalid_timeout(self):
        for value in (True, float('nan'), float('inf'), 0., 1.):
            cfg = copy.deepcopy(self.cfg)
            cfg['signal_fallback']['initial_prepare_timeout_s'] = value
            with self.assertRaises(ValueError): validate_fallback(cfg, [-4]*5, [4]*5)

    def test_reject_out_of_range_fallback_pose(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['signal_fallback']['arm_waypoints_rad'][0][0] = 100.
        with self.assertRaises(ValueError): validate_fallback(cfg, [-4]*5, [4]*5)

    def test_reject_initial_motion(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['tower_base_xyz_yaw'][0] += .1
        with self.assertRaises(ValueError): validate_fallback(cfg, [-4]*5, [4]*5)

    def test_initial_success_with_raw_B09_png(self):
        env = self.environment()
        raw = cv2.imread(str(HERE_DATA / 'B09_arrival_view.png'))[:, 640:]
        env['sig'] = raw
        for i in range(40):
            env['t'] = i / 30.
            self.execute(env)
            if env['phase'] == 'record': break
        self.assertEqual(env['phase'], 'record')
        self.assertEqual(env['signal_camera'], 'mast')
        self.assertFalse(env['result'].get('fallback_used', False))
        self.assertEqual(env['session'].barcode, 'P2-02')
        self.assertTrue(self.route.recording)
        self.assertIsNone(self.route.timeout_action(100., 'late'))

    def test_timeout_moves_once_and_keeps_barcode(self):
        env = self.environment()
        env['t'] = 1.99
        self.execute(env)
        self.assertEqual(env['phase'], 'prepare')
        env['t'] = 2.
        self.execute(env)
        self.assertEqual(env['phase'], 'move_transport')
        self.assertIsNone(env['base_path'])
        np.testing.assert_allclose(env['target'], self.cfg['signal_fallback']['transport_arm_q_rad'])
        self.assertEqual(env['signal_camera'], 'wrist')
        self.assertIsNone(env['patch'])
        self.assertIsNone(env['anchor'])
        self.assertIsNone(env['started_signal'])
        self.assertEqual(env['session'].barcode, 'P2-02')
        self.assertFalse(self.route.recording)
        self.assertIsNone(self.route.started)
        self.assertFalse(oblique._original)
        self.assertFalse(oblique._tiers)
        self.assertEqual(env['motion_goal'], self.cfg['signal_fallback']['base_xyz_yaw'])

    def test_fallback_waits_for_settling_then_uses_fresh_wrist_OFF(self):
        env = self.environment(); env['t'] = 2.; self.execute(env)
        env['t'] = 8.; self.execute(env, 'move')
        self.assertEqual(env['phase'], 'move_transport')
        self.assertIsNone(self.route.started)
        env.update(t=8.5, stable_since=8.)
        self.execute(env, 'move')
        self.assertEqual(env['phase'], 'move_base')
        self.assertIsNone(self.route.started)
        env.update(t=12., stable_since=11.5, base_path=None)
        self.execute(env, 'move')
        self.assertEqual(env['phase'], 'move')
        self.assertIsNone(self.route.started)
        env.update(t=12.5, stable_since=12.)
        self.execute(env, 'move')
        self.assertEqual(env['phase'], 'prepare')
        self.assertEqual(self.route.started, 12.5)
        raw = cv2.imread(str(HERE_DATA / 'B08_ready.png'))[:, :640]
        env['sig'] = raw
        for i in range(1, 45):
            env['t'] = 12.5 + i / 30.
            self.execute(env)
            if env['phase'] == 'record': break
        self.assertEqual(env['phase'], 'record')
        self.assertEqual(env['signal_camera'], 'wrist')
        self.assertTrue(self.route.recording)
        self.assertEqual(env['session'].barcode, 'P2-02')
        self.assertIsNotNone(env['patch'].reference)

    def test_fallback_final_timeout_stops(self):
        env = self.environment(); env['t'] = 2.; self.execute(env)
        env.update(t=3., stable_since=2.5); self.execute(env, 'move')
        env.update(t=3.5, stable_since=3., base_path=None); self.execute(env, 'move')
        env.update(t=4., stable_since=3.5); self.execute(env, 'move')
        env['t'] = 34.
        with self.assertRaisesRegex(ValueError, 'Fallback wrist preparation timeout'): self.execute(env)
        self.assertFalse(self.route.recording)
        self.assertEqual(env['session'].barcode, 'P2-02')

    def test_cannot_ready_while_moving(self):
        self.route.begin(0.)
        self.assertEqual(self.route.timeout_action(2., 'missing'), 'fallback')
        with self.assertRaises(ValueError): self.route.lock(2.1)
        with self.assertRaises(ValueError): self.route.timeout_action(2.1, 'moving')

    def test_reject_boxes_touching_image_boundary(self):
        for box in [(0, 10, 60, 100), (10, 0, 60, 100), (580, 10, 640, 100), (10, 400, 60, 480)]:
            self.assertFalse(self.route.accepts_box(box, (480, 640, 3)))
        self.assertTrue(self.route.accepts_box((539, 56, 603, 136), (480, 640, 3)))

    def test_partial_B07_tower_never_issues_READY(self):
        env = self.environment()
        env['sig'] = cv2.imread(str(HERE_DATA / 'B07_ready.png'))[:, 640:]
        for i in range(45):
            env['t'] = i / 30.
            self.execute(env)
        self.assertFalse(self.route.recording)
        self.assertIsNone(env['started_signal'])
        self.assertIsNone(env['prep'].patch)
        self.assertEqual(env['phase'], 'prepare')

    def test_partial_side_raw_measured_scene_can_prepare(self):
        env = self.environment()
        env['sig'] = cv2.imread(str(HERE_DATA / 'auto30cm01_fallback_started.png'))[:, 640:]
        for i in range(45):
            env['t'] = i / 30.
            self.execute(env)
            if env['phase'] == 'record': break
        self.assertEqual(env['phase'], 'record')
        self.assertEqual(env['signal_camera'], 'mast')
        self.assertTrue(env['result']['ready_evidence']['horizontal_crop_allowed'])
        self.assertEqual(env['session'].barcode, 'P2-02')

    def test_thin_side_sliver_is_rejected(self):
        tiers = dict(R=(632, 50, 8, 38), Y=(590, 80, 40, 38), G=(585, 110, 40, 38))
        self.assertFalse(self.route.accepts_box((585, 50, 640, 148), (480, 640, 3), tiers))

    def test_measured_scene_fallback_pose_is_replanned_inside_maze(self):
        cfg, _ = load_config(HERE / 'config_auto_signal_fallback_30cm.json')
        p = cfg['barcode']['center_m']
        np.testing.assert_allclose(np.array(cfg['tower_scene']['xy_m']) - p[:2], [.3, .3])
        base = cfg['signal_fallback']['base_xyz_yaw']
        self.assertLess(base[0], .8)  # Do not translate the chassis into wall_d0 at X~1.069.
        self.assertGreaterEqual(base[1], 1.1)
        from plan_poses import tower_target, evaluate
        _, model = load_config(HERE / 'config_auto_signal_fallback_30cm.json')
        target, _ = tower_target(cfg)
        info = evaluate(model, cfg['signal_fallback']['arm_waypoints_rad'][-1], base,
                        target, .35, 608.22, aim_up_deg=12.)[0]
        self.assertLess(info['look_down_deg'], 40.)
        self.assertLess(info['off_axis_deg'], 1.)
        self.assertTrue(.25 < info['distance_m'] < .5)

    def mast_environment(self):
        self.cfg, _ = load_config(HERE / 'config_auto_mast_relocate_30cm.json')
        self.route = SignalFallback(self.cfg['signal_fallback'])
        return self.environment()

    def test_mast_relocation_keeps_camera_arm_and_barcode(self):
        env = self.mast_environment()
        initial_target = env['target'].copy()
        oblique._tiers[(1, 2, 3, 4)] = {'stale': True}
        old_prep = env['prep']
        env['t'] = 2.; self.execute(env)
        self.assertEqual(env['phase'], 'move_base')
        self.assertEqual(env['signal_camera'], 'mast')
        self.assertEqual(self.route.camera, 'mast')
        np.testing.assert_array_equal(env['target'], initial_target)
        self.assertEqual(env['session'].barcode, 'P2-02')
        self.assertIsNone(env['patch'])
        self.assertIsNone(env['anchor'])
        self.assertIsNot(env['prep'], old_prep)
        self.assertIsNone(env['prep'].patch)
        self.assertIsNone(self.route.started)
        self.assertTrue(oblique._original)
        self.assertFalse(oblique._tiers)
        self.assertEqual(self.route.events[-1]['action'], 'relocate_mast')
        env.update(t=5., stable_since=None, base_path=None)
        self.execute(env, 'move')
        self.assertEqual(env['phase'], 'move_base')
        self.assertIsNone(self.route.started)
        env.update(t=5.5, stable_since=5.)
        self.execute(env, 'move')
        self.assertEqual(env['phase'], 'prepare')
        self.assertEqual(self.route.started, 5.5)
        np.testing.assert_array_equal(env['target'], initial_target)
        raw = cv2.imread(str(HERE_DATA / 'B09_arrival_view.png'))[:, 640:]
        env['sig'] = raw
        for i in range(40):
            env['t'] = 5.5 + i / 30.
            self.execute(env)
            if env['phase'] == 'record': break
        self.assertEqual(env['phase'], 'record')
        self.assertEqual(self.route.camera, 'mast')
        self.assertIsNotNone(env['patch'].reference)
        self.assertEqual(env['session'].barcode, 'P2-02')
        self.assertIsNone(self.route.timeout_action(100., 'late'))

    def test_mast_relocation_cannot_change_arm_configuration(self):
        cfg, _ = load_config(HERE / 'config_auto_mast_relocate_30cm.json')
        for key, value in [('camera', 'invalid'), ('transport_arm_q_rad', [0.]*5),
                           ('arm_waypoints_rad', [[0.]*5])]:
            bad = copy.deepcopy(cfg); bad['signal_fallback'][key] = value
            with self.assertRaises(ValueError): validate_fallback(bad, [-4]*5, [4]*5)

    def test_mast_relocation_timeout_is_final(self):
        env = self.mast_environment(); env['t'] = 2.; self.execute(env)
        env.update(t=6., stable_since=5.5, base_path=None)
        self.execute(env, 'move')
        env['t'] = 36.
        with self.assertRaisesRegex(ValueError, 'Fallback mast preparation timeout'):
            self.execute(env)
        self.assertEqual(self.route.camera, 'mast')
        self.assertFalse(self.route.recording)
        self.assertEqual(env['session'].barcode, 'P2-02')

    def test_mast_relocation_projection_is_inside_image(self):
        from common import base_transform
        from plan_poses import tower_target
        cfg, model = load_config(HERE / 'config_auto_mast_relocate_30cm.json')
        center, height = tower_target(cfg)
        camera = base_transform(cfg['signal_fallback']['base_xyz_yaw']) @ model.mast_to_optical()
        fx = 320 / np.tan(np.radians(cfg['camera']['hfov_deg']/2))
        # Include 30mm radius envelope, not just tower centerline.
        points = [center + [x, y, z] for x in [-.03, .03] for y in [-.03, .03]
                  for z in [-height/2, height/2]]
        optical = (np.linalg.inv(camera) @ np.c_[points, np.ones(len(points))].T).T[:, :3]
        pixels = optical[:, :2] / optical[:, 2, None]*fx + [320, 240]
        self.assertTrue(np.all(optical[:, 2] > 0))
        self.assertTrue(np.all(pixels > [30, 30]))
        self.assertTrue(np.all(pixels < [610, 450]))

    def test_wall_budget_is_bounded_and_defaults_to_900(self):
        self.assertEqual(wall_budget({}), 900.)
        self.assertEqual(wall_budget(dict(max_wall_seconds=1200)), 1200.)
        for value in (True, float('nan'), 50, 1801):
            with self.assertRaises(ValueError): wall_budget(dict(max_wall_seconds=value))


if __name__ == '__main__':
    unittest.main()
