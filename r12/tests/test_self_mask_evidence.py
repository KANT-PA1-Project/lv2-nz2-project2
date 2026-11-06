# Regression tests for explicit-mask archive provenance and actual CPU callsites.
from pathlib import Path
import ast, copy, hashlib, json, sys, time, unittest
import numpy as np
R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / 'src'))
from arm_practice.self_mask_evidence import preserve_self_mask_pair, array_sha256
H = 'a' * 64

def identity():
    return dict(run_id='cpu-test', frame_index=7, camera='r07_camera1', clock_domain='synthetic_packet_20hz', stamp_ns=1350000000, capture_monotonic_ns=10, readback_start_monotonic_ns=20, readback_end_monotonic_ns=30, calibration_sha256=H, source_sha256='b' * 64)

def fixture(shape=(8, 9)):
    r = np.arange(np.prod(shape), dtype=np.float32).reshape(shape) / 10
    r.flat[0] = np.nan
    r.flat[1] = np.inf
    r.flat[2] = -0.0
    seg = np.zeros(shape, np.uint32)
    seg[1:3, 2:4] = 10
    mask = seg == 10
    m = r.copy()
    m[mask] = 0
    return [r, m, mask, seg, [10]]

class Tests(unittest.TestCase):

    def test_actual_bits_and_owned_readonly(self):
        a = fixture()
        saved = [x.copy() for x in a[:4]]
        e = preserve_self_mask_pair(*a, **identity())
        meta = json.loads(e.metadata_json)
        for x, y in zip((e.raw, e.masked, e.self_mask), a):
            self.assertFalse(np.shares_memory(x, y))
            self.assertFalse(x.flags.writeable)
        for x in a[:4]:
            x[...] = 0
        self.assertEqual(e.raw.tobytes(), saved[0].tobytes())
        self.assertEqual(e.masked.tobytes(), saved[1].tobytes())
        self.assertEqual(e.self_mask.tobytes(), saved[2].tobytes())
        self.assertEqual(meta['raw_sha256'], array_sha256(saved[0]))
        self.assertFalse(meta['native_epoch_verified'])
        self.assertFalse(meta['motion_authorized'])
        self.assertEqual(len(e.archive_fields()), 4)

    def test_zeros_do_not_infer_mask(self):
        a = fixture()
        a[0][4:] = 0
        a[1][4:] = 0
        e = preserve_self_mask_pair(*a, **identity())
        self.assertEqual(int(e.self_mask.sum()), 4)

    def test_strided_input_owned(self):
        a = fixture((20, 30))
        a = [x[::2, ::3] if isinstance(x, np.ndarray) else x for x in a]
        e = preserve_self_mask_pair(*a, **identity())
        self.assertEqual(e.raw.tobytes(), a[0].tobytes())
        self.assertTrue(e.raw.flags.c_contiguous)

    def test_invalid_inputs(self):
        cases = []
        for i, val in [(0, None), (1, None), (2, None), (3, None), (2, np.zeros((8, 9), np.uint8)), (0, np.zeros((8, 9), np.float64)), (3, np.zeros((8, 9), np.float32)), (4, None), (4, [-1]), (4, [True]), (4, [10] * 4097), (4, [])]:
            a = fixture()
            a[i] = val
            cases.append((a, identity()))
        for shape in [(8, 8), (0, 0), (1, 3000001)]:
            a = fixture()
            a[0] = np.zeros(shape, np.float32)
            cases.append((a, identity()))
        for key, value in [('run_id', 'bad\nrun'), ('frame_index', True), ('frame_index', -1), ('frame_index', 2 ** 63), ('camera', 'unknown'), ('clock_domain', 'native_issuer'), ('capture_monotonic_ns', 0), ('readback_start_monotonic_ns', 0), ('readback_end_monotonic_ns', 19), ('source_sha256', '0'), ('calibration_sha256', 'f' * 63)]:
            kw = identity()
            kw[key] = value
            cases.append((fixture(), kw))
        for index in [(4, 4), (1, 2)]:
            a = fixture()
            a[1][index] = 42
            cases.append((a, identity()))
        a = fixture()
        a[2][4, 4] = True
        cases.append((a, identity()))
        for i, (a, kw) in enumerate(cases):
            with self.subTest(i=i):
                with self.assertRaises(ValueError):
                    preserve_self_mask_pair(*a, **kw)

    def test_bitwise_nonfinite_payload(self):
        a = fixture()
        a[0].view(np.uint32)[0, 0] = 2143289410
        a[1].view(np.uint32)[0, 0] = 2143289410
        preserve_self_mask_pair(*a, **identity())
        a[1].view(np.uint32)[0, 0] = 2143289411
        with self.assertRaises(ValueError):
            preserve_self_mask_pair(*a, **identity())

    def test_serialization_roundtrip_no_pickle(self):
        import io
        e = preserve_self_mask_pair(*fixture(), **identity())
        buf = io.BytesIO()
        np.savez(buf, **e.archive_fields())
        buf.seek(0)
        with np.load(buf, allow_pickle=False) as p:
            meta = json.loads(str(p['r07_camera1_self_mask_evidence_json']))
            self.assertEqual(array_sha256(p['r07_camera1_depth_unmasked']), meta['raw_sha256'])
            self.assertEqual(array_sha256(p['r07_camera1_self_mask']), meta['self_mask_sha256'])

    def test_random_reference_masks(self):
        rng = np.random.default_rng(2042)
        for _ in range(100):
            shape = tuple(rng.integers(1, 60, 2))
            r = rng.normal(size=shape).astype(np.float32)
            seg = rng.integers(0, 10, size=shape, dtype=np.uint32)
            ids = [1, 3, 8]
            b = np.isin(seg, ids)
            m = r.copy()
            m[b] = 0
            e = preserve_self_mask_pair(r, m, b, seg, ids, **identity())
            self.assertEqual(e.masked.tobytes(), m.tobytes())
            self.assertEqual(e.self_mask.tobytes(), b.tobytes())

    def test_actual_cpu_callsite_off_on(self):

        class Ann:

            def __init__(self, data):
                self.data = data

            def get_data(self, **_):
                return self.data
        src = (R / 'scripts/r12_sensor_sim.py').read_text()
        begin = src.index(' clearance_depth_arrays={}')
        end = src.index('\n if gpu_acquisition:', begin)
        import textwrap
        block = compile(textwrap.dedent(src[begin:end]), '<actual_cpu_readback>', 'exec')
        root = Path('/tmp')
        arrays = []

        def camera():
            a = fixture()
            rgb = np.arange(8 * 9 * 4, dtype=np.uint8).reshape(8, 9, 4)
            raw = a[0]
            data = {'data': a[3], 'info': {'idToLabels': {'0': '/World/Environment', '10': '/World/Spawn/R12/arm'}}}
            arrays.extend([rgb, raw, a[3]])
            return (Ann(rgb), Ann(raw), Ann(data))
        one = camera()
        right = camera()
        extras = {}
        deps = {}
        segments = {}
        for n in ['r07_camera2', 'r07_camera3', 'arm_camera']:
            for side in ['left', 'right']:
                rgb, dep, seg = camera()
                k = n + '_' + side + '_optical'
                extras[k] = rgb
                segments[k] = seg
                if side == 'left':
                    deps[n] = dep
        before = [x.tobytes() for x in arrays]

        def archive(camera, raw, masked, mask, seg, ids, started):
            k = identity()
            k.update(camera=camera, readback_start_monotonic_ns=started, readback_end_monotonic_ns=time.monotonic_ns())
            return preserve_self_mask_pair(raw, masked, mask, seg, ids, **k).archive_fields()
        common = dict(np=np, time=time, os=__import__('os'), json=json, out=root, frame=2, gpu_acquisition=False, gpu_validate=False, archive_clearance_depth=True, anns=[one[0], right[0]], depth=one[1], segments=[one[2], right[2]], extra_cameras=extras, extra_segments=segments, extra_depth=deps, read_image_annotation=lambda a: a.get_data(), archive_actual_self_pair=archive)
        ns = []
        for enabled in [False, True]:
            n = {**common, 'archive_self_mask_pair': enabled, 'readback_parts': dict.fromkeys(['rgb_depth_ns', 'segmentation_ns', 'mask_ns', 'diagnostic_write_ns'], 0)}
            exec(block, n)
            ns.append(n)
        self.assertEqual(ns[0]['rgb'].tobytes(), ns[1]['rgb'].tobytes())
        self.assertEqual(ns[0]['dep'].tobytes(), ns[1]['dep'].tobytes())
        for k, v in ns[0]['extras'].items():
            self.assertEqual(v.tobytes(), ns[1]['extras'][k].tobytes())
        self.assertEqual(len(ns[0]['clearance_depth_arrays']), 4)
        self.assertEqual(len(ns[1]['clearance_depth_arrays']), 16)
        self.assertEqual(before, [x.tobytes() for x in arrays])
        for name in ['r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera']:
            d = ns[1]['clearance_depth_arrays']
            meta = json.loads(str(d[name + '_self_mask_evidence_json']))
            self.assertEqual(meta['raw_sha256'], array_sha256(d[name + '_depth_unmasked']))
            self.assertEqual(meta['self_mask_sha256'], array_sha256(d[name + '_self_mask']))

    def test_real_adapter_capture_identity(self):
        source = (R / 'scripts/r12_sensor_sim.py').read_text()
        tree = ast.parse(source)
        adapter = next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'archive_actual_self_pair'))
        ns = dict(preserve_self_mask_pair=preserve_self_mask_pair, np=np, time=time, capture_run_id='actual-adapter-run', frame=13, frame_capture_monotonic_ns=10, self_mask_calibration_hash='a' * 64, self_mask_source_hash='b' * 64)
        exec(compile(ast.Module(body=[adapter], type_ignores=[]), '<actual_adapter>', 'exec'), ns)
        a = fixture()
        fields = ns['archive_actual_self_pair']('arm_camera', *a, 20)
        meta = json.loads(str(fields['arm_camera_self_mask_evidence_json']))
        self.assertEqual(meta['run_id'], 'actual-adapter-run')
        self.assertEqual(meta['frame_index'], 13)
        self.assertEqual(meta['stamp_ns'], 1650000000)
        self.assertEqual(meta['capture_monotonic_ns'], 10)
        self.assertEqual(meta['camera'], 'arm_camera')
        self.assertEqual(meta['source_sha256'], 'b' * 64)
        self.assertFalse(meta['encoder_association_verified'])
        self.assertIsNone(meta['issuer_certificate'])

    def test_actual_optin_rejection_guards(self):
        import os, tempfile
        from unittest.mock import patch
        source = (R / 'scripts/r12_sensor_sim.py').read_text()
        begin = source.index('archive_clearance_depth=os.getenv')
        end = source.index('from arm_practice.steering_transition', begin)
        block = compile(source[begin:end], '<actual_optin_guards>', 'exec')
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / 'configs/sensors').mkdir(parents=True)
            (root / 'configs/sensors/camera_rig.json').write_text('{}')
            common = dict(os=os, np=np, Path=Path, root=root, __file__=str(R / 'scripts/r12_sensor_sim.py'), archive_geometry_fields=lambda: {'geometry': 'cpu fixture'})
            for env in ({'R12_ARCHIVE_SELF_MASK_PAIR': '1', 'R12_ARCHIVE_CLEARANCE_DEPTH': '0'}, {'R12_ARCHIVE_SELF_MASK_PAIR': '1', 'R12_ARCHIVE_CLEARANCE_DEPTH': '1', 'R12_GPU_ACQUISITION': '1'}):
                with patch.dict(os.environ, env, clear=True):
                    with self.assertRaises(ValueError):
                        exec(block, common.copy())
            with patch.dict(os.environ, {}, clear=True):
                n = common.copy()
                exec(block, n)
                self.assertFalse(n['archive_self_mask_pair'])
                self.assertNotIn('self_mask_source_hash', n)
