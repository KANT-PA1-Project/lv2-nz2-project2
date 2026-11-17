import subprocess
import sys
import time
import unittest
from unittest.mock import patch
import numpy as np
from arm_practice.perception_transport import JsonPipeReader, frame_identity, require_response

class TransportTests(unittest.TestCase):
    def worker(self, code):
        p = subprocess.Popen([sys.executable, '-u', '-c', code], stdout=subprocess.PIPE)
        def cleanup():
            if p.poll() is None:
                p.terminate()
            p.wait(timeout=3)
            p.stdout.close()
        self.addCleanup(cleanup)
        return JsonPipeReader(p.stdout)

    def test_logs_and_buffered_messages(self):
        r = self.worker('print(\'log\\n{bad}\\n[1]\\n{"ready":true}\\n{"stamp_ns":7}\')')
        self.assertTrue(r.receive(lambda o: o.get('ready'), 2)['ready'])
        self.assertEqual(r.receive(lambda o: 'stamp_ns' in o, 2)['stamp_ns'], 7)

    def test_fragmented_json(self):
        r = self.worker('import sys,time;sys.stdout.write(\'{"ok":\');sys.stdout.flush();time.sleep(.03);print("true}")')
        self.assertTrue(r.receive(lambda o: True, 2)['ok'])

    def test_partial_line_times_out(self):
        r = self.worker('import sys,time;sys.stdout.write("partial");sys.stdout.flush();time.sleep(5)')
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            r.receive(lambda o: True, .2)
        self.assertLess(time.monotonic()-started, 1)

    def test_eof(self):
        r = self.worker('print("not json")')
        with self.assertRaises(EOFError):
            r.receive(lambda o: True, 2)

    def test_output_limit(self):
        r = self.worker('print("x"*100)')
        r.max_bytes = 32
        with self.assertRaises(ValueError):
            r.receive(lambda o: True, 2)

    def test_invalid_timeout(self):
        r = self.worker('pass')
        for timeout in [0, -1, float('inf'), float('nan')]:
            with self.assertRaises(ValueError):
                r.receive(lambda o: True, timeout)

    def test_frame_metadata_and_request_mismatch(self):
        frame = dict(run_id=np.array('a'), clock_domain=np.array('synthetic_packet_20hz'),
                     frame_index=np.int64(0), stamp_ns=np.int64(1000000000),
                     capture_monotonic_ns=np.int64(100))
        req = frame_identity(frame, 'arm_camera')
        res = dict(req, inference_started_monotonic_ns=110, inference_completed_monotonic_ns=120)
        with patch('arm_practice.perception_transport.time.monotonic_ns', return_value=130):
            self.assertTrue(require_response(req, res, 30)['matched'])
            with self.assertRaises(ValueError):
                require_response(req, res, 29)
            res['run_id'] = 'b'
            with self.assertRaises(ValueError):
                require_response(req, res, 30)

    def test_invalid_npz_identity(self):
        base = dict(run_id='a', clock_domain='synthetic_packet_20hz', frame_index=0,
                    stamp_ns=1000000000, capture_monotonic_ns=100)
        for key,value in [('frame_index', True), ('stamp_ns', 1.0), ('run_id', ['a'])]:
            with self.assertRaises(ValueError):
                frame_identity(dict(base, **{key:value}), 'arm_camera')


    def test_cached_response_expires(self):
        from arm_practice.perception_transport import fresh_response
        req = dict(run_id='a', clock_domain='synthetic_packet_20hz', camera='arm_camera',
                   frame_index=0, stamp_ns=1000000000, capture_monotonic_ns=100)
        res = dict(req, inference_started_monotonic_ns=110, inference_completed_monotonic_ns=120,
                   barcode_boxes=[[1,2,3,4]])
        with patch('arm_practice.perception_transport.time.monotonic_ns', return_value=130):
            self.assertIn('barcode_boxes', fresh_response(req, res, 30))
        with patch('arm_practice.perception_transport.time.monotonic_ns', return_value=131):
            self.assertEqual(fresh_response(req, res, 30), {})

    def test_owned_worker_is_reaped(self):
        from arm_practice.perception_transport import stop_worker
        p = subprocess.Popen([sys.executable, '-u', '-c',
            'import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);print("ready",flush=True);time.sleep(10)'],
            stdout=subprocess.PIPE, stdin=subprocess.PIPE)
        try:
            self.assertEqual(p.stdout.readline(), b'ready\n')
            stop_worker(p, timeout_s=.1)
            self.assertIsNotNone(p.returncode)
            self.assertTrue(p.stdout.closed)
            stop_worker(p, timeout_s=.1)
        finally:
            if p.poll() is None:
                p.kill();p.wait()

if __name__ == '__main__':
    unittest.main()
