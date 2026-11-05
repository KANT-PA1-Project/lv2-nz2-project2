import os
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from arm_practice.native_rgbd.trusted_capture_ipc import TrustedCaptureChannel


class LifecycleTests(unittest.TestCase):
    def test_allocation_domain_explicit_and_must_match_environment(self):
        with tempfile.TemporaryDirectory() as out, patch.dict(os.environ, {'ROS_DOMAIN_ID': '33'}):
            for domain in [None, True, -1, 233, 34]:
                with self.subTest(domain=domain), self.assertRaisesRegex(ValueError, 'allocation_domain_mismatch'):
                    TrustedCaptureChannel(None, '.', out, domain_id=domain, read_local_feedback=lambda: None)

    def test_initialization_failure_reaps_owned_child_and_socket_only(self):
        for failure in ['accept_timeout', 'invalid_hello']:
            tracked = {}
            unrelated = subprocess.Popen(['/usr/bin/python3', '-c', 'import time;time.sleep(30)'])
            def broken_initialize(channel, issuer, source, out):
                channel.temp = Path(tempfile.mkdtemp(prefix='r12-lifecycle-test-'))
                channel.path = channel.temp/'test.sock'
                channel.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                channel.listener.bind(str(channel.path))
                channel.log = (Path(out)/'owned.log').open('w')
                channel.child = subprocess.Popen(['/usr/bin/python3', '-c', 'import time;time.sleep(30)'])
                tracked.update(channel=channel, child=channel.child, temp=channel.temp)
                raise RuntimeError(failure)
            try:
                with tempfile.TemporaryDirectory() as out, patch.dict(os.environ, {'ROS_DOMAIN_ID': '33'}), patch.object(TrustedCaptureChannel, '_initialize', broken_initialize):
                    with self.assertRaisesRegex(RuntimeError, failure):
                        TrustedCaptureChannel(None, '.', out, domain_id=33, read_local_feedback=lambda: None)
                    self.assertIsNotNone(tracked['child'].poll())
                    self.assertFalse(tracked['temp'].exists())
                    self.assertTrue(tracked['channel'].log.closed)
                    self.assertTrue((Path(out)/'ipc_initialization_failure.json').exists())
                    self.assertIsNone(unrelated.poll())
            finally:
                unrelated.terminate()
                unrelated.wait(timeout=3)
