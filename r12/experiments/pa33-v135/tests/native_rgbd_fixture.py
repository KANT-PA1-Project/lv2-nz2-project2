import copy
import hashlib
import os
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch
import numpy as np

from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer, canonical
from arm_practice.native_rgbd.trusted_capture_ipc import make_request, send_packet
from arm_practice.native_rgbd.separated_sensor_inputs import SeparatedSensorInputs, JOINTS


class BoundaryFixture:
    def setUp(self):
        pixel_env=patch.dict(os.environ,{"R12_ISAAC_PIXEL_CENTER_V1":"1"});pixel_env.start();self.addCleanup(pixel_env.stop)
        self.now = time.monotonic_ns()
        self.sock, self.server = socket.socketpair()
        self.addCleanup(self.sock.close)
        self.addCleanup(self.server.close)
        self.cameras = ['arm_camera_left_optical', 'arm_camera_right_optical']
        def fk(q):
            transform = np.eye(4)
            transform[0, 3] = q['arm_joint1']
            return {k: transform.copy() for k in self.cameras}
        issuer = RendererEpochIssuer(run_id='r', clock_epoch='epoch', cameras=self.cameras,
            joint_names=JOINTS, physics_hz=960, model_sha256='0'*64, source_sha256='1'*64,
            fk=fk, depth_camera_map={'arm_camera_depth': self.cameras[0]})
        for step, q, age in [(96, .1, 60_000_000), (144, .2, 10_000_000)]:
            start = self.now-age
            issuer.record_encoder_epoch(physics_step=step, sim_time_s=step/960,
                read_start_ns=start, read_end_ns=start+100, measured_encoder=[q]*5)
        self.pixels = {k: np.zeros((480, 640, 3), np.uint8) for k in self.cameras}
        self.depths = {'arm_camera_depth': np.ones((480, 640), np.float32)}
        rgb_meta = {k: {'referenceTimeNumerator': 100000000, 'referenceTimeDenominator': 1000000000} for k in self.cameras}
        depth_meta = {'arm_camera_depth': copy.deepcopy(rgb_meta[self.cameras[0]])}
        self.sealed = issuer.issue(native_camera_metadata=rgb_meta, pixels=self.pixels,
            native_depth_metadata=depth_meta, depths=self.depths,
            native_metadata_after={'rgb': rgb_meta, 'depth': depth_meta},
            current_physics_step=144, readback_return_ns=self.now, now_ns=self.now)
        verdict = issuer.verify(self.sealed, pixels=self.pixels, depths=self.depths, now_ns=self.now)
        self.request = make_request(self.sealed, self.pixels, self.depths, nonce='n', gid='a1'*16)
        self.feedback = {'read_start_ns': self.now-1_000_000, 'read_end_ns': self.now-999900,
            'sequence': 1, 'physics_step': 144, 'q': [.2]*5,
            'identity': {'run_id': 'r', 'clock_epoch': 'epoch', 'source_sha256': '1'*64,
                'model_sha256': '0'*64, 'producer_pid': os.getpid(), 'stream_id': 'stream',
                'kind': 'current_local_actuator_encoder', 'joint_names': JOINTS}}
        self.boundary = SeparatedSensorInputs(self.sock, producer_pid=os.getpid(), uid=os.getuid(),
            run_id='r', clock_epoch='epoch', source_sha256='1'*64, model_sha256='0'*64,
            publisher_gid='a1'*16, feedback_stream_id='stream')
        response = {'kind': 'verdict', 'nonce': 'n', 'publisher_gid': 'a1'*16,
            'request_sha256': hashlib.sha256(canonical(self.request)).hexdigest(),
            'verdict': verdict, 'local_feedback': self.feedback}
        send_packet(self.server, response)
        _, self.capture, self.current = self.boundary.receive_response(self.request,
            sealed=self.sealed, pixels=self.pixels, depths=self.depths)
