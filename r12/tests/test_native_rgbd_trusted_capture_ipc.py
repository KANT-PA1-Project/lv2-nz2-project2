import copy
import json
import os
import socket
import struct
import unittest

from arm_practice.native_rgbd.trusted_capture_ipc import (MAX_HEADER, MAX_BYTES, send_packet, receive_packet,
                                require_peer, peer_credentials, request_matches,
                                valid_gid, make_request)
from test_native_rgbd_renderer_epoch_issuer import fresh


class IPCContractTests(unittest.TestCase):
    def test_socket_peer_pid_and_uid(self):
        a, b = socket.socketpair()
        try:
            peer = peer_credentials(a)
            require_peer(peer, os.getpid(), os.getuid())
            with self.assertRaisesRegex(ValueError, 'untrusted_peer'):
                require_peer(peer, os.getpid() + 1, os.getuid())
            with self.assertRaisesRegex(ValueError, 'untrusted_peer'):
                require_peer(peer, os.getpid(), os.getuid() + 1)
        finally:
            a.close()
            b.close()

    def test_wire_round_trip_binary(self):
        a, b = socket.socketpair()
        try:
            send_packet(a, {'kind': 'sample'}, [b'\x00\xff\x01'])
            self.assertEqual(receive_packet(b), ({'kind': 'sample'}, [b'\x00\xff\x01']))
        finally:
            a.close()
            b.close()

    def test_oversize_header_rejected(self):
        a, b = socket.socketpair()
        try:
            a.sendall(struct.pack('!I', MAX_HEADER + 1))
            with self.assertRaisesRegex(ValueError, 'invalid_header_length'):
                receive_packet(b)
        finally:
            a.close()
            b.close()

    def test_malformed_blob_sizes_rejected(self):
        for lengths in [[True], [-1], [MAX_BYTES + 1], [1] * 13]:
            a, b = socket.socketpair()
            try:
                raw = json.dumps({'blob_lengths': lengths}).encode()
                a.sendall(struct.pack('!I', len(raw)) + raw)
                with self.assertRaisesRegex(ValueError, 'invalid_blob_lengths'):
                    receive_packet(b)
            finally:
                a.close()
                b.close()

    def test_closed_peer_rejected(self):
        a, b = socket.socketpair()
        a.close()
        try:
            with self.assertRaisesRegex(ValueError, 'peer_closed'):
                receive_packet(b)
        finally:
            b.close()

    def test_GID_shape_zero_and_case(self):
        self.assertTrue(valid_gid('a1' * 24))
        for gid in [None, '', '0' * 48, 'A' * 48, 'x' * 48, 'a' * 47]:
            self.assertFalse(valid_gid(gid))

    def request(self):
        issuer, pixels, args = fresh()
        sealed = issuer.issue(**args)
        return make_request(sealed, pixels, args['depths'], nonce='n', gid='a1' * 24)

    def test_exact_request_matches(self):
        request = self.request()
        request_matches(copy.deepcopy(request), request)

    def test_any_identity_change_rejected(self):
        expected = self.request()
        for field in expected:
            changed = copy.deepcopy(expected)
            changed[field] = None
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'binding_mismatch'):
                request_matches(changed, expected)

    def test_extra_key_rejected(self):
        expected = self.request()
        changed = dict(expected, image_capture_association_verified=True)
        with self.assertRaisesRegex(ValueError, 'binding_mismatch'):
            request_matches(changed, expected)

    def test_actual_depth_byte_change_changes_request(self):
        issuer, pixels, args = fresh()
        sealed = issuer.issue(**args)
        before = make_request(sealed, pixels, args['depths'], nonce='n', gid='a1' * 24)
        args['depths']['depth'][4, 5] += .01
        after = make_request(sealed, pixels, args['depths'], nonce='n', gid='a1' * 24)
        with self.assertRaisesRegex(ValueError, 'depth_sha256'):
            request_matches(after, before)


if __name__ == '__main__':
    unittest.main()
