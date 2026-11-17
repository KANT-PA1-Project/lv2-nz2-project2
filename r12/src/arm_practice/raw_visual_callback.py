"""Isolated callback adapter pinned to independently discovered endpoint GIDs.

A generation is local callback ownership, not a source capture/session certificate.
Create a new instance after endpoint changes; never relabel queued old callbacks.
"""
from copy import deepcopy
from .raw_visual_evidence import snapshot_raw_odometry, snapshot_raw_status
from .raw_visual_admission import RawVisualAdmission


def _gid(value):
    if not isinstance(value, dict) or not isinstance(value.get('implementation_identifier'), str) or not value['implementation_identifier']:
        raise ValueError('invalid_publisher_gid')
    data = value.get('data')
    if not isinstance(data, (bytes, list, tuple)) or len(data) != 16 or any(type(x) is not int or not 0 <= x <= 255 for x in data) or not any(data):
        raise ValueError('invalid_publisher_gid')
    return value['implementation_identifier'], tuple(data)


class RawVisualCallback:
    def __init__(self, generation, run_id, clock_epoch, *, expected_gids, max_receipt_age_ns=50000000):
        if type(generation) is not str or not generation:
            raise ValueError('invalid_generation')
        if set(expected_gids) != {'odometry', 'status'}:
            raise ValueError('both_endpoints_required')
        self.generation = generation
        self.session = (run_id, clock_epoch)
        self.expected = {k: _gid(v) for k,v in expected_gids.items()}
        self.admission = RawVisualAdmission(run_id, clock_epoch, max_receipt_age_ns=max_receipt_age_ns)
        self.faulted = False

    def receive(self, kind, message, info, *, callback_generation, received_ns, now_ns, expected_stamp_ns):
        if callback_generation != self.generation:
            raise ValueError('old_callback_generation')
        if self.faulted:
            raise ValueError('callback_faulted_requires_new_instance')
        if kind not in self.expected:
            raise ValueError('invalid_kind')
        try:
            gid = _gid(info['publisher_gid'])
            if gid != self.expected[kind]:
                raise ValueError('publisher_endpoint_changed')
        except (KeyError,TypeError,ValueError):
            self.faulted = True
            raise
        snapshot = snapshot_raw_odometry if kind == 'odometry' else snapshot_raw_status
        record = snapshot(message, run_id=self.session[0], clock_epoch=self.session[1], received_ns=received_ns)
        record['publisher_gid'] = dict(implementation_identifier=gid[0],data=list(gid[1]))
        record['local_callback_generation'] = self.generation
        try:
            result = self.admission.push(kind, record, now_ns=now_ns, expected_stamp_ns=expected_stamp_ns)
        except ValueError:
            if self.admission.collector.faulted:
                self.faulted = True
            raise
        return dict(result, received_snapshot=deepcopy(record), endpoint_matches_pinned_gid=True)
