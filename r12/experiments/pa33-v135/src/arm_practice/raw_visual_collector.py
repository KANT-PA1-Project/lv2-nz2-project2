"""Isolated bounded exact-stamp collector. Single-thread/caller serialized API."""
from copy import deepcopy
from .raw_visual_evidence import pair_raw_visual_evidence


class RawVisualCollector:
    def __init__(self, run_id, clock_epoch, *, capacity=8, timeout_ns=250000000):
        if type(capacity) is not int or capacity < 1 or type(timeout_ns) is not int or timeout_ns < 1:
            raise ValueError('invalid_limits')
        self.capacity, self.timeout_ns = capacity, timeout_ns
        self.reset(run_id, clock_epoch)

    def reset(self, run_id, clock_epoch):
        if any(type(x) is not str or not x for x in (run_id, clock_epoch)):
            raise ValueError('invalid_session')
        if hasattr(self, 'session') and (run_id, clock_epoch) == self.session:
            raise ValueError('reset_requires_new_session')
        self.session = (run_id, clock_epoch)
        self.pending, self.retired, self.last_now, self.faulted = {}, -1, -1, False

    def _retire(self, stamp):
        self.retired = max(self.retired, stamp)
        self.pending = {s: v for s, v in self.pending.items() if s > self.retired}

    def expire(self, now_ns):
        if type(now_ns) is not int or now_ns < self.last_now or now_ns < 0:
            self.faulted = True
            raise ValueError('clock_regression')
        self.last_now = now_ns
        expired = [s for s, v in self.pending.items() if now_ns-v['first_ns'] > self.timeout_ns]
        if expired:
            self._retire(max(expired))
        return len(expired)

    def push(self, kind, record, *, now_ns):
        self.expire(now_ns)
        if self.faulted:
            raise ValueError('collector_faulted_requires_new_session')
        if kind not in ('odometry', 'status'):
            raise ValueError('invalid_kind')
        if (record['run_id'], record['clock_epoch']) != self.session or record['clock_domain'] != 'synthetic_packet_20hz':
            raise ValueError('session_mismatch')
        stamp, received = record['stamp_ns'], record['received_monotonic_ns']
        if type(stamp) is not int or stamp < 0 or type(received) is not int or not 0 <= received <= now_ns:
            raise ValueError('invalid_time')
        topic = '/visual_slam/tracking/odometry' if kind == 'odometry' else '/visual_slam/status'
        if record['source_topic'] != topic:
            raise ValueError('source_mismatch')
        if stamp <= self.retired:
            return dict(state='retired', pair=None)
        if now_ns-received > self.timeout_ns:
            self._retire(stamp)
            return dict(state='expired', pair=None)
        entry = self.pending.setdefault(stamp, dict(first_ns=received))
        if kind in entry:
            # Re-delivery receipt time alone is not a content conflict or TTL refresh.
            old = {k:v for k,v in entry[kind].items() if k != 'received_monotonic_ns'}
            new = {k:v for k,v in record.items() if k != 'received_monotonic_ns'}
            if old != new:
                self.faulted = True
                self.pending.clear()
                raise ValueError('conflicting_duplicate')
            return dict(state='duplicate', pair=None)
        entry[kind] = deepcopy(record)
        entry['first_ns'] = min(entry['first_ns'], received)
        if len(self.pending) > self.capacity:
            self._retire(min(self.pending))
        if stamp not in self.pending:
            return dict(state='capacity_retired', pair=None)
        if 'odometry' in entry and 'status' in entry:
            pair = pair_raw_visual_evidence(entry['odometry'], entry['status'])
            self._retire(stamp)
            return dict(state='paired', pair=pair)
        return dict(state='pending', pair=None)
