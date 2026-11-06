"""Single-thread isolated pair admission. Local session labels are not source authentication."""
from .raw_visual_collector import RawVisualCollector
from .raw_odometry_gate import check_raw_odometry


class RawVisualAdmission:
    def __init__(self, run_id, clock_epoch, *, max_receipt_age_ns=50000000, capacity=8):
        if type(max_receipt_age_ns) is not int or max_receipt_age_ns <= 0:
            raise ValueError('invalid_age_limit')
        self.max_age = max_receipt_age_ns
        self.collector = RawVisualCollector(run_id, clock_epoch, capacity=capacity, timeout_ns=max_receipt_age_ns)
        self.last_consumed = None

    def reset(self, run_id, clock_epoch):
        self.collector.reset(run_id, clock_epoch)
        self.last_consumed = None

    def push(self, kind, record, *, now_ns, expected_stamp_ns):
        # Invalid caller state must not consume a valid pending pair.
        if type(expected_stamp_ns) is not int or expected_stamp_ns < 0:
            raise ValueError('invalid_expected_stamp')
        result = self.collector.push(kind, record, now_ns=now_ns)
        answer = dict(state=result['state'], structurally_eligible=False, raw_odometry=None,
                      source_session_verified=False, capture_freshness_verified=False,
                      execution_authorized=False)
        if result['pair'] is None:
            return answer
        raw = result['pair']['raw_odometry']
        check = check_raw_odometry(raw, expected_stamp_ns=expected_stamp_ns,
                                   last_consumed_stamp_ns=self.last_consumed, now_ns=now_ns,
                                   max_receipt_age_ns=self.max_age)
        # Both accepted and rejected completed pairs are terminal; no stamp resurrection.
        self.last_consumed = raw['stamp_ns']
        status_age = now_ns-raw['raw_status']['received_monotonic_ns']
        reason = check['reason']
        eligible = check['structurally_eligible']
        if not 0 <= status_age <= self.max_age:
            eligible, reason = False, 'status_receipt_age_invalid'
        if raw['tracking_valid'] is not True:
            eligible, reason = False, 'tracking_not_valid'
        return dict(answer, structurally_eligible=eligible, reason=reason, raw_odometry=raw)
