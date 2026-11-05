"""Inactive, single-thread capture lifetime guard for a future runtime owner.

The caller authenticates receipts and enforces their deadlines before completion.
This guard neither validates IPC nor grants motion. Failed transactions retain
buffers until the owning runtime tears down its consumers.
"""


class CaptureLifecycle:
    def __init__(self):
        self._capture = None
        self._pending = set()
        self._failed = False
        self._used = set()
        self._rejections = {}

    @property
    def quiescent(self):
        return self._capture is None and not self._failed

    def begin(self, capture_id, owned_buffers):
        if not self.quiescent:
            raise ValueError('capture_transaction_not_quiescent')
        if not isinstance(capture_id, str) or not capture_id or capture_id in self._used:
            raise ValueError('capture_id_invalid_or_replayed')
        if not isinstance(owned_buffers, tuple) or not owned_buffers:
            raise ValueError('capture_buffers_required')
        self._used.add(capture_id)
        self._capture = (capture_id, owned_buffers)
        self._pending = {'decoder', 'ros'}
        self._rejections = {}

    def complete_validated(self, capture_id, consumer):
        if self._failed or self._capture is None or capture_id != self._capture[0]:
            raise ValueError('capture_completion_mismatch')
        if consumer not in self._pending:
            raise ValueError('capture_consumer_invalid_or_duplicate')
        self._pending.remove(consumer)

    def reject_validated(self, capture_id, consumer, reason):
        """A validated negative result is terminal, but never acceptance.

        A decoder can reject geometry before requesting a current-feedback
        refresh. The runtime must still finish ROS and preserve this rejection.
        """
        if not isinstance(reason, str) or not reason:
            raise ValueError('capture_rejection_reason_required')
        self.complete_validated(capture_id, consumer)
        self._rejections[consumer] = reason

    def fail(self):
        if self._capture is None:
            raise ValueError('no_capture_transaction')
        self._failed = True

    def release(self):
        if self._failed or self._capture is None or self._pending:
            raise ValueError('capture_consumers_not_successfully_complete')
        result = {'capture_id': self._capture[0],
                  'consumers_accepted': not self._rejections,
                  'rejections': dict(self._rejections),
                  'motion_authorized': False}
        self._capture = None
        return result
