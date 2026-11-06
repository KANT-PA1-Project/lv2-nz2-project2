"""Opt-in bounded publication diagnostics, independent of controller decisions."""
import json
import time
import uuid
from pathlib import Path
from .capture_bundle_identity import CaptureSessionLedger
from .image_publication_evidence import message_evidence


class PublicationDiagnostics:
    def __init__(self, directory, limit=20, *, buffered=False):
        if type(limit) is not int or not 1 <= limit <= 200:
            raise ValueError('invalid_record_limit')
        self.path = Path(directory) / ('publication_evidence_' + uuid.uuid4().hex + '.jsonl')
        self.limit = limit
        self.buffered = buffered
        self._pending = []
        self.count = 0
        self.error = None
        self._ledger = None
        self._generation = uuid.uuid4().hex
        self._row = None

    def begin(self, data, packet):
        self._row = None
        if self.count >= self.limit or self.error is not None:
            return
        self._row = dict(started_ns=time.monotonic_ns(), images=[], errors=[],
                         execution_authorized=False, pose_join_verified=False)
        try:
            if self._ledger is None:
                self._ledger = CaptureSessionLedger(packet['capture_run_id'], self._generation)
            self._row['bundle'] = self._ledger.inspect(data, packet, generation=self._generation)
        except (ValueError, TypeError, KeyError) as exc:
            self._row['errors'].append(str(exc))

    def published(self, image, info):
        if self._row is None:
            return
        if not image.header.frame_id.startswith(('r07_camera1_', 'r07_camera2_', 'r07_camera3_')):
            return
        try:
            record = message_evidence(image, info)
            record['publish_returned_ns'] = time.monotonic_ns()
            bundle = self._row.get('bundle', {})
            record['matches_bundle'] = bool(bundle.get('sequence_eligible') and
                record['stamp_ns'] == bundle['stamp_ns'] and
                record['image_sha256'] == bundle['image_sha256'].get(record['frame_id']))
            self._row['images'].append(record)
        except (ValueError, TypeError, KeyError) as exc:
            self._row['errors'].append(str(exc))

    def finish(self):
        if self._row is None:
            return
        row, self._row = self._row, None
        expected = {f'r07_camera{c}_{s}_optical' for c in (1, 2, 3) for s in ('left', 'right')}
        row['complete_correspondence'] = bool(not row['errors'] and len(row['images']) == 6 and
            {v['frame_id'] for v in row['images']} == expected and all(v['matches_bundle'] for v in row['images']))
        row['scope'] = 'Local outgoing message evidence; publish return is not delivery or cuVSLAM consumption.'
        if self.buffered:
            self._pending.append(row)
            self.count += 1
            return
        try:
            with self.path.open('x' if self.count == 0 else 'a') as f:
                f.write(json.dumps(row, allow_nan=False) + '\n')
            self.count += 1
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)

    def flush(self):
        """Persist bounded diagnostic rows after sensor processing, never in control decisions."""
        if not self._pending:
            return
        temporary = self.path.with_suffix('.jsonl.tmp')
        try:
            with temporary.open('x') as file:
                for row in self._pending:
                    file.write(json.dumps(row, allow_nan=False) + '\n')
            temporary.replace(self.path)
            self._pending.clear()
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)
            raise
