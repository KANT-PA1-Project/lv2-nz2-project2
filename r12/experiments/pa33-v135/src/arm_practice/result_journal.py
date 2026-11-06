"""Incremental diagnostic checkpoints; never a source of control observations.

The caller owns an append-only history. During execution, JSONL contains complete
records and the atomic progress file commits a readable prefix. The existing
full JSON array is still written by the caller after control execution ends.
Flush is not a power-loss durability guarantee. Disk writes can still block.
"""
import json
from pathlib import Path
import time


class ResultJournal:
    def __init__(self, result_path):
        self.result_path = Path(result_path)
        self.path = self.result_path.with_suffix('.jsonl')
        self.progress_path = self.result_path.with_name(self.result_path.stem + '_progress.json')
        self.metrics_path = self.result_path.with_name(self.result_path.stem + '_checkpoint_metrics.json')
        if any(p.exists() for p in (self.result_path, self.path, self.progress_path, self.metrics_path)):
            raise FileExistsError('Diagnostic results must be preserved in a new run directory')
        self._stream = self.path.open('xb')
        self.count = 0
        self.closed = False
        self.failed = False
        self.metrics = []

    def checkpoint(self, rows):
        if self.closed or self.failed:
            raise RuntimeError('Result journal is closed or failed')
        if not isinstance(rows, list) or len(rows) < self.count:
            raise ValueError('Result history must be an append-only list')
        if len(rows) == self.count:
            return
        start = time.monotonic_ns()
        try:
            # Serialize only the newly appended records, not the accumulated run.
            lines = [json.dumps(row, separators=(',', ':'), allow_nan=False)
                     for row in rows[self.count:]]
            payload = ('\n'.join(lines) + '\n').encode('utf-8')
            serialized = time.monotonic_ns()
            self._stream.write(payload)
            self._stream.flush()
            appended = time.monotonic_ns()
            progress = dict(schema='r12_result_journal_v1', rows=len(rows),
                            committed_bytes=self._stream.tell(), journal=self.path.name,
                            latest=json.loads(lines[-1]), final_array_pending=True)
            temporary = self.progress_path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(progress, separators=(',', ':'), allow_nan=False))
            temporary.replace(self.progress_path)
            done = time.monotonic_ns()
            self.metrics.append(dict(rows=len(rows), new_rows=len(lines), bytes=len(payload),
                                     start_monotonic_ns=start, end_monotonic_ns=done,
                                     serialize_ns=serialized-start, append_ns=appended-serialized,
                                     progress_ns=done-appended, total_ns=done-start))
            self.count = len(rows)
        except BaseException:
            self.failed = True
            raise

    def close(self):
        if self.closed:
            return
        self._stream.close()
        self.closed = True
        temporary = self.metrics_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(dict(rows=self.count, failed=self.failed,
                                             checkpoints=self.metrics), separators=(',', ':')))
        temporary.replace(self.metrics_path)


def read_committed_rows(progress_path):
    """Read only the complete prefix declared by the last atomic checkpoint."""
    progress_path = Path(progress_path)
    progress = json.loads(progress_path.read_text())
    if progress.get('schema') != 'r12_result_journal_v1':
        raise ValueError('Unknown journal schema')
    name = progress['journal']
    count, size = progress['rows'], progress['committed_bytes']
    if not isinstance(name, str) or Path(name).name != name:
        raise ValueError('Journal must be adjacent to its progress file')
    if type(count) is not int or count <= 0 or type(size) is not int or size <= 0:
        raise ValueError('Invalid committed prefix')
    with (progress_path.parent / name).open('rb') as stream:
        payload = stream.read(size)
    if len(payload) != size or not payload.endswith(b'\n'):
        raise ValueError('Committed journal prefix is incomplete')
    rows = [json.loads(line) for line in payload.splitlines()]
    if len(rows) != count or rows[-1] != progress['latest']:
        raise ValueError('Progress and journal disagree')
    return rows
