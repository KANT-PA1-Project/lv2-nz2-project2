import json
from pathlib import Path
import tempfile
import unittest

from arm_practice.result_journal import ResultJournal, read_committed_rows


class ResultJournalTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'results.json'

    def journal(self):
        journal = ResultJournal(self.path)
        self.addCleanup(journal.close)
        return journal

    def test_two_checkpoints_preserve_exact_history_without_duplicates(self):
        journal = self.journal()
        rows = [dict(stamp_ns=10, nested=dict(ids=[], label='퍽'))]
        journal.checkpoint(rows)
        first_bytes = journal.path.read_bytes()
        rows.append(dict(stamp_ns=20, nested=dict(ids=[3], label='박스')))
        journal.checkpoint(rows)
        journal.checkpoint(rows)
        self.assertTrue(journal.path.read_bytes().startswith(first_bytes))
        self.assertEqual(read_committed_rows(journal.progress_path), rows)
        self.assertEqual([m['new_rows'] for m in journal.metrics], [1, 1])
        self.assertFalse(self.path.exists())

    def test_uncommitted_tail_is_ignored(self):
        journal = self.journal()
        rows = [dict(stamp_ns=10)]
        journal.checkpoint(rows)
        with journal.path.open('ab') as stream:
            stream.write(b'{"partial":')
        self.assertEqual(read_committed_rows(journal.progress_path), rows)

    def test_missing_committed_bytes_and_changed_latest_rejected(self):
        journal = self.journal()
        journal.checkpoint([dict(stamp_ns=10)])
        progress = json.loads(journal.progress_path.read_text())
        progress['latest'] = dict(stamp_ns=11)
        journal.progress_path.write_text(json.dumps(progress))
        with self.assertRaises(ValueError):
            read_committed_rows(journal.progress_path)
        journal.path.write_bytes(b'')
        with self.assertRaises(ValueError):
            read_committed_rows(journal.progress_path)

    def test_failed_serialization_preserves_committed_prefix_and_stops_writer(self):
        journal = self.journal()
        rows = [dict(stamp_ns=10)]
        journal.checkpoint(rows)
        before = journal.path.read_bytes()
        with self.assertRaises(ValueError):
            journal.checkpoint(rows + [dict(value=float('nan'))])
        self.assertEqual(journal.path.read_bytes(), before)
        self.assertEqual(read_committed_rows(journal.progress_path), rows)
        with self.assertRaises(RuntimeError):
            journal.checkpoint(rows + [dict(value=1)])

    def test_history_shrink_and_existing_artifacts_rejected(self):
        journal = self.journal()
        journal.checkpoint([dict(index=1)])
        with self.assertRaises(ValueError):
            journal.checkpoint([])
        with self.assertRaises(FileExistsError):
            ResultJournal(self.path)

    def test_close_and_complete_final_array_compatibility(self):
        journal = self.journal()
        rows = [dict(index=i) for i in range(5)]
        journal.checkpoint(rows)
        journal.close()
        self.path.write_text(json.dumps(rows))
        self.assertEqual(json.loads(self.path.read_text()), read_committed_rows(journal.progress_path))
        self.assertEqual(json.loads(journal.metrics_path.read_text())['rows'], len(rows))
        with self.assertRaises(RuntimeError):
            journal.checkpoint(rows + [dict(index=6)])


if __name__ == '__main__':
    unittest.main()
