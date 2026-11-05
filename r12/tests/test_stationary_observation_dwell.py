import unittest
from arm_practice.stationary_observation_dwell import StationaryObservationDwell


class DwellTests(unittest.TestCase):
    def sequence(self, dwell, count=92, excluded=None):
        cameras = sorted(dwell.cameras)
        for i in range(count):
            camera = cameras[(i // 20) % 4] if i % 20 == 0 else None
            if camera == excluded:
                camera = None
            result = dwell.update(1_000_000_000 + i * 50_000_000, True, camera)
        return result

    def test_time_and_all_cameras_required(self):
        self.assertFalse(self.sequence(StationaryObservationDwell(), 81)['complete'])
        self.assertFalse(self.sequence(StationaryObservationDwell(), excluded='arm_camera')['complete'])
        self.assertTrue(self.sequence(StationaryObservationDwell())['complete'])

    def test_motion_resets_time_and_coverage(self):
        dwell = StationaryObservationDwell()
        self.sequence(dwell)
        self.assertFalse(dwell.update(6_000_000_000, False, 'arm_camera')['complete'])
        self.assertEqual(dwell.seen, set())

    def test_gap_requires_new_continuous_dwell(self):
        dwell = StationaryObservationDwell()
        self.sequence(dwell)
        result = dwell.update(7_000_000_000, True, 'r07_camera1')
        self.assertEqual(result['elapsed_s'], 0)
        self.assertEqual(result['inspected_cameras'], ['r07_camera1'])
        self.assertFalse(result['complete'])

    def test_replay_and_invalid_stamp_rejected(self):
        for bad in (1_000_000_000, 999_999_999, -1, True, float('nan')):
            dwell = StationaryObservationDwell()
            dwell.update(1_000_000_000, True, 'r07_camera1')
            self.assertFalse(dwell.update(bad, True, 'arm_camera')['complete'])
            self.assertEqual(dwell.seen, set())

    def test_invalid_duration_rejected(self):
        for seconds in (0, -1, 31, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                StationaryObservationDwell(seconds)


if __name__ == '__main__':
    unittest.main()
