import unittest
from arm_practice.mission_markers import marker_spec, destination_spec
from arm_practice.mission_random import generate
from arm_practice.mission import plan

class MissionMarkerTests(unittest.TestCase):
    def test_physical_sizes_are_not_white_sticker_sizes(self):
        for tag_id in range(15):
            spec = marker_spec(tag_id)
            self.assertEqual(spec['black_size_m'], .048 if tag_id >= 12 else .016)
            self.assertLess(spec['black_size_m'], spec['including_white_margin_m'])
            self.assertEqual(spec['role'], 'destination' if tag_id >= 12 else 'puck')
    def test_unregistered_ids_rejected(self):
        for tag_id in (-1, 15, True, 12.0, '12'):
            with self.assertRaises(ValueError): marker_spec(tag_id)
    def test_unverified_geometry_cannot_be_assumed_identity(self):
        for location in ('L1', 'L2', 'L3'):
            spec = destination_spec(location)
            self.assertIsNone(spec['T_tag_grid'])
            self.assertFalse(spec['mount_geometry_verified'])
            self.assertEqual(spec['mount_surface'], 'top')
    def test_plan_preserves_map_orientation_and_marker_identity(self):
        fixture = generate(42, (4, 4, 4), 'P2-01')
        result = plan(fixture['sequence'], 'P2-01', fixture['pucks'])
        for i, location in enumerate(result['locations']):
            self.assertEqual(location['destination_marker']['id'], 12 + i)
            self.assertEqual(len(location['targets']), 4)
            for target in location['targets']:
                self.assertEqual(target['center_m_evidence_kind'], 'nominal_map_planning_only')
                self.assertEqual(target['grid_offset_m'][2], 0.)
