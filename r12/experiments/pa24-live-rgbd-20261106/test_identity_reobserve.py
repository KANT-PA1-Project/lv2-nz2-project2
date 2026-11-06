"""Real AprilTag decoder tests; synthetic images are not live sensor evidence."""
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent / "source"
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts/sensor_runtime")]
from arm_practice.tag_detection import detect_tag_evidence, tag_evidence
import test_live_view as lifecycle


class DecoderEvidenceTests(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(1)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary)

    def image(self, ids):
        image = np.full((480,640),255,np.uint8)
        for i, identity in enumerate(ids):
            x = 60+i*220
            image[180:280,x:x+100] = cv2.aruco.generateImageMarker(self.dictionary,identity,100)
        return image

    def test_registered_tag_decodes_at_four_scales(self):
        e = detect_tag_evidence(self.image([0]),self.detector)
        self.assertFalse(e['ambiguous'])
        self.assertEqual([x['id'] for x in e['tags']],[0])
        self.assertEqual(e['tags'][0]['agreeing_scales'],[1,2,3,4])

    def test_same_id_in_two_places_is_preserved(self):
        e = detect_tag_evidence(self.image([0,0]),self.detector)
        self.assertEqual(len(e['tags']),2)
        self.assertEqual([x['id'] for x in e['tags']],[0,0])

    def test_two_distinct_ids_are_preserved(self):
        e = detect_tag_evidence(self.image([0,1]),self.detector)
        self.assertEqual(sorted(x['id'] for x in e['tags']),[0,1])

    def test_unregistered_real_decode_is_flagged(self):
        e = detect_tag_evidence(self.image([99]),self.detector)
        self.assertTrue(e['ambiguous'])
        self.assertEqual(e['groups'][0]['reason'],'unregistered_id')

    def test_conflict_cannot_hide_beside_valid_marker(self):
        c = np.array([[10,10],[30,10],[30,30],[10,30]],float)
        e = tag_evidence([(1,0,c),(2,0,c),(1,1,c+100),(2,2,c+100)])
        self.assertEqual(len(e['tags']),1)
        self.assertTrue(e['ambiguous'])
        self.assertIn('conflicting_scale_ids',[x['reason'] for x in e['groups']])

    def test_single_scale_group_cannot_be_silently_discarded(self):
        c = np.array([[10,10],[30,10],[30,30],[10,30]],float)
        e = tag_evidence([(1,0,c),(2,0,c),(1,1,c+100)])
        self.assertTrue(e['ambiguous'])
        self.assertIn('insufficient_scale_agreement',[x['reason'] for x in e['groups']])


class ReobservationTests(unittest.TestCase):
    def setUp(self):
        lifecycle.LifecycleTests.setUp(self)
    assertRejected = lifecycle.LifecycleTests.assertRejected

    def advance(self):
        self.step += 1
        self.body['capture_physics_step'] = self.step
        self.body['current_physics_step'] = self.step

    def test_two_new_frames_required_to_confirm_identity(self):
        first = self.planner.propose(self.capture)
        self.assertFalse(first['identity_confirmed'])
        self.assertTrue(first['reobserve_required'])
        self.advance()
        second = self.planner.propose(self.capture)
        self.assertTrue(second['identity_confirmed'])
        self.assertFalse(second['motion_authorized'])

    def test_rejection_resets_prior_identity_streak(self):
        self.planner.propose(self.capture); self.advance()
        self.planner.propose(self.capture); self.advance()
        def reject(*args):raise ValueError('multiple_visible_markers')
        self.planner._observe = reject
        self.assertFalse(self.planner.propose(self.capture)['identity_confirmed'])
        self.advance();self.planner._observe=lambda *args:{'id':0}
        self.assertTrue(self.planner.propose(self.capture)['reobserve_required'])
        self.advance()
        self.assertTrue(self.planner.propose(self.capture)['identity_confirmed'])

    def test_identity_switch_requires_new_confirmation(self):
        self.planner.propose(self.capture);self.advance()
        self.planner.propose(self.capture);self.advance()
        self.planner._observe=lambda *args:{'id':1}
        r=self.planner.propose(self.capture)
        self.assertFalse(r['identity_confirmed'])
        self.assertEqual(r['identity_streak'],1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
