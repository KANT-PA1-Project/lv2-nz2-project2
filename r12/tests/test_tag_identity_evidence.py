"""ID evidence and fresh-epoch confirmation; no motion authority."""
import unittest
import cv2
import numpy as np
from arm_practice.tag_detection import agreed_tags,detect_tags,detect_tag_evidence,tag_evidence
from arm_practice.native_rgbd.identity_reobservation import IdentityReobservation


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.c=np.array([[10,10],[30,10],[30,30],[10,30]],float)
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector=cv2.aruco.ArucoDetector(self.dictionary)
    def image(self,ids):
        image=np.full((480,640),255,np.uint8)
        for i,identity in enumerate(ids):image[180:280,60+i*220:160+i*220]=cv2.aruco.generateImageMarker(self.dictionary,identity,100)
        return image
    def test_legacy_list_stays_unchanged(self):
        observations=[(1,0,self.c),(2,0,self.c),(1,1,self.c+100),(2,2,self.c+100)]
        evidence=tag_evidence(observations)
        self.assertEqual(evidence['tags'],agreed_tags(observations))
        self.assertEqual(len(evidence['tags']),1)
        self.assertTrue(evidence['ambiguous'])
        self.assertIn('conflicting_scale_ids',[g['reason'] for g in evidence['groups']])
    def test_unconfirmed_second_group_is_not_silent(self):
        e=tag_evidence([(1,0,self.c),(2,0,self.c),(1,1,self.c+100)])
        self.assertTrue(e['ambiguous'])
        self.assertIn('insufficient_scale_agreement',[g['reason'] for g in e['groups']])
    def test_unknown_id_preserved_and_flagged(self):
        e=detect_tag_evidence(self.image([99]),self.detector)
        self.assertTrue(e['ambiguous']);self.assertEqual(e['tags'][0]['id'],99)
        self.assertIsNone(e['tags'][0]['mission_marker'])
    def test_real_duplicate_id_not_collapsed(self):
        e=detect_tag_evidence(self.image([0,0]),self.detector)
        self.assertEqual([t['id'] for t in e['tags']],[0,0])
        self.assertEqual(len(e['groups']),2)
    def test_real_distinct_ids_not_collapsed(self):
        e=detect_tag_evidence(self.image([0,1]),self.detector)
        self.assertEqual(sorted(t['id'] for t in e['tags']),[0,1])
    def test_decoder_legacy_wrapper_parity(self):
        image=self.image([0])
        self.assertEqual(detect_tags(image,self.detector),detect_tag_evidence(image,self.detector)['tags'])
    def test_invalid_corner_shape_rejected(self):
        with self.assertRaisesRegex(ValueError,'invalid_marker_corners'):tag_evidence([(1,0,np.zeros((3,2)))])


class ReobservationTests(unittest.TestCase):
    def setUp(self):self.gate=IdentityReobservation('current-run')
    def confirm(self,identity,step):return self.gate.confirm(identity,physics_step=step,run_id='current-run')
    def test_two_distinct_epochs_required(self):
        self.assertTrue(self.confirm(0,500)['reobserve_required'])
        r=self.confirm(0,501);self.assertTrue(r['identity_confirmed']);self.assertFalse(r['motion_authorized'])
    def test_duplicate_epoch_cannot_supply_second_vote(self):
        self.confirm(0,500)
        with self.assertRaisesRegex(ValueError,'epoch_not_increasing'):self.confirm(0,500)
        self.assertTrue(self.confirm(0,501)['reobserve_required'])
    def test_rejection_resets_votes_but_does_not_rewind_epoch(self):
        self.confirm(0,500);self.confirm(0,501);self.gate.reject()
        with self.assertRaisesRegex(ValueError,'epoch_not_increasing'):self.confirm(0,501)
        self.assertTrue(self.confirm(0,502)['reobserve_required'])
        self.assertTrue(self.confirm(0,503)['identity_confirmed'])
    def test_run_mismatch_resets_confirmation(self):
        self.confirm(0,500)
        with self.assertRaisesRegex(ValueError,'run_mismatch'):self.gate.confirm(0,physics_step=501,run_id='archived-run')
        self.assertTrue(self.confirm(0,502)['reobserve_required'])
    def test_changed_identity_requires_new_votes(self):
        self.confirm(0,500);self.confirm(0,501)
        self.assertTrue(self.confirm(1,502)['reobserve_required'])
        self.assertTrue(self.confirm(1,503)['identity_confirmed'])
    def test_destination_id_cannot_confirm_object(self):
        for step,identity in enumerate((12,13,14,99,True),500):
            with self.assertRaisesRegex(ValueError,'not_registered_object'):self.confirm(identity,step)
    def test_single_epoch_requirement_disallowed(self):
        with self.assertRaisesRegex(ValueError,'requires_two_epochs'):IdentityReobservation('run',required=1)
    def test_empty_run_disallowed(self):
        with self.assertRaisesRegex(ValueError,'missing_identity_run'):IdentityReobservation('')


if __name__=='__main__':unittest.main()
