import unittest
import numpy as np
from arm_practice.tag_detection import agreed_tags

class AgreementTests(unittest.TestCase):
    def setUp(self):
        self.c = np.array([[0,0],[20,0],[20,20],[0,20]],float)
    def test_two_scales_agree(self):
        self.assertEqual(agreed_tags([(2,11,self.c),(3,11,self.c+.2)])[0]['id'],11)
    def test_single_or_repeated_scale_rejected(self):
        self.assertEqual(agreed_tags([(2,0,self.c),(2,0,self.c)]),[])
    def test_conflict_rejected(self):
        self.assertEqual(agreed_tags([(2,0,self.c),(3,11,self.c),(4,11,self.c)]),[])
    def test_different_locations_separate(self):
        r=agreed_tags([(2,0,self.c),(3,11,self.c+100),(4,11,self.c+100)])
        self.assertEqual([x['id'] for x in r],[11])

    def test_destination_scale_and_role_from_decoded_id(self):
        for identity in (0, 11, 12, 13, 14, 15):
            tag = agreed_tags([(1, identity, self.c), (2, identity, self.c)])[0]
            self.assertFalse(tag['pose_verified'])
            if identity == 15:
                self.assertIsNone(tag['mission_marker'])
            else:
                self.assertEqual(tag['mission_marker']['black_size_m'], .048 if identity >= 12 else .016)
                self.assertEqual(tag['mission_marker']['location'], f'L{identity-11}' if identity >= 12 else None)
