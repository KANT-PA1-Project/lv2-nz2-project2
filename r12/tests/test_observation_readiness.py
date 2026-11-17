import copy
import unittest
from arm_practice.observation_readiness import check_observation

class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.p = dict(stamp_ns=1000000000,capture_run_id='a',frame_capture_monotonic_ns=100,
            perception=dict(run_id='a',clock_domain='synthetic_packet_20hz',camera='r07_camera1',
            frame_index=0,stamp_ns=1000000000,capture_monotonic_ns=100,
            inference_started_monotonic_ns=110,inference_completed_monotonic_ns=120,
            tags=[dict(id=0,corners=[[2,2],[4,2],[4,4],[2,4]],agreeing_scales=[1,2])],
            detections=[dict(class_id=0,bbox=[0,0,10,10],confidence=.7,depth_surface_m=1.,
                pose=dict(translation_valid=True,confidence=.7,center_camera_m=[0,0,1.]))]))
    def check(self, now=130):
        r=check_observation(self.p,0,now_ns=now,max_age_ns=30)
        self.assertIs(r['execution_authorized'],False)
        return r
    def test_valid_is_only_diagnostic(self):self.assertTrue(self.check()['evidence_ready'])
    def test_expiry(self):self.assertEqual(self.check(131)['reason'],'stale_response')
    def test_previous_frame(self):
        self.p['stamp_ns']+=50000000
        self.assertFalse(self.check()['evidence_ready'])
    def test_shape_conflict(self):
        d=copy.deepcopy(self.p['perception']['detections'][0]);d['class_id']=1
        self.p['perception']['detections'].append(d)
        self.assertEqual(self.check()['reason'],'class_overlap_ambiguous')
    def test_single_or_duplicate_scale(self):
        for scales in ([1],[1,1]):
            self.p['perception']['tags'][0]['agreeing_scales']=scales
            self.assertFalse(self.check()['evidence_ready'])
    def test_candidate_ids_are_not_evidence(self):
        self.p['perception']['tags']=[]
        self.p['perception']['detections'][0]['tag_ids']=[0]
        self.assertFalse(self.check()['evidence_ready'])
    def test_multiple_tags(self):
        self.p['perception']['tags']*=2
        self.assertFalse(self.check()['evidence_ready'])
    def test_outside_tag(self):
        self.p['perception']['tags'][0]['corners'][0]=[11,2]
        self.assertFalse(self.check()['evidence_ready'])
    def test_depth_sender_flag_ignored(self):
        d=self.p['perception']['detections'][0];d['depth_consistent']=True;d['depth_surface_m']=2
        self.assertEqual(self.check()['reason'],'depth_mismatch')
    def test_nan(self):
        self.p['perception']['detections'][0]['pose']['center_camera_m'][2]=float('nan')
        self.assertFalse(self.check()['evidence_ready'])
    def test_boolean_id(self):
        self.p['perception']['tags'][0]['id']=True
        self.assertFalse(self.check()['evidence_ready'])
    def test_malformed(self):
        self.p['perception']['tags'][0]['corners']=None
        self.assertFalse(self.check()['evidence_ready'])
    def test_archive_never_establishes_freshness(self):
        from arm_practice.observation_readiness import check_archived_content
        r=check_archived_content(self.p['perception'],0)
        self.assertTrue(r['content_ready'])
        self.assertFalse(r['evidence_ready'])
        self.assertFalse(r['live_freshness_verified'])
        self.assertFalse(r['execution_authorized'])
    def test_disjoint_same_id_preserves_faces(self):
        from arm_practice.observation_readiness import check_tag_identity
        tags=self.p['perception']['tags']
        tags.append(dict(id=0,corners=[[6,6],[8,6],[8,8],[6,8]],agreeing_scales=[2,3]))
        r=check_tag_identity(tags,[0,0,10,10])
        self.assertTrue(r['identity_consistent'])
        self.assertEqual(r['face_count'],2)
        self.assertEqual(r['faces'][1]['corners'],tags[1]['corners'])
        self.assertFalse(r['object_association_verified'])
        self.assertEqual(self.check()['reason'],'multi_face_object_association_unverified')
    def test_different_ids_conflict(self):
        self.p['perception']['tags'].append(dict(id=1,corners=[[6,6],[8,6],[8,8],[6,8]],agreeing_scales=[1,2]))
        self.assertEqual(self.check()['reason'],'conflicting_tag_ids')
    def test_overlapping_same_id(self):
        self.p['perception']['tags']*=2
        self.assertEqual(self.check()['reason'],'overlapping_tag_evidence')
    def test_large_box_does_not_prove_same_object(self):
        self.p['perception']['detections'][0]['bbox']=[0,0,1000,1000]
        self.p['perception']['tags'].append(dict(id=0,corners=[[600,600],[608,600],[608,608],[600,608]],agreeing_scales=[1,2]))
        self.assertFalse(self.check()['evidence_ready'])
        self.assertFalse(self.check()['identity_evidence']['object_association_verified'])
    def test_degenerate_quad(self):
        self.p['perception']['tags'][0]['corners']=[[2,2]]*4
        self.assertFalse(self.check()['evidence_ready'])
