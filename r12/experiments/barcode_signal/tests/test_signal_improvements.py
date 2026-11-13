import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_tower_signal_photos import load_signal


class SignalImprovementTests(unittest.TestCase):
    def setUp(self):
        self.m=load_signal()
        self.boxes={'R':(0,0,10,10,0.),'Y':(0,10,10,20,0.),'G':(0,20,10,30,0.)}
        self.rgb=np.full((30,10,3),40,np.uint8)

    def test_unique_white_resolves_red_bloom(self):
        self.rgb[:10]=[255,0,0]
        self.rgb[10:20]=[255,180,0]
        self.rgb[:4]=255
        new=self.m.color_from_boxes(self.rgb,self.boxes)
        old=self.m.color_from_boxes(self.rgb,self.boxes,dict(self.m.PARAMS,white_unique_enabled=False))
        self.assertEqual(new['color'],'R')
        self.assertEqual(new['decision'],'unique_white')
        self.assertEqual(old['color'],'UNKNOWN')

    def test_multiple_white_tiers_not_forced_to_a_color(self):
        self.rgb[:20]=255
        self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes)['color'],'UNKNOWN')

    def test_off_reference_requires_confirmation(self):
        with self.assertRaises(ValueError):
            self.m.make_off_reference([self.rgb]*3,self.boxes,context_key='fixed')

    def test_off_reference_requires_three_frames(self):
        with self.assertRaises(ValueError):
            self.m.make_off_reference([self.rgb]*2,self.boxes,context_key='fixed',confirmed_off=True)

    def make_reference(self):
        return self.m.make_off_reference([self.rgb]*3,self.boxes,context_key='fixed',confirmed_off=True)

    def test_bright_colored_unlit_lens_stays_off_with_reference(self):
        self.rgb[10:20]=[220,160,10]
        ref=self.make_reference()
        self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes)['color'],'Y')
        result=self.m.color_from_boxes(self.rgb,self.boxes,off_reference=ref,context_key='fixed')
        self.assertEqual(result['color'],'OFF')
        self.assertEqual(result['decision'],'off_delta')

    def test_new_white_detected_even_if_brightness_delta_is_small(self):
        self.rgb[10:20]=[220,160,10]
        ref=self.make_reference()
        self.rgb[10:20]=255
        result=self.m.color_from_boxes(self.rgb,self.boxes,off_reference=ref,context_key='fixed')
        self.assertEqual(result['color'],'Y')
        self.assertEqual(result['decision'],'unique_white')

    def test_unchanged_white_glint_does_not_become_on(self):
        self.rgb[:10]=255
        ref=self.make_reference()
        self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes,off_reference=ref,context_key='fixed')['color'],'OFF')

    def test_colored_nonwhite_rise_still_detected(self):
        ref=self.make_reference()
        self.rgb[20:]=[0,255,0]
        self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes,off_reference=ref,context_key='fixed')['color'],'G')

    def test_reference_context_change_is_rejected(self):
        ref=self.make_reference()
        for key in [None,'other_exposure']:
            self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes,off_reference=ref,context_key=key)['reason'],'off_reference_mismatch')

    def test_reference_roi_change_is_rejected(self):
        ref=self.make_reference()
        boxes=dict(self.boxes,R=(1,0,10,10,0.))
        self.assertEqual(self.m.color_from_boxes(self.rgb,boxes,off_reference=ref,context_key='fixed')['reason'],'off_reference_mismatch')

    def test_reference_is_copied_and_readonly(self):
        ref=self.make_reference()
        self.rgb[:]=255
        self.assertTrue(np.all(ref.rgb==40))
        with self.assertRaises(ValueError):
            ref.rgb[0,0]=0

    def test_camera_identity_includes_pose_and_exposure(self):
        T=np.eye(4); K=np.eye(3)
        a=self.m.camera_context(T,K,self.rgb.shape,'exp10_gain2')
        self.assertNotEqual(a,self.m.camera_context(T,K,self.rgb.shape,'exp20_gain2'))
        T[0,3]=.01
        self.assertNotEqual(a,self.m.camera_context(T,K,self.rgb.shape,'exp10_gain2'))

    def test_projected_roi_centers_hit_the_correct_physical_tier(self):
        from test_tower_signal import K, CENTER, look_at
        for eye in [[.12,.35,.30],[.734,.95,.8767],[.40,.95,.70]]:
            T=look_at(eye,CENTER)
            angle=np.radians(10)
            T[:3,:3]=T[:3,:3]@np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
            boxes,polygons,reason=self.m.tier_rois(T,K,(480,640,3))
            self.assertIsNone(reason)
            for c,points in polygons.items():
                uv=np.mean(points,axis=0)
                ray=T[:3,:3]@np.linalg.solve(K,[*uv,1.])
                offset=T[:2,3]-self.m.TOWER['xy_m']
                a=np.dot(ray[:2],ray[:2]);b=2*np.dot(offset,ray[:2])
                q=np.dot(offset,offset)-(self.m.TOWER['lens_diameter_m']/2)**2
                distance=min(np.roots([a,b,q]))
                z=T[2,3]+distance*ray[2]-self.m.TOWER['base_z_m']
                lo,hi=self.m.TOWER['tiers'][c]
                self.assertGreater(float(z),lo)
                self.assertLess(float(z),hi)

    def test_polygon_mask_excludes_white_background(self):
        self.rgb[:]=255
        self.rgb[:,2:8]=40
        polygons={c:[[2,b[1]],[7,b[1]],[7,b[3]-1],[2,b[3]-1]] for c,b in self.boxes.items()}
        self.assertEqual(self.m.color_from_boxes(self.rgb,self.boxes,roi_polygons=polygons)['color'],'OFF')


if __name__=='__main__':
    unittest.main()
