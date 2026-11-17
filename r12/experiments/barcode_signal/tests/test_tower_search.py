import importlib.util
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from test_tower_signal_photos import load_signal
spec = importlib.util.spec_from_file_location('tower_search', ROOT/'src/arm_practice/tower_signal_search.py')
search = importlib.util.module_from_spec(spec)
spec.loader.exec_module(search)


class TowerSearchTests(unittest.TestCase):
    def setUp(self):
        self.rgb = np.full((180,240,3), 30, np.uint8)
        self.polygons = {c: [[90,y],[105,y],[105,y+14],[90,y+14]] for c,y in zip('RYG',[50,70,90])}
        self.boxes = {c: (90,y,105,y+14,.5) for c,y in zip('RYG',[50,70,90])}
        self.colors = {'R':(130,25,25),'Y':(140,100,20),'G':(10,110,70)}

    def paint(self, dx=0, dy=0):
        for c, poly in self.polygons.items():
            center=np.asarray(poly).mean(axis=0)+[dx,dy]
            physical=np.array([center+[-11,-9],center+[11,-9],center+[11,9],center+[-11,9]])
            cv2.fillPoly(self.rgb, [np.rint(physical).astype(np.int32)], self.colors[c])

    def locate(self):
        return search.locate_body(self.rgb,self.boxes,self.polygons,confirmed_off=True)

    def test_recovers_shift_from_off_colours(self):
        self.paint(24,-18)
        r = self.locate()
        self.assertTrue(r['found'],r)
        self.assertLess(np.linalg.norm(np.array(r['shift_px'])-[24,-18]),5)

    def test_scale_is_not_shrunk_for_colour_purity(self):
        self.paint(20,10)
        r=self.locate()
        self.assertTrue(r['found'],r)
        self.assertEqual(r['scale'],1.)
        self.assertGreaterEqual(min(r['boundary_support']),search.SEARCH['min_boundary'])
        rejected=search.locate_body(self.rgb,self.boxes,self.polygons,confirmed_off=True,params={'scales':(.85,1.)})
        self.assertEqual(rejected['reason'],'unsupported_scale_search')

    def test_empty_frame_refused(self):
        self.assertEqual(self.locate()['reason'],'body_not_found')

    def test_coloured_wall_refused(self):
        self.rgb[:]=self.colors['R']
        self.assertFalse(self.locate()['found'])

    def test_wrong_band_order_refused(self):
        self.paint()
        self.rgb[50:65,90:106]=self.colors['G']
        self.rgb[90:105,90:106]=self.colors['R']
        self.assertFalse(self.locate()['found'])

    def test_missing_tier_refused(self):
        self.paint()
        self.rgb[70:85,90:106]=30
        self.assertFalse(self.locate()['found'])

    def test_duplicate_bodies_refused(self):
        self.paint(-30,0)
        self.paint(30,0)
        self.assertEqual(self.locate()['reason'],'ambiguous_bodies')

    def test_outside_search_refused(self):
        self.paint(90,0)
        self.assertFalse(self.locate()['found'])

    def test_off_confirmation_required(self):
        self.paint()
        r=search.locate_body(self.rgb,self.boxes,self.polygons)
        self.assertEqual(r['reason'],'off_confirmation_required')

    def test_white_flash_not_body(self):
        self.rgb[:]=255
        self.assertFalse(self.locate()['found'])

    def test_registered_baseline_and_presence(self):
        signal=load_signal()
        self.paint(20,10)
        reg=self.locate()
        self.assertTrue(reg['found'])
        ref=signal.make_off_reference([self.rgb]*3,reg['boxes'],roi_polygons=reg['polygons'],confirmed_off=True,context_key='fixed')
        off=search.read_registered(signal,self.rgb,reg,context_key='fixed',off_reference=ref)
        self.assertEqual(off['color'],'OFF')
        cv2.fillPoly(self.rgb,[np.rint(reg['polygons']['R']).astype(np.int32)],(255,255,255))
        on=search.read_registered(signal,self.rgb,reg,context_key='fixed',off_reference=ref)
        self.assertEqual(on['color'],'R')
        self.rgb[:]=30
        self.assertEqual(search.read_registered(signal,self.rgb,reg,context_key='fixed',off_reference=ref)['reason'],'registered_body_lost')

    def test_reference_context_change_refused(self):
        signal=load_signal()
        self.paint()
        reg=self.locate()
        ref=signal.make_off_reference([self.rgb]*3,reg['boxes'],roi_polygons=reg['polygons'],confirmed_off=True,context_key='old')
        self.assertEqual(search.read_registered(signal,self.rgb,reg,context_key='new',off_reference=ref)['color'],'UNKNOWN')


if __name__=='__main__':
    unittest.main()
