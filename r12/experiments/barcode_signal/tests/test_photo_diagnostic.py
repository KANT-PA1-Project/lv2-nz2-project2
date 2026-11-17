import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from test_tower_signal_photos import load_signal, tier_boxes_from_bbox


class PhotoDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.module=load_signal()
        self.boxes=tier_boxes_from_bbox([10,10,40,100])

    def test_inner_boxes_order(self):
        self.assertEqual(self.boxes['R'][:4],(16,16,34,34))
        self.assertEqual(self.boxes['Y'][:4],(16,46,34,64))
        self.assertEqual(self.boxes['G'][:4],(16,76,34,94))

    def test_explicit_roi_single_colors(self):
        for color in 'RYG':
            with self.subTest(color=color):
                rgb=np.full((120,60,3),40,np.uint8)
                x0,y0,x1,y1,_=self.boxes[color]
                rgb[y0:y1,x0:x1]=255
                self.assertEqual(self.module.color_from_boxes(rgb,self.boxes)['color'],color)

    def test_invalid_boxes_fail_closed(self):
        rgb=np.zeros((120,60,3),np.uint8)
        self.assertEqual(self.module.color_from_boxes(rgb,{})['reason'],'invalid_boxes')
        boxes=dict(self.boxes,R=(0,0,0,2,0))
        self.assertEqual(self.module.color_from_boxes(rgb,boxes)['color'],'UNKNOWN')
        boxes=dict(self.boxes,G=(0,0,1000,2,0))
        self.assertEqual(self.module.color_from_boxes(rgb,boxes)['color'],'UNKNOWN')

    def test_same_rule_as_projected_path(self):
        from test_tower_signal import K, CENTER, look_at, render
        T=look_at([0.12,0.35,0.30],CENTER)
        for lit in [(),('R',),('Y',),('G',),('R','Y')]:
            rgb,_=render(T,lit)
            projected=self.module.tower_signal_color(rgb,T,K)
            explicit=self.module.color_from_boxes(rgb,projected['boxes'],roi_polygons=projected['roi_polygons'])
            self.assertEqual(projected,explicit)


if __name__=='__main__':
    unittest.main()
