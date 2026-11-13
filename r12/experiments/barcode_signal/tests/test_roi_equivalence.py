"""Exact differential check against the frozen pre-ROI implementation."""
import importlib.util
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from test_tower_signal_photos import load_signal


def load_old():
    spec=importlib.util.spec_from_file_location('old_signal',ROOT/'tests/data/frozen_signal_before_roi.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RoiEquivalenceTests(unittest.TestCase):
    def test_exact_previous_rule_with_and_without_off_reference(self):
        old,new=load_old(),load_signal()
        rng=np.random.default_rng(36)
        boxes={c:(30,20+j*24,50,36+j*24,.5) for j,c in enumerate('RYG')}
        polygons={c:[[b[0]+1,b[1]+2],[b[2]-2,b[1]+1],[b[2]-1,b[3]-2],[b[0]+2,b[3]-1]] for c,b in boxes.items()}
        for case in range(20):
            rgb=rng.integers(0,256,(480,640,3),dtype=np.uint8)
            frames=rng.integers(0,256,(3,480,640,3),dtype=np.uint8)
            for polys in [None,polygons]:
                for use_reference in [False,True]:
                    with self.subTest(case=case,polygons=polys is not None,reference=use_reference):
                        refs=[m.make_off_reference(frames,boxes,context_key='fixed',roi_polygons=polys,confirmed_off=True) if use_reference else None for m in [old,new]]
                        args=dict(roi_polygons=polys,context_key='fixed')
                        self.assertEqual(old.color_from_boxes(rgb,boxes,off_reference=refs[0],**args),new.color_from_boxes(rgb,boxes,off_reference=refs[1],**args))


if __name__=='__main__':
    unittest.main()
