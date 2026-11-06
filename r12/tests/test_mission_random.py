import unittest
from collections import Counter
from arm_practice.mission_random import generate
from arm_practice.mission import plan, BARCODES
class RandomMissionTests(unittest.TestCase):
    def test_all_count_and_order_combinations(self):
        cases=0
        for r in range(1,7):
            for y in range(1,7):
                g=12-r-y
                if not 1<=g<=6:continue
                for barcode in BARCODES:
                    for seed in range(10):
                        f=generate(seed,(r,y,g),barcode);a=plan(f['sequence'],barcode,f['pucks']);byid={p['id']:p for p in f['pucks']}
                        ids=[t['id'] for loc in a['locations'] for t in loc['targets']]
                        self.assertEqual(sorted(ids),list(range(12)))
                        self.assertEqual(set(Counter(p['source'] for p in f['pucks']).values()),{2})
                        expected=sorted('RYG',key=lambda c:-a['counts'][c] if BARCODES[barcode] else a['counts'][c])
                        self.assertEqual([x['group'] for x in a['locations']],expected)
                        for loc in a['locations']:
                            ps=[byid[t['id']] for t in loc['targets']]
                            self.assertEqual(len(ps),a['counts'][loc['group']])
                            if loc['group']=='Y':self.assertEqual(len({p['shape'] for p in ps}),1)
                            else:self.assertEqual({p['color'] for p in ps},{loc['group']})
                        cases+=1
        self.assertEqual(cases,1000)
