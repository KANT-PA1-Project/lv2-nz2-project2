"""Mission v5 rules. Pure planning; never represents sensor evidence or execution."""
from collections import Counter
from itertools import combinations

LOCATIONS = ((3.670, .950), (2.202, 3.700), (3.670, 5.354))
START_BOXES = ((1.468,.950),(3.670,3.152),(2.936,5.354),(.734,5.354),(2.202,5.354),(3.670,3.886))
DICE = {
    1: ((0,0),), 2: ((-1,1),(1,-1)),
    3: ((-1,1),(0,0),(1,-1)),
    4: ((-1,1),(1,1),(-1,-1),(1,-1)),
    5: ((-1,1),(1,1),(0,0),(-1,-1),(1,-1)),
    6: ((-1,1),(-1,0),(-1,-1),(1,1),(1,0),(1,-1)),
}
BARCODES = {'P2-01':False, 'P2-03':False, 'P2-02':True, 'P2-04':True}


def plan(sequence, barcode, pucks):
    if len(sequence) != 12 or any(c not in 'RYG' for c in sequence):
        raise ValueError('Exactly 12 known R/Y/G observations required')
    counts = Counter(sequence)
    if any(not 1 <= counts[c] <= 6 for c in 'RYG'):
        raise ValueError('Each signal count must be 1..6')
    if barcode not in BARCODES:
        raise ValueError('Unregistered barcode')
    if len(pucks) != 12 or {p['id'] for p in pucks} != set(range(12)):
        raise ValueError('Expected each puck ID 0..11 exactly once')
    if any(p['color'] not in ('R','G') or p['shape'] not in ('cylinder','cuboid') for p in pucks):
        raise ValueError('Unknown puck color or shape')
    assignment = None
    for shape in ('cylinder','cuboid'):
        for chosen in combinations(sorted((p for p in pucks if p['shape']==shape), key=lambda p:p['id']), counts['Y']):
            ids = {p['id'] for p in chosen}
            remaining = [p for p in pucks if p['id'] not in ids]
            if Counter(p['color'] for p in remaining) == Counter(R=counts['R'],G=counts['G']):
                assignment = {'Y':sorted(ids), **{c:sorted(p['id'] for p in remaining if p['color']==c) for c in 'RG'}}
                break
        if assignment is not None:
            break
    if assignment is None:
        raise ValueError('No complete feasible allocation')
    groups = sorted('RYG', key=lambda c: -counts[c] if BARCODES[barcode] else counts[c])
    locations = []
    for index, group in enumerate(groups):
        cx,cy = LOCATIONS[index]
        locations.append({'location':f'L{index+1}', 'group':group, 'targets':[
            {'id':pid,'center_m':[cx+.06*x,cy+.06*y,.1]}
            for pid,(x,y) in zip(assignment[group],DICE[counts[group]])]})
    return {'evidence_kind':'planner_output_not_observation', 'sequence':list(sequence), 'counts':dict(counts), 'barcode':barcode, 'locations':locations}


class SignalRecorder:
    """Consumes timestamped classified frames. Unknowns/gaps fail closed.

    OFF frames separate consecutive equal colors. An external ready event must
    arm this recorder before the first flash; timestamps are simulation seconds.
    """
    def __init__(self, max_gap=.25):
        self.sequence=[]
        self.previous='OFF'
        self.last_time=None
        self.off_since=None
        self.max_gap=max_gap
        self.error=None
        self.complete=False

    def observe(self, time_s, color):
        import math
        if self.error: raise ValueError(self.error)
        try:
            if not math.isfinite(time_s): raise ValueError('Non-finite timestamp')
            if self.last_time is not None and not 0 < time_s-self.last_time <= self.max_gap:
                raise ValueError('Nonmonotonic timestamp or missing frames')
            if color not in ('R','Y','G','OFF'): raise ValueError('Unknown signal color')
            if color != 'OFF':
                if self.complete: raise ValueError('Unexpected flash after completion')
                if self.previous=='OFF':
                    self.sequence.append(color)
                    if len(self.sequence)>12: raise ValueError('More than 12 flashes')
                elif color!=self.previous:
                    raise ValueError('Color transition without OFF interval')
                self.off_since=None
            else:
                if self.off_since is None: self.off_since=time_s
                if len(self.sequence) and time_s-self.off_since >= 3.-1e-9:
                    if len(self.sequence)!=12: raise ValueError('Fewer than 12 flashes')
                    if any(not 1<=self.sequence.count(c)<=6 for c in 'RYG'):
                        raise ValueError('Invalid signal counts')
                    self.complete=True
            self.previous=color
            self.last_time=time_s
        except ValueError as error:
            self.error=str(error)
            raise
        return self.complete
