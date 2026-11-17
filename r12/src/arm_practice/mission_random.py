"""Construct solvable practice cases; generated inventory is not sensor evidence."""
import random
from .mission import plan

def generate(seed, counts=None, barcode=None):
    rng=random.Random(seed)
    options=[(r,y,12-r-y) for r in range(1,7) for y in range(1,7) if 1<=12-r-y<=6]
    counts=rng.choice(options) if counts is None else tuple(counts)
    if counts not in options:raise ValueError('Invalid counts')
    barcode=barcode or rng.choice(['P2-01','P2-02','P2-03','P2-04'])
    r,y,g=counts;shape=rng.choice(['cylinder','cuboid'])
    pucks=[{'color':c,'shape':rng.choice(['cylinder','cuboid'])} for c in 'R'*r+'G'*g]
    pucks += [{'color':rng.choice('RG'),'shape':shape} for _ in range(y)]
    rng.shuffle(pucks)
    sources=[f'S{i}' for i in range(1,7) for _ in range(2)];rng.shuffle(sources)
    for i,p in enumerate(pucks):p.update(id=i,source=sources[i])
    sequence=list('R'*r+'Y'*y+'G'*g);rng.shuffle(sequence)
    case={'description':f'Solvable synthetic random practice seed {seed}; no perception evidence','seed':seed,'sequence':''.join(sequence),'barcode':barcode,'pucks':pucks}
    plan(case['sequence'],barcode,pucks)
    return case
