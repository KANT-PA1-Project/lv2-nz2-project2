"""Diagnostic pairing from tag identity and depth geometry, not YOLO containment.
No semantic box acceptance or control admission is returned. Input multi-scale
observations retain the existing two-distinct-scales rule. GT never enters.
"""
import itertools
from .dual_plane import plane,from_planes

def associate_planes(records):
 if len(records)>12:raise ValueError('bounded_tag_count_exceeded')
 pairs=[]
 for i,j in itertools.combinations(range(len(records)),2):
  a,b=records[i],records[j]
  if a['id']!=b['id']:continue
  try:g=from_planes(a['plane'],b['plane'])
  except ValueError:continue
  pairs.append(dict(indices=[i,j],tag_id=a['id'],geometry=g))
 if len(pairs)!=1:raise ValueError('missing_or_ambiguous_geometric_pair')
 return dict(pair=pairs[0],semantic_box_association_validated=False,budget_certified=False,control_admitted=False,motion_authorized=False)

def associate(tags,depth,K):
 if not isinstance(tags,list) or any(not isinstance(t,dict) for t in tags):raise ValueError('invalid_tag_container')
 if len(tags)>12:raise ValueError('bounded_tag_count_exceeded')
 records=[]
 for t in tags:
  if type(t.get('id'))is not int or not 0<=t['id']<12:raise ValueError('invalid_identity')
  scales=t.get('agreeing_scales',[])
  if not isinstance(scales,list):raise ValueError('invalid_scale_container')
  if any(type(s)is not int or s not in(1,2,3,4)for s in scales)or len(set(scales))<2:raise ValueError('insufficient_scale_agreement')
  records.append(dict(id=t['id'],plane=plane(t['corners'],depth,K)))
 return associate_planes(records)
