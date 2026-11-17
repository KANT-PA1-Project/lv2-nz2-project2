"""Strict single-puck semantic association and diagnostic consensus candidate.
Never yields authoritative receipt, pose confidence, uncertainty or admission.
"""
import math
import cv2
import numpy as np

def reals(xs,n):return isinstance(xs,list)and len(xs)==n and all(type(x)in(int,float)and math.isfinite(x)for x in xs)
def associate(row,target):
 if not isinstance(row,dict):raise ValueError('invalid_row_container')
 if type(target)is not int or not 0<=target<12:raise ValueError('invalid_target')
 if row.get('shadow_fresh_at_completion')is not True:raise ValueError('stale_shadow')
 s=row['live_shadow']
 if not isinstance(s,dict)or type(s.get('id'))is not int:raise ValueError('invalid_shadow_identity')
 stamp=s['perception_envelope']['stamp_ns']
 if any(type(row[k])is not int or row[k]<0 for k in ['frame','capture_ns'])or type(stamp)is not int or stamp<0:raise ValueError('invalid_epoch')
 if s['run_id']!=row['run_id']or s['frame_index']!=row['frame']or s['capture_monotonic_ns']!=row['capture_ns']or s['id']!=target:raise ValueError('shadow_identity_mismatch')
 detections=row['detections']
 if not isinstance(detections,list)or any(not isinstance(d,dict)or type(d.get('class_id'))is not int or d['class_id']<0 for d in detections):raise ValueError('invalid_detection_container')
 boxes=[d for d in detections if d['class_id']<4]
 if len(boxes)!=1:raise ValueError('not_exactly_one_object_box')
 box=boxes[0];confidence=box.get('confidence')
 if box['class_id']not in(1,3)or box.get('class_name')!={1:'red_cuboid',3:'green_cuboid'}.get(box['class_id']):raise ValueError('noncuboid_semantics')
 if type(confidence)not in(int,float)or not math.isfinite(confidence)or not .25<=confidence<=1:raise ValueError('invalid_detector_confidence')
 xy=box['bbox']
 if not reals(xy,4)or not xy[0]<xy[2]or not xy[1]<xy[3]:raise ValueError('invalid_bbox')
 tags=row['tags']
 if not isinstance(tags,list) or len(tags)!=2 or any(not isinstance(t,dict) for t in tags):raise ValueError('invalid_two_face_container')
 for tag in tags:
  if type(tag.get('id'))is not int or tag['id']!=target:raise ValueError('tag_ID_conflict')
  scales=tag.get('agreeing_scales',[])
  if not isinstance(scales,list):raise ValueError('invalid_scale_container')
  if any(type(x)is not int or x not in(1,2,3)for x in scales)or len(set(scales))<2:raise ValueError('insufficient_scale_agreement')
  cs=tag.get('corners')
  if not isinstance(cs,list)or len(cs)!=4 or any(not reals(c,2)for c in cs):raise ValueError('invalid_tag_corners')
  c=np.asarray(cs,np.float32)
  if not cv2.isContourConvex(c) or abs(cv2.contourArea(c))<25:raise ValueError('degenerate_tag_quad')
  if not all(xy[0]<=x<=xy[2]and xy[1]<=y<=xy[3]for x,y in cs):raise ValueError('tag_outside_bbox')
 if sorted(map(tuple,tags[0]['corners']))==sorted(map(tuple,tags[1]['corners'])):raise ValueError('duplicate_tag_quad')
 ids=box.get('tag_ids')
 if not isinstance(ids,list)or not ids or any(type(x)is not int or x!=target for x in ids):raise ValueError('box_ID_conflict')
 return dict(run_id=row['run_id'],frame=row['frame'],capture_ns=row['capture_ns'],stamp_ns=stamp,id=target,class_id=box['class_id'],bbox=xy)

class IdentityCandidate:
 def __init__(self,target):self.target=target;self.reset()
 def reset(self):self.previous=None;self.count=0
 def set_target(self,target):self.target=target;self.reset()
 def update(self,row):
  try:
   current=associate(row,self.target);p=self.previous
   if p is not None:
    if current['run_id']==p['run_id']and (current['frame']<=p['frame']or current['capture_ns']<=p['capture_ns']or current['stamp_ns']<=p['stamp_ns']):raise ValueError('duplicate_or_reordered_epoch')
    contiguous=current['run_id']==p['run_id']and current['frame']==p['frame']+1 and current['class_id']==p['class_id']
   else:contiguous=False
   self.count=self.count+1 if contiguous else 1;self.previous=current
   return dict(semantic_checks_passed=True,consecutive_frames=self.count,candidate_identity_agreement=self.count>=3,authoritative_identity=False,adapter_ready=False,motion_authorized=False)
  except (ValueError,KeyError,TypeError)as e:
   self.reset();return dict(rejected=str(e),semantic_checks_passed=False,consecutive_frames=0,candidate_identity_agreement=False,authoritative_identity=False,adapter_ready=False,motion_authorized=False)
