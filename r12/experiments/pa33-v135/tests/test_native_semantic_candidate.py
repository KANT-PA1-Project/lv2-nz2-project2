import copy,pytest
from arm_practice.native_rgbd.semantic_candidate import IdentityCandidate

def row(i=0):
 return dict(frame=i,capture_ns=i+1,run_id='r',shadow_fresh_at_completion=True,live_shadow=dict(run_id='r',frame_index=i,capture_monotonic_ns=i+1,id=0,perception_envelope={'stamp_ns':i+1}),detections=[dict(class_id=3,class_name='green_cuboid',confidence=.9,bbox=[0,0,100,100],tag_ids=[0,0])],tags=[dict(id=0,agreeing_scales=[1,2],corners=[[10,10],[20,10],[20,20],[10,20]]),dict(id=0,agreeing_scales=[1,2],corners=[[40,10],[50,10],[50,20],[40,20]])])
def test_three_and_no_authority():
 g=IdentityCandidate(0)
 for i in range(3):r=g.update(row(i))
 assert r['candidate_identity_agreement'];assert not r['authoritative_identity']
@pytest.mark.parametrize('fault',['duplicatebox','wrongID','outside','cylinder','noconf','nan','boolconf','missingtag','one_scale','stale','epoch'])
def test_failures_clear_old_success(fault):
 g=IdentityCandidate(0)
 for i in range(3):g.update(row(i))
 r=row(3)
 if fault=='duplicatebox':r['detections']*=2
 if fault=='wrongID':r['tags'][0]['id']=1
 if fault=='outside':r['tags'][0]['corners'][0][0]=-1
 if fault=='cylinder':r['detections'][0]['class_id']=2
 if fault=='noconf':del r['detections'][0]['confidence']
 if fault=='nan':r['detections'][0]['bbox'][0]=float('nan')
 if fault=='boolconf':r['detections'][0]['confidence']=True
 if fault=='missingtag':r['tags'].pop()
 if fault=='one_scale':r['tags'][0]['agreeing_scales']=[1]
 if fault=='stale':r['shadow_fresh_at_completion']=False
 if fault=='epoch':r['live_shadow']['frame_index']=2
 assert not g.update(r)['candidate_identity_agreement'];assert g.count==0;assert g.update(row(4))['consecutive_frames']==1

def test_reset_target_and_reorder():
 g=IdentityCandidate(0)
 for i in range(3):g.update(row(i))
 assert not g.update(row(2))['semantic_checks_passed'];g.reset();assert g.count==0;g.set_target(1);assert not g.update(row(4))['semantic_checks_passed'];g.set_target(0);assert g.update(row(5))['consecutive_frames']==1

def test_new_run_restarts():
 g=IdentityCandidate(0)
 for i in range(3):g.update(row(i))
 r=row(3);r['run_id']=r['live_shadow']['run_id']='s';assert g.update(r)['consecutive_frames']==1

@pytest.mark.parametrize('bad',[None,[],{'bad':'row'}])
def test_malformed_row_rejects(bad):assert not IdentityCandidate(0).update(bad)['semantic_checks_passed']
@pytest.mark.parametrize('fault',['boolclass','strbox','wrongclasslabel','boolshadowid'])
def test_malformed_semantics(fault):
 r=row()
 if fault=='boolclass':r['detections'].append({'class_id':True})
 if fault=='strbox':r['detections'].append('box')
 if fault=='wrongclasslabel':r['detections'][0]['class_name']='red_cuboid'
 if fault=='boolshadowid':r['live_shadow']['id']=False
 assert not IdentityCandidate(0).update(r)['semantic_checks_passed']

@pytest.mark.parametrize('fault',['none_tag','none_tags','none_scales','duplicate_quad','reversed_duplicate','collinear','bowtie','same_ID_other_object','occluded','distractor'])
def test_independent_negative_resets(fault):
 g=IdentityCandidate(0)
 for i in range(3):assert g.update(row(i))['semantic_checks_passed']
 r=row(3)
 if fault=='none_tag':r['tags'][0]=None
 if fault=='none_tags':r['tags']=None
 if fault=='none_scales':r['tags'][0]['agreeing_scales']=None
 if fault=='duplicate_quad':r['tags'][1]=copy.deepcopy(r['tags'][0])
 if fault=='reversed_duplicate':r['tags'][1]['corners']=r['tags'][0]['corners'][::-1]
 if fault=='collinear':r['tags'][1]['corners']=[[40,10],[41,10],[42,10],[43,10]]
 if fault=='bowtie':r['tags'][1]['corners']=[[40,10],[50,20],[50,10],[40,20]]
 if fault=='same_ID_other_object':r['detections'].append(copy.deepcopy(r['detections'][0]))
 if fault=='occluded':r['tags']=r['tags'][:1]
 if fault=='distractor':r['detections'].append(dict(class_id=1,class_name='red_cuboid'))
 assert not g.update(r)['semantic_checks_passed'];assert g.count==0;assert g.previous is None
