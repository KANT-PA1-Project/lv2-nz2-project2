import pytest
from arm_practice.native_rgbd.distinct_identity import diagnostic_identity

def obj(ids):return {'pose':{'valid':True},'depth_surface_m':.3,'tag_ids':ids}
@pytest.mark.parametrize('ids',[[0],[0,0],[0,0,0],[11,11]])
def test_repeated_faces_same_identity(ids):assert diagnostic_identity([obj(ids)],fresh=True)==ids[0]
@pytest.mark.parametrize('ids',[[0,1],[0,True],[0,-1],[0,12],[0,0.0],[0,'0'],[],None])
def test_invalid_or_mixed_identity(ids):assert diagnostic_identity([obj(ids)],fresh=True)is None
@pytest.mark.parametrize('ids',[[0],[1]])
def test_multiple_boxes_remain_ambiguous(ids):assert diagnostic_identity([obj([0,0]),obj(ids)],fresh=True)is None
@pytest.mark.parametrize('value',[None,float('nan'),float('inf'),True,0.,10.])
def test_depth_invalid(value):
 o=obj([0,0]);o['depth_surface_m']=value;assert diagnostic_identity([o],fresh=True)is None
@pytest.mark.parametrize('value',[False,'true',1])
def test_invalid_pose(value):
 o=obj([0,0]);o['pose']['valid']=value;assert diagnostic_identity([o],fresh=True)is None

def test_stale_fails():assert diagnostic_identity([obj([0,0])],fresh=False)is None

@pytest.mark.parametrize('ids',[[0,1],[0,True],[],None])
def test_ambiguous_pose_box_cannot_be_hidden_by_another_valid_box(ids):
 assert diagnostic_identity([obj(ids),obj([0,0])],fresh=True) is None

@pytest.mark.parametrize('objects',[None,{},[None],[{'pose':None}]])
def test_malformed_container_fails_closed(objects):
 assert diagnostic_identity(objects,fresh=True) is None
