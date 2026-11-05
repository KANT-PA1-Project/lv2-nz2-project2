import numpy as np,pytest
import arm_practice.native_rgbd.combine as c

def inputs():
 return dict(tags=[],depth=None,K=None,shape='cuboid',decoded={'detections':[],'run_id':'r','frame':0,'capture_ns':1,'geometry':{'run_id':'r','frame_index':0,'capture_monotonic_ns':1,'T_base_camera':np.eye(4).tolist(),'T_base_tcp':np.eye(4).tolist()}})
@pytest.mark.parametrize('key',['T_base_camera','T_base_tcp'])
@pytest.mark.parametrize('fault',['shape','nan','nonrigid','lastrow'])
def test_bad_fk_matrix_rejected(monkeypatch,key,fault):
 monkeypatch.setattr(c,'associate',lambda *a:{'pair':{'tag_id':0,'geometry':{'center_camera_m':[0,0,.3],'rotation_matrix':np.eye(3).tolist()}}});a=inputs();T=np.eye(4)
 if fault=='shape':T=np.eye(3)
 if fault=='nan':T[0,0]=np.nan
 if fault=='nonrigid':T[0,0]=2
 if fault=='lastrow':T[3,0]=1
 a['decoded']['geometry'][key]=T.tolist()
 with pytest.raises(ValueError):c.shadow(**a)
@pytest.mark.parametrize('center,R',[([0,0,float('nan')],np.eye(3)),([0,0,-1],np.eye(3)),([0,0,1],np.eye(3)*2),([0,0,1],np.eye(2))])
def test_bad_shadow_pose_rejected(monkeypatch,center,R):
 monkeypatch.setattr(c,'associate',lambda *a:{'pair':{'tag_id':0,'geometry':{'center_camera_m':center,'rotation_matrix':R.tolist()}}})
 with pytest.raises(ValueError):c.shadow(**inputs())
