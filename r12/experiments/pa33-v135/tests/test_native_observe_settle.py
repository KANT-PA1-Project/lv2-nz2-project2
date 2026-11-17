import numpy as np,pytest
from arm_practice.native_rgbd.observe_setup import settled,TARGET

def test_settle():assert settled(TARGET,[0]*5,[.02768613,-.02768613],[0,0])
@pytest.mark.parametrize('fault',['nan','q1','speed','grip','gripspeed','shape'])
def test_not_settled(fault):
 q=TARGET.copy();v=np.zeros(5);g=np.array([.02768613,-.02768613]);gv=np.zeros(2)
 if fault=='nan':q[2]=np.nan
 if fault=='q1':q[0]=-.2
 if fault=='speed':v[3]=.021
 if fault=='grip':g[:]=0
 if fault=='gripspeed':gv[0]=.004
 if fault=='shape':q=q[:4]
 assert not settled(q,v,g,gv)
