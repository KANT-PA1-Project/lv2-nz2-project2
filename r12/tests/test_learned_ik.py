import numpy as np
from arm_practice.learned_ik import CloudGate,finish

def test_cloud_rejects_hull_holes_and_nonfinite():
    points=np.array([[0,0,0],[.02,0,0],[0,.02,0],[0,0,.02]])
    gate=CloudGate(points)
    assert gate.contains([.004,.004,.004])
    assert not gate.contains([.02,.02,.02])
    assert not gate.contains([np.nan,0,0])
    assert not CloudGate(points*10).contains([.04,.04,.04])

def test_solver_reports_mm_and_preserves_unobservable_joint():
    fk=lambda q:np.asarray(q[:3])
    home=np.array([0.,0.,0.,0.,.2]);lo=np.full(5,-1.);hi=-lo
    q,error,_=finish(fk,np.array([.1,-.2,.3]),home,lo,hi,home)
    assert error<.01 and q[4]==home[4]
    q,error,_=finish(fk,np.array([2.,0.,0.]),home,lo,hi,home)
    assert error>=999 and np.all(q<=hi) and np.all(q>=lo)
