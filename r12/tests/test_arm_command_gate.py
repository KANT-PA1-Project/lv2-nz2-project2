import math
from arm_practice.arm_command_gate import ArmCommandGate

def gate():return ArmCommandGate(['j'],[-1],[1],[.4])
def test_ownership_and_switch():
    g=gate();g.select('planner',[0]);assert not g.submit('policy',['j'],[.2],1)
    assert g.submit('planner',['j'],[.2],1)
    try:g.select('servo',[.1]);assert False
    except ValueError:pass
    g.select('servo',[0]);assert g.target is None

def test_staleness_limits_and_nan():
    g=gate();g.select('planner',[0]);assert not g.submit('planner',['j'],[2],1)
    assert not g.submit('planner',['j'],[math.nan],1)
    assert g.submit('planner',['j'],[.5],1)
    assert g.step(1,[0],1)==[0]
    assert abs(g.step(1.05,[0],1.05)[0]-.02)<1e-9
    assert g.step(1.4,[.02],1.4)==[.02]
    assert g.step(1.5,[.02],1)==None
    assert g.step(1.5,[math.nan],1.5)==None


def test_hold_does_not_follow_gravity_drift():
    g=gate();assert g.step(1,[.1],1)==[.1]
    assert g.step(1.05,[.08],1.05)==[.1]


def test_reference_slew_does_not_follow_gravity_sag():
    g=gate();g.select('transport',[0]);g.submit('transport',['j'],[.3],1)
    g.step(1,[0],1)
    previous=0
    for k in range(1,21):
        now=1+k*.05;g.submit('transport',['j'],[.3],now)
        target=g.step(now,[-.03],now)[0]
        assert 0<=target-previous<=.02+1e-9
        previous=target
    assert abs(previous-.3)<1e-9
