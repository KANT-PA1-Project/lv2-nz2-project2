import numpy as np
from arm_practice.arm_integral_bias import ArmIntegralBias

def test_caps_reset_and_unwind():
    c=ArmIntegralBias([-1],[1])
    for _ in range(1000):v=c.step([0],[-.2],.05)
    assert abs(v[0]-.025)<1e-9
    assert c.step([0],[.2],.05)[0]<.025
    assert c.step([0],[0],.05,enabled=False)[0]==0

def test_joint_bound_and_invalid_feedback():
    c=ArmIntegralBias([-.35],[.35])
    for _ in range(100):assert c.step([.35],[.2],.05)[0]<=.35
    assert c.bias[0]==0
    try:c.step([0],[float('nan')],.05)
    except ValueError:pass
    else:raise AssertionError('Nonfinite accepted')
    assert c.bias[0]==0

def test_deadband_freezes_bias_but_large_errors_correct():
    c=ArmIntegralBias([-1],[1],deadband=np.pi/4096)
    c.step([0],[-.01],.05);old=c.bias.copy()
    for _ in range(100):c.step([0],[.0005],.05)
    assert np.array_equal(c.bias,old)
    c.step([0],[.002],.05);assert c.bias[0]<old[0]
