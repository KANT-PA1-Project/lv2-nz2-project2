import numpy as np
import pytest
from arm_practice.discrete_lqr import DiscreteLQR


def design(A=None, B=None, Q=None, R=None):
    return DiscreteLQR.design([[1.]] if A is None else A, [[1.]] if B is None else B,
                              [[1.]] if Q is None else Q, [[1.]] if R is None else R,
                              sample_period_s=1/120, model_source='analytic synthetic test, not R12 plant')


def test_scalar_integrator_matches_analytic_solution_and_rollout():
    c=design()
    assert c.gain[0,0] == pytest.approx((np.sqrt(5)-1)/2)
    x=np.array([1.])
    for _ in range(30):x += c.command(x,equilibrium_input=[0],lower=[-10],upper=[10])
    assert abs(x[0])<1e-10


def test_overflow_rejected_before_actuator_clipping():
    c=design(A=[[2.]])
    with pytest.raises(ValueError, match='nonfinite feedback command'):
        c.command([1.7e308], equilibrium_input=[-1.7e308], lower=[-1], upper=[1])


def test_coupled_two_state_discrete_plant_stable():
    a=np.array([[1,.01],[0,1]]);b=np.array([[.00005],[.01]])
    c=design(a,b,np.diag([10,1]),[[.1]])
    assert max(abs(c.poles))<1
    x=np.array([1.,0.]);initial=x@c.riccati@x
    x=a@x+b@c.command(x,equilibrium_input=[0],lower=[-100],upper=[100])
    assert x@c.riccati@x<initial


@pytest.mark.parametrize('kwargs',[{'B':[[0.]]},{'R':[[0.]]},{'Q':[[-1.]]},{'A':[[float('nan')]]}])
def test_invalid_or_unstabilizable_model_rejected(kwargs):
    with pytest.raises(ValueError):design(**kwargs)


def test_saturation_and_invalid_envelope():
    c=design()
    assert c.command([100],equilibrium_input=[0],lower=[-.108],upper=[.108])[0]==-.108
    with pytest.raises(ValueError):c.command([0],equilibrium_input=[0],lower=[1],upper=[-1])
    with pytest.raises(ValueError):c.command([float('nan')],equilibrium_input=[0],lower=[-1],upper=[1])


def test_joint_zoh_matches_known_double_integrator():
    from arm_practice.discrete_lqr import design_joint_lqr
    c=design_joint_lqr([[2]],[[0]],np.diag([10,1]),[[.1]],sample_period_s=.01,model_source='analytic mass test')
    expected=design([[1,.01],[0,1]],[[.000025],[.005]],np.diag([10,1]),[[.1]])
    np.testing.assert_allclose(c.gain,expected.gain,rtol=1e-10)


def test_invalid_mass_rejected():
    from arm_practice.discrete_lqr import design_joint_lqr
    with pytest.raises(ValueError):design_joint_lqr([[-1]],[[0]],np.eye(2),[[1]],sample_period_s=.01,model_source='invalid')
