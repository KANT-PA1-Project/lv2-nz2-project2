import numpy as np
import pytest
from scipy.optimize import minimize
from arm_practice.planar_projection import project_velocity


def test_slide_along_close_wall_preserves_lateral_motion():
    np.testing.assert_allclose(project_velocity([.2,.4],[[1,0],[-1,0]],[0,.2]),[0,.4],atol=1e-10)


def test_narrow_oblique_corridor_does_not_fall_back_to_zero():
    normals=np.array([[1.,.01],[-1.,.01]])
    result=project_velocity([.3,.4],normals,[.001,.001])
    assert np.linalg.norm(result)>.09
    assert np.all(normals@result<=.001+1e-10)


def test_matches_independent_convex_optimizer_and_constraint_order():
    rng=np.random.default_rng(19)
    for _ in range(30):
        angles=rng.uniform(-np.pi,np.pi,90)
        normals=np.c_[np.cos(angles),np.sin(angles)];limits=rng.uniform(0,.4,90)
        target=rng.uniform(-.6,.6,2)
        answer=minimize(lambda v:.5*np.sum((v-target)**2),np.zeros(2),jac=lambda v:v-target,
                        constraints={'type':'ineq','fun':lambda v:limits-normals@v,'jac':lambda v:-normals},
                        method='SLSQP',options={'ftol':1e-12,'maxiter':100})
        assert answer.success
        result=project_velocity(target,normals,limits)
        np.testing.assert_allclose(result,answer.x,atol=1e-7)
        np.testing.assert_allclose(project_velocity(target,normals[::-1],limits[::-1]),result,atol=1e-8)
        assert np.max(normals@result-limits)<1e-8


def test_rejects_invalid_constraints():
    with pytest.raises(ValueError):project_velocity([.2,0],[[0,0]],[0])
    with pytest.raises(ValueError):project_velocity([.2,0],[[1,0]],[-.1])


def test_recorded_corner_retains_safe_translation_instead_of_turning_in_place():
    import json
    from pathlib import Path
    from arm_practice.lidar_guard import constrain_velocity
    sample=json.loads((Path(__file__).parent/'fixtures/lidar_oblique_corner.json').read_text())
    velocity,guard=constrain_velocity(sample['command'],sample['scans'],sample['extrinsics'],footprint=sample['footprint'])
    # The old12-iteration fallback returned [0,0,-.174705...] here.
    assert guard=='limited' and np.linalg.norm(velocity[:2])>.16
    np.testing.assert_allclose(velocity,[.12868805376639222,-.11041654495062095,-.1747051850883861],atol=1e-8)
