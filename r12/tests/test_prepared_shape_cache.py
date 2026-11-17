import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
import conservative_grid_sweep as core


def test_source_mutation_is_new_preparation_not_cached_alias():
    core.clear_preparation_cache()
    body=np.array([[0.,0.],[.1,0.],[0.,.1]])
    saved,radius=core._prepare_body(body)
    body[1,0]=.2
    changed,new_radius=core._prepare_body(body)
    assert radius==.1 and new_radius==.2
    assert saved[1,0]==.1 and changed[1,0]==.2
    assert not saved.flags.writeable
    assert core.preparation_cache_info()['body']['misses']==2


def test_preparation_is_bounded_and_pose_map_not_cached():
    core.clear_preparation_cache()
    footprint=np.array([[0.,0.],[.1,0.],[0.,.1]])
    for i in range(25):core._prepare_body(footprint*(1+i*.01))
    assert core.preparation_cache_info()['body']['currsize']==16
    grid=np.zeros((40,40),dtype=bool)
    assert core.motion_clear([1.,1.],0,[0,0,0],footprint,grid,.1)
    grid[:]=True
    assert not core.motion_clear([1.,1.],0,[0,0,0],footprint,grid,.1)


def test_world_rounding_collapse_is_rejected():
    # Body is nondegenerate, but loses all width after translation.
    body=np.array([[0.,0.],[1e-12,0.],[0.,1e-12]])
    world=body+1e6
    with pytest.raises(ValueError,match='degenerate_world'):
        core._cell_cover_prepared(world,np.eye(2),1.,0.,max_cells=100)
