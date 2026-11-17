"""Regression for full-cell coverage and fail-closed map input contracts."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from conservative_grid_sweep import convex_cell_cover, motion_clear


def test_arbitrarily_small_corner_overlap_is_covered():
    polygon=np.array([[.1,.1],[.2000001,.1],[.2000001,.2000001],[.1,.2000001]])
    cells=set(map(tuple,convex_cell_cover(polygon,.1)))
    assert (2,2) in cells
    grid=np.zeros((10,10),bool);grid[2,2]=True
    assert not motion_clear([0,0],0,[0,0,0],polygon,grid,.1)


@pytest.mark.parametrize('dtype',[bool,np.uint8,np.int32])
def test_binary_maps_preserve_occupied_cells(dtype):
    foot=[[-.04,-.04],[.04,-.04],[.04,.04],[-.04,.04]]
    grid=np.zeros((20,20),dtype=dtype)
    assert motion_clear([.5,.5],0,[0,0,0],foot,grid,.1)
    grid[5,5]=1
    assert not motion_clear([.5,.5],0,[0,0,0],foot,grid,.1)


@pytest.mark.parametrize('value,dtype',[(-1,np.int32),(2,np.uint8),(0.,float),(float('nan'),float)])
def test_nonbinary_and_float_maps_fail_closed(value,dtype):
    grid=np.zeros((20,20),dtype=dtype);grid[0,0]=value
    assert not motion_clear([.5,.5],0,[0,0,0],[[-.04,-.04],[.04,-.04],[0,.04]],grid,.1)


def test_large_finite_cover_rejected_before_grid_allocation():
    with pytest.raises(ValueError,match='bounded'):
        convex_cell_cover([[0,0],[1e6,0],[0,1e6]],1e-6)


def test_touching_closed_cell_is_not_exempted():
    assert (0,0) in set(map(tuple,convex_cell_cover([[.1,.1],[.2,.1],[.1,.2]],.1)))
