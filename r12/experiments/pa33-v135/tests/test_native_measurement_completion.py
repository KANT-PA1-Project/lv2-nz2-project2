import pytest
from arm_practice.native_rgbd.measurement_completion import summarize
def rows(n):return [dict(frame=i,captured=True)for i in range(n)]
@pytest.mark.parametrize('n',[0,4,20])
def test_requested_count(n):
    r=summarize(rows(n),n)
    assert r['measurement_complete'] and r['capture_pass']==(n>0)
    assert not r['mission_success'] and not r['performance_accepted']
def test_nineteen_of_twenty_is_only_measurement_complete():
    v=rows(20);v[1]['captured']=False;r=summarize(v,20)
    assert r['measurement_complete'] and not r['capture_pass'] and not r['performance_accepted']
def test_partial():
    assert not summarize(rows(4),20)['measurement_complete']
def test_timeout():
    r=summarize(rows(20),20,aborted=True)
    assert not r['measurement_complete'] and not r['capture_pass']
def test_capture_timeout_row():
    v=rows(4);v[2].update(captured=False,reasons=['native_ready_wall_timeout'])
    assert summarize(v,4)['measurement_complete'] and not summarize(v,4)['capture_pass']
def test_duplicate_or_missing_frame():
    v=rows(4);v[-1]['frame']=2
    assert not summarize(v,4)['measurement_complete']
@pytest.mark.parametrize('n',[True,-1,4.0])
def test_bad_request(n):
    with pytest.raises(ValueError):summarize([],n)
