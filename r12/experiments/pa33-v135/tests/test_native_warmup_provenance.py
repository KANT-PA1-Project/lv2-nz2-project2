import json,pytest
from arm_practice.native_rgbd.warmup_provenance import plan,digest,replay

def rows(n):return [dict(frame=i,captured=True,attempts=4,ready_age_ms=120,capture_monotonic_ns=1000+i,physics_step=100+i)for i in range(n)]
def run(r,p):return replay(json.dumps(r).encode(),p,digest(p))
def test_failed_warmup_is_preserved_not_retried():
    r=rows(5);r[1].update(captured=False,ready_age_ms=None);p=plan(2,3);out=run(r,p)
    assert out['warmup_failures']==[1] and out['measurement_capture_pass']
    assert not out['all_epoch_capture_pass'] and not out['performance_accepted']
    assert [x['original_record']for x in out['records']]==r
    assert not out['live_freshness_authorized'] and not out['motion_authorized']
def test_phase_plan_cannot_change_after_pinning():
    p=plan(2,3);sha=digest(p);p['warmup_epochs']=3
    with pytest.raises(ValueError):replay(json.dumps(rows(5)).encode(),p,sha)
def test_partial_measurement():
    assert not run(rows(4),plan(2,3))['measurement_complete']
def test_no_warmup_retains_cold_failure():
    r=rows(3);r[1].update(captured=False,ready_age_ms=None)
    assert not run(r,plan(0,3))['measurement_capture_pass']
@pytest.mark.parametrize('kind',['duplicate','reorder','overbudget','render5','late_success'])
def test_invalid_records_rejected(kind):
    r=rows(3)
    if kind=='duplicate':r[2]['frame']=1
    if kind=='reorder':r.reverse()
    if kind=='overbudget':r=rows(4)
    if kind=='render5':r[0]['attempts']=5
    if kind=='late_success':r[0]['ready_age_ms']=201
    with pytest.raises(ValueError):run(r,plan(1,2))
def test_original_timestamp_and_digest_preserved():
    r=rows(3);o=run(r,plan(1,2))
    for source,target in zip(r,o['records']):
        assert target['original_row_sha256']==digest(source)
        assert target['original_record']['capture_monotonic_ns']==source['capture_monotonic_ns']
