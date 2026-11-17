import copy
import pytest
from arm_practice.native_rgbd.mission_entrypoint import MissionNativeProducer
from test_native_rgbd_renderer_epoch_issuer import fresh

def fixture(monkeypatch):
    issuer,pixels,args=fresh();sealed=issuer.issue(**args)
    producer=MissionNativeProducer.__new__(MissionNativeProducer);producer.issuer=issuer;producer.pending=dict(certificate=sealed,pixels=pixels,depths=args['depths']);producer.rows=[{}]
    monkeypatch.setattr('arm_practice.native_rgbd.mission_entrypoint.time.monotonic_ns',lambda:args['now_ns'])
    return producer,sealed,args

def test_live_issuer_HMAC_connected(monkeypatch):
    p,c,a=fixture(monkeypatch);assert p.verify_for_publication(c)==c['body'];assert p.rows[-1]['native_verification']['rgbd_capture_association_verified']
    with pytest.raises(ValueError):p.verify_for_publication(c)

@pytest.mark.parametrize('mutation',['signature','body','rgb','depth','foreign_certificate','stale'])
def test_live_tampering_rejected(monkeypatch,mutation):
    p,c,a=fixture(monkeypatch)
    if mutation=='signature':c['signature']='0'*64
    if mutation=='body':c['body']['encoder_history']['encoder_q']=[99]
    if mutation=='rgb':p.pending['pixels']['left'][0,0,0]=1
    if mutation=='depth':p.pending['depths']['depth'][0,0]+=1
    if mutation=='foreign_certificate':c=copy.deepcopy(c);c['body']['run_id']='other'
    if mutation=='stale':monkeypatch.setattr('arm_practice.native_rgbd.mission_entrypoint.time.monotonic_ns',lambda:1_600_000_000)
    with pytest.raises(ValueError):p.verify_for_publication(c)
    assert 'native_verification' not in p.rows[-1]
