from types import SimpleNamespace
import pytest
from arm_practice.native_rgbd.observation_owner import ObservationOwner


def owner(events, all_received=True, rejected=False):
    sample = dict(captured=True, certificate={}, pixels={'wrist': b'pixels'}, depths={})
    def send(*args): events.append('send')
    def verify(): events.append('verify')
    def publish(*args, **kwargs):
        events.append('ros')
        return {'all_received': all_received}
    def collect(frame):
        events.append('collect')
        return {'accepted': not rejected}
    return ObservationOwner(
        capture=SimpleNamespace(acquire=lambda: sample),
        prepare_publication=lambda s: ({}, {}),
        decoder=SimpleNamespace(send=send, verify_before_publication=verify, collect=collect),
        ros=SimpleNamespace(publish=publish)), sample


def test_owner_waits_for_required_ros_before_releasing_owned_buffers():
    events=[]; g,_=owner(events)
    result=g.observe(1)
    assert events == ['send', 'verify', 'ros', 'collect']
    assert g.pending is None and g.state == 'idle'
    assert not result['motion_authorized']


def test_incomplete_ros_keeps_buffers_and_blocks_next_capture():
    events=[]; g,sample=owner(events, all_received=False)
    with pytest.raises(ValueError, match='ROS_incomplete'): g.observe(1)
    assert 'collect' not in events
    assert g.pending is sample and g.state == 'failed'
    with pytest.raises(ValueError, match='not_idle'): g.observe(2)


def test_decoder_rejection_is_preserved_after_ros_completion():
    events=[]; g,_=owner(events, rejected=True)
    result=g.observe(1)
    assert result['decoded']['accepted'] is False
    assert result['motion_authorized'] is False
    assert g.state == 'idle'


def test_decoder_disconnect_does_not_release_buffers():
    events=[]; g,sample=owner(events)
    def fail(frame): raise ConnectionError('decoder disconnected')
    g.decoder.collect=fail
    with pytest.raises(ConnectionError): g.observe(1)
    assert g.pending is sample and g.state == 'failed'
