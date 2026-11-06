import pytest
from arm_practice.native_rgbd.capture_lifecycle import CaptureLifecycle

@pytest.mark.parametrize('order', [('decoder', 'ros'), ('ros', 'decoder')])
def test_both_consumers_required(order):
    g = CaptureLifecycle()
    g.begin('one', (b'pixels',))
    g.complete_validated('one', order[0])
    with pytest.raises(ValueError):
        g.release()
    with pytest.raises(ValueError):
        g.begin('two', (b'pixels',))
    assert not g.quiescent
    g.complete_validated('one', order[1])
    g.release()
    assert g.quiescent
    with pytest.raises(ValueError):
        g.begin('one', (b'pixels',))


def test_failure_retains_buffers_and_blocks_gc_boundary():
    g = CaptureLifecycle()
    buffer = bytearray(b'pixels')
    g.begin('one', (buffer,))
    g.fail()
    assert g._capture[1][0] is buffer
    assert not g.quiescent
    with pytest.raises(ValueError):
        g.release()
    with pytest.raises(ValueError):
        g.complete_validated('one', 'ros')


def test_wrong_id_duplicate_and_unknown_receipts_do_not_complete():
    g = CaptureLifecycle()
    g.begin('one', (b'pixels',))
    for ident, consumer in [('other', 'ros'), ('one', 'unknown')]:
        with pytest.raises(ValueError):
            g.complete_validated(ident, consumer)
    g.complete_validated('one', 'decoder')
    with pytest.raises(ValueError):
        g.complete_validated('one', 'decoder')
    with pytest.raises(ValueError):
        g.release()


@pytest.mark.parametrize('ros_first', [True, False])
def test_early_decoder_rejection_is_terminal_without_becoming_acceptance(ros_first):
    g = CaptureLifecycle()
    g.begin('rejected', (b'pixels',))
    if ros_first:
        g.complete_validated('rejected', 'ros')
    g.reject_validated('rejected', 'decoder', 'missing_geometric_pair')
    if not ros_first:
        with pytest.raises(ValueError):
            g.release()
        g.complete_validated('rejected', 'ros')
    result = g.release()
    assert not result['consumers_accepted']
    assert not result['motion_authorized']
    assert result['rejections'] == {'decoder': 'missing_geometric_pair'}
    # Re-observation is a new transaction; the failed observation is retained
    # in the returned summary rather than relabelled as a successful frame.
    g.begin('reobserve', (b'new_pixels',))
    g.complete_validated('reobserve', 'ros')
    g.complete_validated('reobserve', 'decoder')
    assert g.release()['consumers_accepted']
