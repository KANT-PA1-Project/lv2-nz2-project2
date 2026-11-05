import numpy as np
from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer


def issuer():
    return RendererEpochIssuer(run_id="r", clock_epoch="c", cameras=["cam"],
        joint_names=["j"+str(i) for i in range(5)], physics_hz=960,
        model_sha256="a"*64, source_sha256="b"*64,
        fk=lambda q: {"cam": np.eye(4)}, depth_camera_map={"depth":"cam"})


def record(x, step, qd=None, fd=None):
    x.record_encoder_epoch(physics_step=step, sim_time_s=step/960,
        read_start_ns=step*10000+1, read_end_ns=step*10000+2,
        measured_encoder=[0.]*5, native_qd=qd, same_step_fd=fd)
    return x.history[step]["capture_settling"]


def test_no_velocity_or_sparse_epochs_cannot_claim_capture_settled():
    x=issuer()
    assert not record(x,0)["settled_interval"]
    for n in range(1,481): record(x,n*2,[0.]*5,[0.]*5)
    assert x.history[960]["capture_settling"]["continuous_steps"] == 0


def test_historical_settling_survives_current_motion_without_relabeling():
    x=issuer()
    for n in range(480): result=record(x,n,[.001]*5,[0.]*5)
    assert result["settled_interval"] and not result["motion_authorized"]
    assert not record(x,480,[.02]*5,[0.]*5)["settled_interval"]
    assert x.history[479]["capture_settling"]["settled_interval"]
    assert not record(x,481)["settled_interval"]
