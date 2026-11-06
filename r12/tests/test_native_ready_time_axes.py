from arm_practice.native_rgbd.native_ready import NativeReadyWindow

def run(sensor_step, end_ns=100_000_000, physics_step=960):
    rgb=[str(i) for i in range(8)];depth=[str(i) for i in range(4)]
    w=NativeReadyWindow(physics_step=960,physics_hz=960,cameras=rgb,depths=depth,encoder_read_start_ns=0)
    w.begin_render(now_ns=1,physics_step=960)
    m={"referenceTimeNumerator":sensor_step,"referenceTimeDenominator":960}
    meta={"rgb":{k:m for k in rgb},"depth":{k:m for k in depth}}
    return w.assess(before=meta,after=meta,now_ns=end_ns,physics_step=physics_step)

def test_frozen_scene_still_has_wall_latency():
    r=run(960)
    assert r["ready"] and r["wall_wait_ms"]==100
    assert r["sensor_sim_age_s"]==0 and r["physics_advance_during_wait_s"]==0

def test_stale_sensor_has_distinct_scene_age():
    r=run(864)
    assert not r["ready"] and r["sensor_sim_age_s"]==.1
    assert r["physics_advance_during_wait_s"]==0

def test_zero_scene_motion_does_not_bypass_wall_timeout():
    r=run(960,250_000_000)
    assert r["reason"]=="native_ready_wall_timeout"
    assert r["wall_wait_ms"]==250 and r["sensor_sim_age_s"] is None

def test_physics_advance_remains_rejected():
    r=run(960,physics_step=961)
    assert r["reason"]=="physics_advanced_during_native_wait"
    assert r["physics_advance_during_wait_s"]==1/960
