import test_native_mission_capture as fixtures

def case():
    c=fixtures.CaptureTests();c.setUp();return c

def test_stale_metadata_skips_cpu_reads():
    c=case();c.meta_step=48;calls=[]
    for name,a in c.rgb.items():
        value=a.value
        def read(name=name,value=value):calls.append(name);return value
        a.value=read
    result=c.capture.acquire()
    assert not result['captured'] and c.renders==4 and len(calls)==0
    assert all(not x['full_readback']for x in result['ready_trace'])
    assert all(x['physics_before_render']==x['physics_after_readback']for x in result['ready_trace'])

def test_metadata_requires_pixel_read_side_effect_cannot_be_certified():
    c=case();c.meta_step=48;a=next(iter(c.rgb.values()));value=a.value
    def read():c.meta_step=96;return value
    a.value=read;result=c.capture.acquire()
    assert not result['captured'] and c.renders==4
    assert result['reasons']==['native_ready_stale_epoch']*4
    assert result['certificate'] is None

def test_four_render_and_200ms_limits_unchanged():
    c=case();c.render_delta=201_000_000;result=c.capture.acquire()
    assert not result['captured'] and c.renders==1
    assert 'native_ready_wall_timeout'in result['reasons']
