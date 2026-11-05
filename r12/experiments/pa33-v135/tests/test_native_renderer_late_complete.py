import pytest
from arm_practice.native_rgbd.renderer_late_complete import configure, KEY, WAIT_IDLE, WATCH
class Settings:
    def __init__(self):
        self.data={k:False for k in WATCH};self.data.update({WAIT_IDLE:True,KEY:-100});self.writes=[]
    def get(self,k):return self.data[k]
    def set_int(self,k,v):self.writes.append((k,v));self.data[k]=v

def test_only_additional_order_setting():
    s=Settings();r=configure(s)
    assert s.writes==[(KEY,1000)] and r['additional_settings']==1
    assert r['after'][WAIT_IDLE] is True and not r['render_budget_changed']
@pytest.mark.parametrize('key,value',[(WAIT_IDLE,False),(KEY,1000),(KEY,-100.0)])
def test_wrong_baseline_rejected(key,value):
    s=Settings();s.data[key]=value
    with pytest.raises(ValueError):configure(s)
    assert not s.writes

def test_refused_setting_rejected():
    s=Settings();s.set_int=lambda k,v:None
    with pytest.raises(ValueError):configure(s)
