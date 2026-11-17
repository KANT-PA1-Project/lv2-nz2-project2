import pytest
from arm_practice.native_rgbd.renderer_wait_idle import configure,KEY,WATCH
class Settings:
    def __init__(self):self.data={k:False for k in WATCH};self.writes=[]
    def get(self,k):return self.data[k]
    def set_bool(self,k,v):self.writes.append((k,v));self.data[k]=v

def test_only_documented_single_setting_changes():
    s=Settings();result=configure(s)
    assert s.writes==[(KEY,True)] and result['before'][KEY] is False
    assert not result['render_budget_changed'] and not result['pixel_processing_changed']
def test_already_enabled_rejects_identical_trial():
    s=Settings();s.data[KEY]=True
    with pytest.raises(ValueError):configure(s)
    assert not s.writes
def test_setting_refusal_rejected():
    s=Settings();s.set_bool=lambda k,v:None
    with pytest.raises(ValueError):configure(s)
