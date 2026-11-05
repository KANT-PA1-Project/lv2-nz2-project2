from types import SimpleNamespace
import pytest
from arm_practice.native_rgbd.renderer_startup_order import startup_options,validate,ARGS
from arm_practice.native_rgbd.renderer_wait_idle import WATCH,KEY
class Order:
    def __init__(self,value):self.value=value
    def __call__(self):return self.value
    def cache_info(self):return 'test cache'
def fixtures(cached=1001,observer=1001):
    data={k:False for k in WATCH};data[KEY]=True;data['/app/updateOrder/checkForHydraRenderComplete']=1000
    return SimpleNamespace(get=data.get),SimpleNamespace(post_process_graph_tick_order=Order(cached),Get=lambda:SimpleNamespace(_post_process_graph_tick_sub=SimpleNamespace(order=observer)))
def test_startup_arguments_are_only_the_two_existing_settings():
    assert startup_options(True)=={'extra_args':ARGS} and startup_options(False)=={}
def test_actual_observer_order_verified():
    assert validate(*fixtures())['observer_order']==1001
@pytest.mark.parametrize('cached,observer',[(-99,-99),(1001,-99),(-99,1001)])
def test_stale_order_rejected(cached,observer):
    with pytest.raises(ValueError):validate(*fixtures(cached,observer))
