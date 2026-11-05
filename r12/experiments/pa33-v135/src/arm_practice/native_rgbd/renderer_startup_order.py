"""Apply the same two zero-delay settings before extension startup."""
ARGS=['--/app/hydraEngine/waitIdle=true','--/app/updateOrder/checkForHydraRenderComplete=1000']
def startup_options(enabled):
    return {'extra_args':list(ARGS)} if enabled else {}
def validate(settings, synthetic_data):
    from .renderer_wait_idle import WATCH, KEY
    actual={key:settings.get(key) for key in WATCH}
    cached=synthetic_data.post_process_graph_tick_order()
    observer=synthetic_data.Get()._post_process_graph_tick_sub.order
    if actual[KEY] is not True or actual['/app/updateOrder/checkForHydraRenderComplete'] != 1000 or cached != 1001 or observer != 1001:
        raise ValueError('startup_renderer_order_contract_failed')
    return dict(settings=actual,cached_order=cached,observer_order=observer,setting_phase='SimulationApp extra_args before extension startup',extra_args=list(ARGS),cache_info=str(synthetic_data.post_process_graph_tick_order.cache_info()),timing_only_change_relative_to='v65',motion_authorized=False)
