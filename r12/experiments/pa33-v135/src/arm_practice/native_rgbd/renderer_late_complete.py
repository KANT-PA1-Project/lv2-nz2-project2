"""One additional scheduling setting against the waitIdle v64 baseline."""
from .renderer_wait_idle import KEY as WAIT_IDLE, WATCH
KEY = '/app/updateOrder/checkForHydraRenderComplete'
def configure(settings):
    before = {key: settings.get(key) for key in WATCH}
    if before[WAIT_IDLE] is not True or type(before[KEY]) is not int or before[KEY] != -100:
        raise ValueError('unexpected_renderer_baseline')
    settings.set_int(KEY, 1000)
    after = {key: settings.get(key) for key in WATCH}
    if after[KEY] != 1000 or any(before[k] != after[k] for k in WATCH if k != KEY):
        raise ValueError('renderer_single_additional_setting_contract_failed')
    return dict(changed_setting=KEY, before=before, after=after, baseline='v64 waitIdle true', additional_settings=1, settings_relative_to_v63=2, render_budget_changed=False, pixel_processing_changed=False)
