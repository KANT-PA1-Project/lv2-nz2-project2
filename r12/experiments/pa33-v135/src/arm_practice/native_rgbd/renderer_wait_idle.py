"""Single renderer scheduling setting from installed Isaac6.1 zero-delay experience."""
KEY='/app/hydraEngine/waitIdle'
WATCH=[KEY,'/app/updateOrder/checkForHydraRenderComplete','/app/asyncRendering','/app/asyncRenderingLowLatency','/omni/replicator/asyncRendering','/omni/replicator/RTSubframes','/rtx/hydra/supportMultiTickRate']
def configure(settings):
    before={key:settings.get(key)for key in WATCH}
    if before[KEY] is True:raise ValueError('waitIdle_already_enabled_no_new_experiment')
    settings.set_bool(KEY,True)
    after={key:settings.get(key)for key in WATCH}
    if after[KEY] is not True or any(before[k]!=after[k]for k in WATCH if k!=KEY):raise ValueError('renderer_single_setting_contract_failed')
    return dict(changed_setting=KEY,before=before,after=after,physics_rate_changed=False,render_budget_changed=False,pixel_processing_changed=False)
