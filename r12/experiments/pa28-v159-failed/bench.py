from pathlib import Path
import os,json,time,sys
root=Path(__file__).parent;out=Path(os.environ['FLEET_RUN_DIR'])
def save(n,d):
 p=out/n;t=p.with_suffix('.tmp');t.write_text(json.dumps(d,indent=2));t.replace(p)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'fast_shutdown':False,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'create_new_stage':True},experience=str(root/'minimal.kit'))
import omni.kit.app
em=omni.kit.app.get_app().get_extension_manager()
enabled=em.is_extension_enabled('isaacsim.core.simulation_manager')
rep_enabled=em.is_extension_enabled('omni.replicator.core')
assert rep_enabled,'replicator_dependency_confound'
save('extension_readback.json',dict(replicator_enabled=rep_enabled,replicator_in_sysmodules='omni.replicator.core' in sys.modules,manager_enabled=enabled,extension_list=em.get_extensions(),no_world=True,no_fixture=True))
assert enabled == False, 'manager_presence_confound'
for i in range(3):app.update()
save('result.json',dict(completed=True,diagnostic_only=True,mission_success=False))
save('before_close.json',dict(fault=None))
# All diagnostic transactions complete; public extension lifecycle only.
em.set_extension_enabled_immediate('omni.replicator.core',False)
assert not em.is_extension_enabled('omni.replicator.core'),'replicator_disable_failed'
import gc
gc.collect()
save('pre_close_extension_release.json',dict(replicator_enabled=False,gc_completed=True,transactions_complete=True))
app.close();save('after_close.json',dict(clean_return=True,fault=None,replicator_in_sysmodules='omni.replicator.core' in sys.modules))
