"""Stationary quiescent GC barrier, never a freshness refresh or control grant."""
import gc,time,math

def collect_stationary(*,stationary,capture_pending,rpc_pending,command,read_state,collect=gc.collect,clock=time.monotonic_ns):
 if not stationary or capture_pending or rpc_pending or len(command)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)or x!=0 for x in command):raise ValueError('GC_barrier_not_quiescent_stationary')
 before=read_state();automatic=gc.isenabled();start=clock();collected=collect();end=clock();after=read_state()
 if before!=after:raise ValueError('GC_barrier_encoder_or_epoch_changed')
 if automatic!=gc.isenabled():raise ValueError('GC_automatic_policy_changed')
 return dict(start_ns=start,end_ns=end,wall_ms=(end-start)/1e6,collected=collected,automatic_gc_enabled=automatic,encoder_epoch_unchanged=True,scope='GC cost moved before next physics and capture timestamps, not eliminated',motion_authorized=False)
