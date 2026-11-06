"""Quiescent GC boundary; copies measured state, never grants motion."""
import gc,time,math

def freeze_state(state):
 if not isinstance(state,(list,tuple)) or len(state)!=4:raise ValueError('GC_barrier_encoder_state_invalid')
 step,sim,q,qd=state
 if type(step)is not int or step<0 or type(sim)not in(int,float) or not math.isfinite(sim) or sim<0:raise ValueError('GC_barrier_encoder_state_invalid')
 vectors=[]
 for v in (q,qd):
  if not isinstance(v,(list,tuple)) or len(v)!=5 or any(type(x)not in(int,float) or not math.isfinite(x)for x in v):raise ValueError('GC_barrier_encoder_state_invalid')
  vectors.append(tuple(v))
 return (step,sim,*vectors)

def collect_stationary(*,stationary,capture_pending,rpc_pending,command,read_state,collect=gc.collect,clock=time.monotonic_ns):
 if any(type(x)is not bool for x in(stationary,capture_pending,rpc_pending)) or not stationary or capture_pending or rpc_pending or not isinstance(command,(list,tuple)) or len(command)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)or x!=0 for x in command):raise ValueError('GC_barrier_not_quiescent_stationary')
 before=freeze_state(read_state());automatic=gc.isenabled();start=clock();collected=collect();end=clock();after=freeze_state(read_state())
 if before!=after:raise ValueError('GC_barrier_encoder_or_epoch_changed')
 if automatic!=gc.isenabled():raise ValueError('GC_automatic_policy_changed')
 return dict(start_ns=start,end_ns=end,wall_ms=(end-start)/1e6,collected=collected,automatic_gc_enabled=automatic,encoder_epoch_unchanged=True,scope='GC cost moved before next physics and capture timestamps, not eliminated',motion_authorized=False)
