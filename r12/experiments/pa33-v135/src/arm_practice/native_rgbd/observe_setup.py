"""Initial simulator configuration and actual measured settling; no motion path claim."""
import numpy as np
TARGET=np.array([-.30,0.,0.,0.,0.])
def settled(q,qd,gripq,gripv):
 q=np.asarray(q);qd=np.asarray(qd);g=np.asarray(gripq);v=np.asarray(gripv)
 if q.shape!=(5,)or qd.shape!=(5,)or g.shape!=(2,)or v.shape!=(2,):return False
 if not all(np.isfinite(x).all()for x in [q,qd,g,v]):return False
 return bool(abs(q[0]-TARGET[0])<=.01 and np.max(abs(q[1:]))<=.15 and np.max(abs(qd))<=.02 and np.max(abs(g-[.02768613,-.02768613]))<=.001 and np.max(abs(v))<=.003)

def preserve_arm_reference(measured, setup, indices):
 measured=np.asarray(measured,dtype=float);setup=np.asarray(setup,dtype=float)
 if measured.shape!=setup.shape or measured.ndim!=1 or not np.isfinite(measured).all()or not np.isfinite(setup).all():raise ValueError('invalid hold reference')
 if len(indices)!=5 or len(set(indices))!=5 or any(type(i)is not int or i<0 or i>=len(measured)for i in indices):raise ValueError('invalid arm mapping')
 result=measured.copy();result[indices]=setup[indices];return result
