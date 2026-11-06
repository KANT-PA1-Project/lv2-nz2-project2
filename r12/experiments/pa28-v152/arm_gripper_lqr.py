"""Torque LQR for a fixed-fixture simulation diagnostic, not mission admission.
Native generalized inertia plus gravity-compensated local linearization.
Remaining articulation axes assumed constrained: this approximation is tested,
not silently generalized to a floating/mobile robot or contact dynamics.
"""
import numpy as np
from discrete_lqr import DiscreteLQR

def constrained_model(generalized_mass, controlled_dofs, dof_count, fixed_base, dt=1/960):
    M=np.asarray(generalized_mass,dtype=float);ids=np.asarray(controlled_dofs,dtype=int)
    offset=0 if fixed_base else 6
    if M.shape!=(dof_count+offset,dof_count+offset) or not np.all(np.isfinite(M)) or len(set(ids.tolist()))!=len(ids) or np.any(ids<0) or np.any(ids>=dof_count):raise ValueError('invalid_generalized_mass_or_dof_mapping')
    H=M[np.ix_(offset+ids,offset+ids)]
    if not np.allclose(H,H.T,rtol=1e-5,atol=1e-8):raise ValueError('asymmetric_mass')
    H=(H+H.T)/2
    if np.linalg.eigvalsh(H).min()<=0:raise ValueError('nonpositive_mass')
    n=len(ids);I=np.eye(n);Z=np.zeros((n,n));inv=np.linalg.solve(H,I)
    A=np.block([[I,dt*I],[Z,I]]);B=np.vstack([.5*dt*dt*inv,dt*inv])
    return H,A,B,offset

def design(generalized_mass,controlled_dofs,dof_count,fixed_base,dt=1/960):
    if len(controlled_dofs)!=7:raise ValueError('five_arm_two_finger_order_required')
    H,A,B,offset=constrained_model(generalized_mass,controlled_dofs,dof_count,fixed_base,dt)
    qscale=np.array([.025]*5+[.0005]*2);vscale=np.array([.02]*5+[.04]*2);caps=np.array([2.]*5+[10.]*2)
    Q=np.diag(1/np.r_[qscale,vscale]**2);R=np.diag(1/caps**2)
    policy=DiscreteLQR.design(A,B,Q,R,sample_period_s=dt,model_source='PhysX generalized_mass_matrices at initialized fixture; controlled7DOF inertia subblock with other coordinates constrained, exactZOH gravity-compensated double-integrator linearization')
    return policy,dict(H=H.tolist(),A=A.tolist(),B=B.tolist(),Q=Q.tolist(),R=R.tolist(),K=policy.gain.tolist(),poles_real=policy.poles.real.tolist(),poles_imag=policy.poles.imag.tolist(),spectral_radius=float(np.max(abs(policy.poles))),offset=offset,caps=caps.tolist(),assumptions=['other coordinates constrained','constant local inertia','native gravity compensated each tick','Coriolis and contact omitted from local model','native state is simulation-bench feedback, not mission sensor admission'])
