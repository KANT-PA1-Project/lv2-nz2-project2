"""Record packet encoder feedback, never reference commands or native velocity."""
import numpy as np

def arm_feedback_record(packet,joint_names):
    names=packet['names']
    if len(names)!=len(set(names)):raise ValueError('Duplicate encoder joint names')
    indices=[names.index(n) for n in joint_names]
    q=np.asarray(packet['position'],dtype=float);v=np.asarray(packet['velocity'],dtype=float)
    if q.shape!=(len(names),) or v.shape!=(len(names),):raise ValueError('Encoder shape mismatch')
    q=q[indices];v=v[indices]
    finite=bool(np.isfinite(q).all() and np.isfinite(v).all())
    stamp=packet['stamp_ns']
    stamp_valid=isinstance(stamp,int) and not isinstance(stamp,bool) and stamp>=0
    return {'arm_joint_names':list(joint_names),'arm_measured_position':q.tolist() if finite else None,
            'arm_measured_velocity':v.tolist() if finite else None,'arm_feedback_stamp_ns':stamp if stamp_valid else None,
            'arm_feedback_valid':bool(finite and stamp_valid and packet.get('velocity_valid',False)),
            'arm_feedback_source':'quantized_encoder_position_and_finite_difference_velocity',
            'arm_feedback_clock':'packet_simulation_clock','arm_feedback_live_freshness_verified':False}
