"""Body twist from wheel encoder odometry and fused IMU yaw; no command inputs."""
import numpy as np

def base_feedback_record(packet, wheel_twist, encoder_valid, fused_imu, residual):
    result=dict(valid=False,source='wheel_encoder_xy_dual_imu_yaw',stamp_ns=packet.get('stamp_ns'),capture_monotonic_ns=packet.get('frame_capture_monotonic_ns'),run_id=packet.get('capture_run_id'),twist=None)
    try:
        wheel=np.asarray(wheel_twist,dtype=float);gyro=np.asarray(fused_imu['gyro'],dtype=float)
        if not encoder_valid or wheel.shape!=(3,) or gyro.shape!=(3,) or not np.isfinite(wheel).all() or not np.isfinite(gyro).all() or not np.isfinite(residual):return result
        return dict(result,valid=True,twist=[float(wheel[0]),float(wheel[1]),float(gyro[2])],wheel_residual=float(residual))
    except (TypeError,KeyError,ValueError):return result
