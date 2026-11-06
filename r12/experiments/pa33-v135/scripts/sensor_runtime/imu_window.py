"""Anti-alias native IMU samples before reducing 120 Hz physics to 20 Hz control."""
import numpy as np

def reduce_window(samples, expected_count=6):
    if len(samples)!=expected_count:raise ValueError('Incomplete IMU window')
    times=np.array([s['time'] for s in samples],float)
    gyro=np.array([s['gyro'] for s in samples],float);accel=np.array([s['accel'] for s in samples],float)
    if not all(s['valid'] for s in samples) or not np.isfinite(times).all() or not np.isfinite(gyro).all() or not np.isfinite(accel).all():raise ValueError('Invalid IMU window')
    if gyro.shape!=(expected_count,3) or accel.shape!=gyro.shape:raise ValueError('Invalid vector shape')
    # Native sensor timestamps are float32; their quantization grows with uptime.
    tolerance=max(1e-5,2*float(np.spacing(np.float32(np.max(np.abs(times))))))
    intervals=np.diff(times)
    if tolerance>=1/240 or (intervals<=0).any() or not np.allclose(intervals,1/120,rtol=0,atol=tolerance):raise ValueError('Repeated or missing native IMU samples')
    return {'valid':True,'gyro':gyro.mean(axis=0).tolist(),'accel':accel.mean(axis=0).tolist(),'native_sample_count':len(samples),'native_time_range':times[[0,-1]].tolist(),'endpoint_gyro':gyro[-1].tolist()}
