"""Bounded actual physics feedback refresh after completed native acquisition."""
import math
import time

class PostCaptureFeedback:
    def __init__(self, *, capture_step, capture_lower_bound_ns, capture_completed_ns, physics_hz):
        if physics_hz != 960 or not 0 <= capture_lower_bound_ns <= capture_completed_ns:
            raise ValueError('invalid_postcapture_identity')
        self.capture_step = capture_step
        self.lower = capture_lower_bound_ns
        self.completed = capture_completed_ns
        self.hz = physics_hz

    def run(self, *, stepper, read_step, sample_encoder, read_imus):
        start = read_step()
        if start < self.capture_step or time.monotonic_ns() < self.completed:
            raise ValueError('capture_not_complete')
        samples = []
        previous_imus = None
        imu_deltas = []
        imu_valid = True
        for sub in range(16):
            stepper(sub)
            if read_step() != start + sub + 1:
                raise ValueError('postcapture_physics_step_not_distinct')
            if (sub + 1) % 8:
                continue
            row = sample_encoder()
            if row['step'] != start + sub + 1 or row['read_start_ns'] < self.completed:
                raise ValueError('postcapture_encoder_epoch_mismatch')
            if not row['velocity_valid'] or not all(math.isfinite(v) for v in row['velocity']):
                raise ValueError('postcapture_velocity_not_actual_120hz')
            imus = read_imus()
            if len(imus) != 2 or any(not x['valid'] or not math.isfinite(x['time']) for x in imus):
                raise ValueError('postcapture_imu_invalid')
            if previous_imus is not None:
                imu_deltas = [x['time']-y['time'] for x,y in zip(imus,previous_imus)]
                imu_valid = all(abs(dt-1/120)<=1e-7 for dt in imu_deltas)
            previous_imus = imus
            samples.append(dict(encoder=row, imus=imus))
        return dict(start_step=start, end_step=read_step(), physics_steps=16, samples=samples,
                    capture_step=self.capture_step, capture_lower_bound_ns=self.lower,
                    native_ready_complete_before_steps=True, motion_authorized=False,
                    imu_deltas_s=imu_deltas, imu_distinct_120hz_valid=imu_valid, observation_admitted=False)
