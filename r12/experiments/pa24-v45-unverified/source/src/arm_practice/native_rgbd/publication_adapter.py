"""CPU publication plan; a live trusted certificate verifier is mandatory upstream.

No ROS publishers or mission admission. Legacy packets never enter this adapter.
"""
import copy
import math
from fractions import Fraction
from .packet_timeline import NativePacketTimeline,stamp

def exact_float_stamp(t):
    # Preserve actual sensor timestamp; expose exact rational and ns floor error.
    value=Fraction.from_float(t);ns=value*1_000_000_000
    return dict(stamp_ns=ns.numerator//ns.denominator,native_time_num=value.numerator,native_time_den=value.denominator,native_time_s=t)

class NativePublicationAdapter:
    def __init__(self,*,run_id,initial_step,verify_capture):
        if not callable(verify_capture):raise ValueError('trusted_capture_verifier_required')
        self.run_id=run_id;self.timeline=NativePacketTimeline(run_id,initial_step);self.verify_capture=verify_capture;self.last_capture=-1

    def prepare(self,*,packet,certificate,now_ns):
        # Work transactionally: a rejected packet cannot advance time or drop samples.
        p=copy.deepcopy(packet);body=copy.deepcopy(self.verify_capture(certificate))
        t=p['native_packet_timeline'];feedback=p['current_feedback']
        if t['clock_domain']!='native_physics_960hz' or p['run_id']!=self.run_id or t['run_id']!=self.run_id:raise ValueError('wrong_native_clock_or_run')
        if body['run_id']!=self.run_id or body['physics_hz']!=960 or body['capture_physics_step']<=self.last_capture:raise ValueError('capture_run_order_or_rate')
        if type(now_ns)is not int or not body['issued_ns']<=now_ns or not 0<=now_ns-body['capture_monotonic_lower_bound_ns']<=500_000_000:raise ValueError('capture_wall_stale')
        if t['previous_packet_step']!=self.timeline.last_packet_step:raise ValueError('missing_or_reordered_native_packet')
        if t['capture_physics_step']!=body['capture_physics_step'] or t['capture_monotonic_lower_bound_ns']!=body['capture_monotonic_lower_bound_ns']:raise ValueError('capture_packet_identity_mismatch')
        trial=copy.deepcopy(self.timeline)
        streams=[x['native_samples']for x in p['all_IMU_samples']]
        trial.add(streams)
        calculated=trial.packet(current_step=t['current_physics_step'],capture_body=body)
        if calculated!=t or p['all_IMU_samples']!=t['imu']:raise ValueError('modified_native_timeline')
        current=t['current_physics_step'];position=feedback['position'];velocity=feedback['velocity']
        if position['physics_step']!=current or velocity['step']!=current or not velocity['velocity_valid']:raise ValueError('current_encoder_epoch_mismatch')
        for f in [position,velocity]:
            if not body['issued_ns']<=f['read_start_ns']<=f['read_end_ns']<=now_ns or now_ns-f['read_start_ns']>100_000_000:raise ValueError('current_encoder_stale')
        if any(len(feedback[k][v])!=5 or not all(math.isfinite(x)for x in feedback[k][v])for k,v in [('position','q'),('velocity','q'),('velocity','velocity')]):raise ValueError('invalid_current_encoder')
        history=body['encoder_history'];capture=body['capture_physics_step']
        if history['physics_step']!=capture or history['canonical_sim_time_num']!=capture or history['canonical_sim_time_den']!=960:raise ValueError('historical_fk_epoch_mismatch')
        historical=dict(stamp_ns=stamp(capture),clock_domain='native_physics_960hz',capture_physics_step=capture,capture_monotonic_lower_bound_ns=body['capture_monotonic_lower_bound_ns'],camera_tf=copy.deepcopy(history['camera_fk']),image_hashes=copy.deepcopy(body['pixel_sha256']),depth_hashes=copy.deepcopy(body['depth_sha256']),perception_request=dict(stamp_ns=stamp(capture),capture_monotonic_ns=body['capture_monotonic_lower_bound_ns'],clock_domain='native_physics_960hz',run_id=self.run_id),publication_authorized=False)
        imu=[]
        # Publish each native sample identity, never window means at current time.
        for index,stream in enumerate(streams):
            for sample in stream:
                imu.append(dict(sensor_index=index,frame_id=['opencr_front_imu','opencr_rear_imu'][index],clock_domain='native_imu_float32_time',**exact_float_stamp(sample['time']),physics_read_step=sample['physics_step'],gyro=copy.deepcopy(sample['gyro']),accel=copy.deepcopy(sample['accel'])))
        imu.sort(key=lambda x:(x['stamp_ns'],x['sensor_index']))
        result=dict(clock=dict(stamp_ns=stamp(current),clock_domain='native_physics_960hz'),current_encoder=dict(stamp_ns=stamp(current),physics_step=current,position=copy.deepcopy(position),velocity=copy.deepcopy(velocity)),historical=historical,imu_native_samples=imu,mission_admitted=False,publication_authorized=False,scope='CPU_publication_plan_only')
        self.timeline=trial;self.last_capture=capture
        return result
