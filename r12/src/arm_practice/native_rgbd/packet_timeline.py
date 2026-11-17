"""Native physics packet timing candidate. Does not grant mission admission."""
import copy
import math
import struct

HZ=960
STRIDE=8

def f32(x):return struct.unpack('f',struct.pack('f',x))[0]
def stamp(step):return step*1_000_000_000//HZ

def radius(t):
    # Half an ULP at the timestamp's magnitude, explicit float32 representation.
    return 2**(math.floor(math.log2(t))-24)

def validate_sample(sample):
    step=sample['physics_step']; t=sample['time']; sim=sample['physics_time_s']
    if type(step)is not int or step<=0 or not math.isfinite(sim) or not math.isfinite(t) or t<=0 or not sample['valid']:
        raise ValueError('invalid_native_imu_identity')
    # World advances with float32(1/960); binary64 accumulation error is bounded
    # by one ULP per addition. This is numerical error, not timing slack.
    expected_sim=step*f32(1/HZ)
    accumulation_bound=(step+1)*math.ulp(expected_sim)
    if abs(sim-expected_sim)>accumulation_bound:raise ValueError('physics_step_time_mismatch')
    if t!=f32(t) or t!=f32(sim-1/HZ):raise ValueError('imu_phase_or_representation_mismatch')
    if radius(t)>=1/480:raise ValueError('imu_timestamp_precision_insufficient')
    if any(len(sample[k])!=3 or not all(math.isfinite(v)for v in sample[k])for k in ['gyro','accel']):raise ValueError('invalid_imu_vector')

def validate_pair(a,b):
    if b['physics_step']-a['physics_step']!=STRIDE or b['time']<=a['time']:
        raise ValueError('missing_or_repeated_imu_step')
    if abs(b['time']-a['time']-1/120)>radius(a['time'])+radius(b['time']):
        raise ValueError('imu_cadence_outside_representation_intervals')

class NativePacketTimeline:
    def __init__(self,run_id,initial_step,max_pending_samples=240):
        self.run_id=run_id;self.initial_step=initial_step;self.last_packet_step=initial_step
        self.pending=[[],[]];self.last=[None,None]
        if type(max_pending_samples)is not int or not 1<=max_pending_samples<=240:raise ValueError("invalid_pending_budget")
        self.max_pending_samples=max_pending_samples
    def add(self,streams):
        if len(streams)!=2 or len(streams[0])!=len(streams[1]):raise ValueError('incomplete_imu_pair')
        if any(len(self.pending[i])+len(streams[i])>self.max_pending_samples for i in range(2)):raise ValueError('native_pending_sample_budget_exceeded')
        trial=copy.deepcopy(self.last)
        for i,stream in enumerate(streams):
            for sample in stream:
                validate_sample(sample)
                if trial[i] is not None:validate_pair(trial[i],sample)
                elif sample['physics_step']!=self.initial_step+STRIDE:raise ValueError('missing_initial_imu_sample')
                trial[i]=sample
        if [x['physics_step']for x in streams[0]]!=[x['physics_step']for x in streams[1]]:raise ValueError('imu_pair_epoch_mismatch')
        for i in range(2):self.pending[i].extend(copy.deepcopy(streams[i]))
        self.last=copy.deepcopy(trial)
    def packet(self,*,current_step,capture_body):
        if type(current_step)is not int or current_step<=self.last_packet_step:raise ValueError('invalid_current_packet_step')
        if capture_body['run_id']!=self.run_id or capture_body['physics_hz']!=HZ or capture_body['capture_physics_step']+16!=current_step:
            raise ValueError('capture_current_epoch_mismatch')
        expected=(current_step-self.last_packet_step)//STRIDE
        if (current_step-self.last_packet_step)%STRIDE or any(len(s)!=expected or not s or s[-1]['physics_step']!=current_step for s in self.pending):raise ValueError('incomplete_native_packet_interval')
        summaries=[]
        for stream in self.pending:
            summaries.append(dict(valid=True,gyro=[sum(s['gyro'][k]for s in stream)/expected for k in range(3)],accel=[sum(s['accel'][k]for s in stream)/expected for k in range(3)],endpoint_gyro=stream[-1]['gyro'],native_sample_count=expected,native_time_range=[stream[0]['time'],stream[-1]['time']],native_samples=copy.deepcopy(stream)))
        result=dict(clock_domain='native_physics_960hz',run_id=self.run_id,stamp_ns=stamp(current_step),current_physics_step=current_step,previous_packet_step=self.last_packet_step,packet_dt_ns=stamp(current_step)-stamp(self.last_packet_step),capture_stamp_ns=stamp(capture_body['capture_physics_step']),capture_physics_step=capture_body['capture_physics_step'],capture_monotonic_lower_bound_ns=capture_body['capture_monotonic_lower_bound_ns'],imu=summaries,observation_admitted=False,motion_authorized=False)
        self.pending=[[],[]];self.last_packet_step=current_step
        return result
