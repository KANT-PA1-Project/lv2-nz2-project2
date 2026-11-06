"""Inactive dedicated small feedback socket candidate; no RGB dependency.

Call only with a separately authenticated producer socket. No public JSON can
create a receipt; read_packet performs its own socket read and peer check.
"""
import copy
import json
import math
import time
from .trusted_capture_ipc import receive_packet,peer_credentials,require_peer

class PriorityFeedbackReceiver:
    def __init__(self,sock,*,producer_pid,uid,run_id,source_sha256,stream_id):
        self.sock=sock;self.pid=producer_pid;self.uid=uid
        self.identity=dict(run_id=run_id,source_sha256=source_sha256,stream_id=stream_id)
        self.last_sequence=-1;self.last_step=-1;self.pending={};self.receipt_times={}
    def validate(self,data,now):
        if data['identity']!=self.identity or data['kind']!='current_feedback':raise ValueError('feedback_session_mismatch')
        if type(data['sequence'])is not int or data['sequence']<0:raise ValueError('invalid_feedback_sequence')
        feedback=data['feedback'];pos=feedback['position'];vel=feedback['velocity']
        if any(type(step)is not int or step<0 for step in [pos['physics_step'],vel['step']]) or pos['physics_step']!=vel['step'] or vel['velocity_valid'] is not True:raise ValueError('feedback_epoch_mismatch')
        for sample in [pos,vel]:
            start,end=sample['read_start_ns'],sample['read_end_ns']
            if any(type(v)is not int for v in [start,end,now]) or not 0<=start<=end<=now or end-start>1_000_000:raise ValueError('invalid_feedback_clock')
            if now-start>100_000_000:raise ValueError('feedback_older_than_100ms')
        for values in [pos['q'],vel['q'],vel['velocity']]:
            if not isinstance(values,(list,tuple)) or len(values)!=5 or not all(type(v) in (int,float) and math.isfinite(v)for v in values):raise ValueError('invalid_feedback_values')
        if pos['q']!=vel['q']:raise ValueError('current_position_velocity_q_mismatch')
        return pos['physics_step']
    def receive(self):
        require_peer(peer_credentials(self.sock),self.pid,self.uid)
        data,blobs=receive_packet(self.sock)
        if blobs or len(json.dumps(data))>16384:raise ValueError('large_payload_forbidden_on_feedback_channel')
        now=time.monotonic_ns();step=self.validate(data,now)
        if data['sequence']<=self.last_sequence or step<=self.last_step:raise ValueError('feedback_replay_or_reorder')
        # One outstanding receipt: callers must consume/discard it before reading again.
        if self.pending:raise ValueError('unconsumed_feedback_receipt')
        handle=object();self.pending[handle]=copy.deepcopy(data);self.last_sequence=data['sequence'];self.last_step=step
        self.receipt_times[handle]=now
        return handle
    def consume(self,handle):
        if handle not in self.pending:raise ValueError('live_feedback_receipt_required')
        data=self.pending.pop(handle);received_ns=self.receipt_times.pop(handle);now=time.monotonic_ns();self.validate(data,now)
        return dict(feedback=copy.deepcopy(data['feedback']),evaluated_at_ns=now,received_at_ns=received_ns,feedback_fresh=True,motion_authorized=False,requires_independent_capture_and_safety_gates=True)
