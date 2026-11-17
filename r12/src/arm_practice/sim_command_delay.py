"""Opt-in simulator command-delay primitive, not connected to robot runtime.

Caller advances physics independently and polls in ONE simulated nanosecond
clock domain. Original sensor capture time is never rewritten. Safety stops
bypass delay; this queue is not a substitute for current sensor/actuator gates.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Command:
    sequence: int
    capture_ns: int
    due_ns: int
    velocity: tuple

class SimCommandDelay:
    def __init__(self, run_id, *, delay_ns, max_age_ns, capacity=64):
        if not isinstance(run_id,str) or not run_id:
            raise ValueError('run_id required')
        for value in (delay_ns,max_age_ns,capacity):
            if type(value) is not int:raise ValueError('integer limits required')
        if delay_ns<0 or max_age_ns<=0 or capacity<=0:raise ValueError('invalid limits')
        self.run_id=run_id;self.delay_ns=delay_ns;self.max_age_ns=max_age_ns;self.capacity=capacity
        self.pending=[];self.active=None;self.last_time=-1;self.last_sequence=-1;self.last_capture=-1;self.fault=None

    def _fail(self,reason):
        self.pending.clear();self.active=None;self.fault=reason
        return False

    def _time(self,now_ns):
        if self.fault:return False
        if type(now_ns) is not int or now_ns<0 or now_ns<self.last_time:return self._fail('invalid_or_reversed_clock')
        self.last_time=now_ns
        return True

    def submit(self, *, run_id, sequence, capture_ns, now_ns, velocity):
        if not self._time(now_ns):return False
        if run_id!=self.run_id:return self._fail('wrong_run')
        if type(sequence) is not int or sequence<=self.last_sequence:return self._fail('out_of_order_sequence')
        if type(capture_ns) is not int or not 0<=capture_ns<=now_ns or capture_ns<self.last_capture:return self._fail('invalid_capture_time')
        if now_ns-capture_ns>self.max_age_ns:return self._fail('expired_at_submission')
        try:v=tuple(float(x) for x in velocity)
        except (TypeError,ValueError):return self._fail('invalid_velocity')
        if len(v)!=3 or not all(math.isfinite(x) for x in v):return self._fail('invalid_velocity')
        if len(self.pending)>=self.capacity:return self._fail('queue_overflow')
        self.pending.append(Command(sequence,capture_ns,now_ns+self.delay_ns,v))
        self.last_sequence=sequence;self.last_capture=capture_ns
        return True

    def sample(self, now_ns, *, safety_stop=False):
        if not self._time(now_ns):return self._zero(self.fault)
        if safety_stop:
            self.pending.clear();self.active=None
            return self._zero('safety_stop')
        due=[x for x in self.pending if x.due_ns<=now_ns]
        self.pending=[x for x in self.pending if x.due_ns>now_ns]
        if due:self.active=max(due,key=lambda x:x.sequence)
        if self.active is None:return self._zero('await_command')
        age=now_ns-self.active.capture_ns
        if age>self.max_age_ns:
            self.active=None
            return self._zero('expired_at_application')
        return {'velocity':self.active.velocity,'reason':'delayed_command','sequence':self.active.sequence,'capture_ns':self.active.capture_ns,'age_ns':age,'clock_domain':'simulated_ns'}

    @staticmethod
    def _zero(reason):
        return {'velocity':(0.,0.,0.),'reason':reason,'sequence':None,'capture_ns':None,'age_ns':None,'clock_domain':'simulated_ns'}
