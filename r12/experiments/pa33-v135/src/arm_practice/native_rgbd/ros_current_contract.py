"""Original actual encoder sample on authenticated ROS producer socket."""
from .priority_feedback import PriorityFeedbackReceiver
CORE=('step','read_start_ns','read_end_ns','q','velocity','velocity_valid','sim_time_s')
class CurrentROSGuard:
    def __init__(self,run,source):
        self.validator=PriorityFeedbackReceiver(None,producer_pid=-1,uid=-1,run_id=run,source_sha256=source,stream_id='ros-current-'+run);self.last_sequence=-1;self.last_step=-1;self.latest=None
    def validate(self,data,now,commit=True):
        step=self.validator.validate(data,now)
        if commit:
            if data['sequence']<=self.last_sequence or step<=self.last_step:raise ValueError('ROS_feedback_replay_or_reorder')
            self.last_sequence=data['sequence'];self.last_step=step;self.latest=data['feedback']
        return step
    def match_image_plan(self,current):
        if self.latest is None:raise ValueError('ROS_feedback_missing')
        if any(current['velocity'][k]!=self.latest['velocity'][k]for k in CORE)or current['position']['q']!=self.latest['position']['q']or current['position']['physics_step']!=self.last_step:raise ValueError('ROS_feedback_image_epoch_mismatch')
        return True
