"""Opt-in stationary acquisition inside r12_sensor_sim; no perception/motion admission."""
from fractions import Fraction
import hashlib
import json
import time
import os
from pathlib import Path
import numpy as np
from .mission_capture import NativeMissionCapture
from .renderer_epoch_issuer import RendererEpochIssuer

MAX_CURRENT_AGE_NS=100_000_000

def quantized(q):
    return np.rint(np.asarray(q)*4096/(2*np.pi))*2*np.pi/4096

def atomic(path,value):
    p=path.with_suffix(path.suffix+'.tmp');p.write_text(json.dumps(value,indent=2));p.replace(path)

class EncoderVelocityHistory:
    """Actual120Hz distinct physics samples. Wall age never refreshed by a repeated step."""
    def __init__(self):self.previous=None;self.latest=None
    def sample(self,*,q,step,sim_time_s,read_start_ns,read_end_ns):
        if type(step)is not int or type(read_start_ns)is not int or type(read_end_ns)is not int or read_start_ns>read_end_ns:raise ValueError('invalid_control_encoder_identity')
        q=np.asarray(q,float)
        if q.shape!=(5,) or not np.isfinite(q).all():raise ValueError('invalid_control_encoder_q')
        row=dict(q=q.tolist(),step=step,sim_time_s=sim_time_s,read_start_ns=read_start_ns,read_end_ns=read_end_ns,velocity=None,velocity_valid=False)
        if self.previous is not None:
            old=self.previous;dt=sim_time_s-old['sim_time_s']
            if step<=old['step'] or dt<=0:raise ValueError('repeated_control_encoder_epoch')
            row.update(velocity=(((q-np.asarray(old['q'])+np.pi)%(2*np.pi)-np.pi)/dt).tolist(),velocity_valid=abs(dt-1/120)<1e-8)
        self.previous=row;self.latest=row
        return row
    def current(self,now_ns):
        if self.latest is None:return dict(valid=False,reason='no_actual_control_encoder_velocity')
        r=dict(self.latest);age=now_ns-r['read_start_ns'];r.update(age_ns=age,valid=bool(r['velocity_valid']and 0<=age<=MAX_CURRENT_AGE_NS),reason='fresh'if r['velocity_valid']and 0<=age<=MAX_CURRENT_AGE_NS else'control_encoder_velocity_stale_or_missing')
        return r

class MissionNativeProducer:
    def __init__(self,*,root,out,run_id,schema,robot,world,arm,joint_names,rig,rgb,depth,rgb_refs,depth_refs,requested_epochs):
        self.requested_epochs=requested_epochs
        from .warmup_provenance import digest
        self.phase_envelope=json.loads((Path(out)/'precommitted_phase_plan.json').read_text())
        self.phase_plan=self.phase_envelope['plan'];self.phase_sha=os.environ['R12_PHASE_PLAN_SHA256']
        assert digest(self.phase_plan)==self.phase_sha==self.phase_envelope['plan_sha256']
        assert self.phase_plan['warmup_epochs']+self.phase_plan['measurement_epochs']==requested_epochs
        self.phase_verified_ns=time.monotonic_ns()
        assert self.phase_envelope['committed_monotonic_ns']<self.phase_verified_ns
        self.root=Path(root);self.out=Path(out);self.robot=robot;self.world=world;self.arm=list(arm);self.schema=schema;self.rows=[];self.pending=None;self.history=EncoderVelocityHistory()
        manifest=self.root/'native_entrypoint_manifest.json';m=json.loads(manifest.read_text());source=hashlib.sha256(manifest.read_bytes()).hexdigest();model=hashlib.sha256(json.dumps({k:v for k,v in m['sha256'].items()if k.startswith(('models/','assets/','configs/'))},sort_keys=True).encode()).hexdigest()
        self.issuer=RendererEpochIssuer(run_id=run_id,clock_epoch='native-mission-stationary-epoch1',cameras=list(rgb),joint_names=joint_names,physics_hz=960,model_sha256=model,source_sha256=source,fk=rig.transforms,depth_camera_map={k:k.replace('_depth','_left_optical')for k in depth},camera_schema=schema)
        self.capture=NativeMissionCapture(issuer=self.issuer,rgb=rgb,depth=depth,rgb_reference=rgb_refs,depth_reference=depth_refs,read_epoch=lambda:(int(world.current_time_step_index),float(world.current_time)),read_joint_positions=robot.get_joint_positions,arm_indices=self.arm,joint_names=joint_names,render=world.render)
        self.sample_control_encoder()
        from .live_decode_bridge import LiveDecodeBridge
        self.decoder_bridge=LiveDecodeBridge(root,out,run_id,schema,self.issuer,self.capture.read_current_feedback)
        from .packet_timeline import NativePacketTimeline
        self.timeline=NativePacketTimeline(run_id,int(world.current_time_step_index))
        self.ros_bridge=None;self.priority_bridge=None
        if os.getenv('R12_NATIVE_ROS_PUBLICATION')=='1':
            from .live_publication_bridge import LivePublicationBridge
            from .publication_adapter import NativePublicationAdapter
            self.publication_adapter=NativePublicationAdapter(run_id=run_id,initial_step=int(world.current_time_step_index),verify_capture=self.verify_for_publication)
            self.ros_bridge=LivePublicationBridge(root,out,run_id,schema)
            if os.getenv('R12_PRIORITY_FEEDBACK')=='1':
                from .priority_feedback_bridge import PriorityFeedbackBridge
                self.priority_bridge=PriorityFeedbackBridge(root,out,run_id,self.issuer.source)
        atomic(self.out/'native_producer_preflight.json',dict(run_id=run_id,source_sha256=source,model_calibration_sha256=model,camera_schema_sha256=schema.sha256,entrypoint='scripts/r12_sensor_sim.py',stationary=True,RGB_count=len(rgb),depth_count=len(depth),initial_physics_step=int(world.current_time_step_index),historical_camera_FK_only=True,current_control_feedback_separate=True,decoder_started=False,motion_authorized=False))
    def sample_control_encoder(self):
        step=int(self.world.current_time_step_index);t=float(self.world.current_time);start=time.monotonic_ns();q=quantized(self.robot.get_joint_positions()[self.arm]);end=time.monotonic_ns()
        if int(self.world.current_time_step_index)!=step:raise ValueError('physics_advanced_during_control_sample')
        return self.history.sample(q=q,step=step,sim_time_s=t,read_start_ns=start,read_end_ns=end)
    def current_feedback(self):
        f=self.capture.read_current_feedback();now=time.monotonic_ns();age=now-f['read_start_ns'];v=self.history.current(now)
        return dict(position=f,position_age_ns=age,position_valid=0<=age<=MAX_CURRENT_AGE_NS,velocity=v,control_admitted=False,reason='producer_diagnostic_only_no_actuator_consumer',historical_q_used_as_current=False)
    def acquire(self,frame,*,command,substeps,imu_samples):
        if self.pending is not None:raise ValueError('unflushed_native_capture')
        if np.any(np.asarray(command)!=0):raise ValueError('nonzero_motion_in_native_stationary')
        if substeps!=48 or any(len(x)!=6 or not all(s['valid']for s in x)for x in imu_samples):raise ValueError('native_physics_or_IMU_contract_failed')
        self.timeline.add(imu_samples)
        result=self.capture.acquire();row=dict(frame=frame,physics_step=int(self.world.current_time_step_index),captured=result['captured'],attempts=result['attempts'],reasons=result.get('reasons',[]),ready_age_ms=result.get('ready_age_ms'),substeps=substeps,imu_counts=[len(x)for x in imu_samples],command=list(map(float,command)),motion_authorized=False,decoder_started=False,observation_admitted=False)
        if result['captured']:
            body=result['certificate']['body'];verification=dict(deferred_to_live_publication=True) if self.ros_bridge else self.issuer.verify(result['certificate'],pixels=result['pixels'],depths=result['depths'],now_ns=time.monotonic_ns(),expected_current_run=self.issuer.run_id)
            row.update(capture_native_ns=int(Fraction(body['capture_physics_step'],body['physics_hz'])*1_000_000_000),capture_monotonic_ns=body['capture_monotonic_lower_bound_ns'],capture_encoder=body['encoder_history'],native_verification=verification)
        row['ready_trace']=result.get('ready_trace',[])
        self.rows.append(row);self.pending=result;self.pending_frame=frame
        return result
    def refresh_after_capture(self,stepper,read_imus):
        from .postcapture_feedback import PostCaptureFeedback
        if self.pending is None or not self.pending['captured']:raise ValueError('no_producer_owned_capture')
        b=self.pending['certificate']['body']
        before=hashlib.sha256(json.dumps(b,sort_keys=True).encode()).hexdigest()
        sequence=PostCaptureFeedback(capture_step=b['capture_physics_step'],capture_lower_bound_ns=b['capture_monotonic_lower_bound_ns'],capture_completed_ns=b['issued_ns'],physics_hz=960)
        result=sequence.run(stepper=stepper,read_step=lambda:int(self.world.current_time_step_index),sample_encoder=self.sample_control_encoder,read_imus=read_imus)
        if hashlib.sha256(json.dumps(b,sort_keys=True).encode()).hexdigest()!=before:raise ValueError('historical_capture_mutated_during_current_refresh')
        self.timeline.add([[dict(row['imus'][i],physics_step=row['encoder']['step'],physics_time_s=row['encoder']['sim_time_s']) for row in result['samples']] for i in range(2)])
        result['original_capture_body_unchanged']=True
        result['current_feedback']=self.current_feedback()
        if self.priority_bridge:result['priority_feedback']=self.priority_bridge.send(result['current_feedback'])
        if self.ros_bridge:result['early_ROS_feedback']=self.ros_bridge.publish_current(result['current_feedback'],self.issuer.source,self.pending_frame+1)
        result['image_age_ns_after_refresh']=time.monotonic_ns()-b['capture_monotonic_lower_bound_ns']
        result['image_fresh_after_refresh']=0<=result['image_age_ns_after_refresh']<=500_000_000
        result['observation_admitted']=False
        self.rows[-1]['postcapture_feedback']=result
        return result
    def verify_for_publication(self,certificate):
        if self.pending is None or certificate!=self.pending['certificate']:raise ValueError('not_owned_live_capture')
        verify_start=time.monotonic_ns()
        result=self.issuer.verify(certificate,pixels=self.pending['pixels'],depths=self.pending['depths'],now_ns=time.monotonic_ns(),expected_current_run=self.issuer.run_id)
        if not result['rgbd_capture_association_verified']:raise ValueError('live_capture_verification_failed:'+result['reason'])
        self.rows[-1]['live_HMAC_hash_timing']=dict(start_ns=verify_start,end_ns=time.monotonic_ns())
        self.rows[-1]['native_verification']=result
        return certificate['body']
    def publish_native_packet(self,timing,current):
        if not self.ros_bridge:return
        packet=dict(run_id=self.issuer.run_id,native_packet_timeline=timing,current_feedback=current,all_IMU_samples=timing['imu'])
        prepare_start=time.monotonic_ns()
        plan=self.publication_adapter.prepare(packet=packet,certificate=self.pending['certificate'],now_ns=time.monotonic_ns())
        self.rows[-1]['publication_adapter_timing']=dict(start_ns=prepare_start,end_ns=time.monotonic_ns())
        self.decoder_bridge.send(self.pending['certificate'],self.pending['pixels'],self.pending['depths'],self.rows[-1]['native_verification'],self.pending_frame)
        self.decoder_bridge.verify_before_publication()
        self.rows[-1]['ROS_publication']=self.ros_bridge.publish(self.pending['certificate'],self.pending['pixels'],self.pending['depths'],plan,self.rows[-1]['native_verification'],decoder=self.decoder_bridge)
    def native_packet_timing(self):
        return self.timeline.packet(current_step=int(self.world.current_time_step_index),capture_body=self.pending['certificate']['body'])
    def packet_summary(self,current):
        row=self.rows[-1];row['independent_current_feedback']=current
        return dict(captured=row['captured'],native_physics_step=row['physics_step'],native_time_ns=row.get('capture_native_ns'),synthetic_packet_time_is_not_capture_time=True,current_feedback=current,observation_admitted=False,reason='producer_only_no_identity_pose_clearance_consumer')
    def finish_frame(self,frame):
        r=self.pending
        if r is None or frame!=self.pending_frame:raise ValueError('native_capture_frame_mismatch')
        if r['captured']:
            self.rows[-1]['live_decoder']=self.decoder_bridge.collect(frame)
            # Copies are owned, read-only and still separate from legacy masks.
            raw=self.out/f'native_raw_{frame:02}.npz'
            np.savez(raw,**r['pixels'],**r['depths'])
            atomic(self.out/f'native_certificate_{frame:02}.json',r['certificate'])
            self.rows[-1].update(raw_archive=raw.name,raw_archive_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),archive_not_a_freshness_refresh=True)
        self.pending=None
        atomic(self.out/'native_producer_frames.json',self.rows)
    def close(self):
        self.decoder_bridge.close()
        if self.priority_bridge:self.priority_bridge.close()
        if self.ros_bridge:self.ros_bridge.close()
        if self.pending is not None:raise ValueError('unflushed_native_capture_at_shutdown')
        from .measurement_completion import summarize
        atomic(self.out/'native_producer_completion.json',summarize(self.rows,self.requested_epochs))
        from .warmup_provenance import replay
        raw=(self.out/'native_producer_frames.json').read_bytes()
        accounting=replay(raw,self.phase_plan,self.phase_sha)
        accounting.update(provenance='Native run with fixed phase plan committed before child launch',precommit=self.phase_envelope,producer_plan_verified_monotonic_ns=self.phase_verified_ns,live_freshness_authorized=False)
        atomic(self.out/'phase_accounting.json',accounting)
