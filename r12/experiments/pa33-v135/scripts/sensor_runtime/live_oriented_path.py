"""Opt-in live Nav2/nvBlox candidate boundary; never an actuator authority.

ROS callbacks pass their actual message objects here. Grid snapshots are not
replaced by the known maze map. Only map-frame, axis-aligned DistanceMapSlice
messages are supported; a different frame requires a reviewed TF adapter.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import math
import subprocess
import sys
import time
import numpy as np
from arm_practice.oriented_plan_guard import accept_oriented_plan
from oriented_path_adapter import heading_gate, uncertainty_footprint
from swept_footprint import motion_clear


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _stamp(msg):
    return int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)


def _atomic(path, value):
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


@dataclass(frozen=True)
class NavPathRequest:
    sequence: int
    goal_index: int
    map_revision: str
    footprint_revision: str
    sent_monotonic_s: float


@dataclass(frozen=True)
class PendingPath:
    request: NavPathRequest
    received_monotonic_s: float
    frame: str
    stamp_ns: int
    points: tuple
    pose_frames: tuple


class LiveOrientedPath:
    """One subprocess candidate at a time, with explicit request generations."""
    def __init__(self, footprint, directory, *, clock=time.monotonic,
                 max_sensor_age=.15, max_map_age=1., max_plan_age=5.,
                 worker_timeout=8., position_margin=.01,
                 yaw_error=math.radians(2), worker_python=None,
                 max_path_callback_age=1.):
        self.footprint = np.asarray(footprint, float).copy()
        if self.footprint.ndim != 2 or self.footprint.shape[1] != 2 or len(self.footprint)<3 or not np.isfinite(self.footprint).all():
            raise ValueError('Finite padded CAD footprint required')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.clock = clock
        self.max_sensor_age, self.max_map_age, self.max_plan_age = max_sensor_age, max_map_age, max_plan_age
        self.worker_timeout, self.worker_python = worker_timeout, worker_python or sys.executable
        self.position_margin, self.yaw_error = position_margin, yaw_error
        self.footprint_revision = _hash({'vertices':self.footprint.tolist(),
            'position_margin':position_margin, 'yaw_error':yaw_error})
        self.map = self.sensor = self.plan = self.plan_local = None
        self.path = self.active_request = None
        self.goal_index = None
        self.phase = 'await_stop'
        self.reason = 'waiting_live_path_and_slice'
        self.sequence = 0
        self._nav_sequence = 0
        self._nav_request = self._pending_path = None
        self.max_path_callback_age = max_path_callback_age
        self.path_queue_reason = 'empty'
        self._future = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='oriented-ipc')
        self.audit = []

    def _clear(self, reason, *, clear_path=False):
        self.plan = self.plan_local = None
        self.active_request = None
        self.phase, self.reason = 'await_stop', reason
        if clear_path:self.path = None

    def set_goal(self, goal_index):
        if type(goal_index) is not int or goal_index < 0:
            raise ValueError('Nonnegative integer goal index required')
        if goal_index != self.goal_index:
            self.goal_index = goal_index
            self._clear('goal_changed', clear_path=True)
            self._invalidate_nav('goal_changed')

    def _invalidate_nav(self, reason):
        self._nav_request = self._pending_path = None
        self.path_queue_reason = reason

    def _expire_nav_request(self):
        request = self._nav_request
        if request is None:return
        age = self.clock()-request.sent_monotonic_s
        if not math.isfinite(age) or age<0 or age>self.max_plan_age:
            reason = 'nav_request_timeout' if math.isfinite(age) and age>self.max_plan_age else 'nav_request_clock_invalid'
            self._invalidate_nav(reason)
            self._clear(reason,clear_path=True)

    def begin_path_request(self, goal_index):
        """Bind the Nav2 request to the exact live goal/map generation."""
        self._expire_nav_request()
        # The existing candidate process owns the only worker slot until
        # poll() drains it, including a done but not yet consumed result.
        if self._future is not None:return None
        if goal_index != self.goal_index or self.map is None:
            return None
        self._nav_sequence += 1
        self._nav_request = NavPathRequest(self._nav_sequence, goal_index,
            self.map['revision'], self.footprint_revision, self.clock())
        self._pending_path = None
        return self._nav_request

    def path_request_current(self, request):
        self._expire_nav_request()
        return bool(request is not None and request == self._nav_request and
            request.goal_index == self.goal_index and self.map is not None and
            request.map_revision == self.map['revision'] and
            request.footprint_revision == self.footprint_revision)

    def path_request_in_flight(self):
        return self.path_request_current(self._nav_request)

    def queue_path(self, msg, goal_index, request, *, received_monotonic_s):
        """Bounded one-message immutable queue. No old sensor-context check here."""
        if not self.path_request_current(request) or goal_index != self.goal_index:
            return dict(queued=False, reason='superseded_nav_request')
        if self._pending_path is not None:
            return dict(queued=False, reason='duplicate_nav_result')
        try:
            if not math.isfinite(received_monotonic_s):raise ValueError('invalid_callback_clock')
            if not 2<=len(msg.poses)<=10000:raise ValueError('path_geometry_invalid')
            points = tuple((float(p.pose.position.x),float(p.pose.position.y)) for p in msg.poses)
            if not np.isfinite(points).all():raise ValueError('path_geometry_invalid')
            self._pending_path = PendingPath(request, float(received_monotonic_s),
                str(msg.header.frame_id), _stamp(msg), points,
                tuple(str(p.header.frame_id) for p in msg.poses))
            self.path_queue_reason = 'waiting_current_packet_sensor'
            return dict(queued=True, reason=self.path_queue_reason,
                        nav_request_sequence=request.sequence,
                        received_monotonic_s=received_monotonic_s)
        except (ValueError,TypeError,OverflowError,AttributeError) as error:
            self._invalidate_nav('invalid_queued_path:'+str(error))
            return dict(queued=False,reason=self.path_queue_reason)

    def consume_pending_path(self):
        """Call only after this packet's note_sensor; retain all original clocks."""
        pending = self._pending_path
        if pending is None:return None
        self._pending_path = None
        now = self.clock()
        reason = None
        if not self.path_request_current(pending.request):
            reason=self.path_queue_reason if self.path_queue_reason in ('nav_request_timeout','nav_request_clock_invalid') else 'superseded_nav_request'
        elif not 0<=now-pending.received_monotonic_s<=self.max_path_callback_age:reason='path_callback_stale'
        elif not 0<=now-pending.request.sent_monotonic_s<=self.max_plan_age:reason='nav_request_stale'
        self._nav_request = None
        if reason:
            self._clear(reason,clear_path=True)
            self.path_queue_reason = reason
            return dict(submitted=False,reason=reason)
        header = SimpleNamespace(frame_id=pending.frame,stamp=SimpleNamespace(
            sec=pending.stamp_ns//10**9,nanosec=pending.stamp_ns%10**9))
        msg = SimpleNamespace(header=header,poses=[SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),pose=SimpleNamespace(
                position=SimpleNamespace(x=p[0],y=p[1])))
            for p,frame in zip(pending.points,pending.pose_frames)])
        context = dict(nav_request_sequence=pending.request.sequence,
            nav_request_sent_monotonic_s=pending.request.sent_monotonic_s,
            path_received_monotonic_s=pending.received_monotonic_s,
            path_consumed_monotonic_s=now)
        verdict = self.on_path(msg,pending.request.goal_index,nav_context=context)
        self.path_queue_reason = verdict.get('reason','submitted')
        return {**verdict,**context}

    def note_sensor(self, pose, twist, *, sim_stamp_ns, received_monotonic_s,
                    valid, transport_settled):
        p, v = np.asarray(pose if pose is not None else [], float), np.asarray(twist, float)
        valid = bool(valid and p.shape == (3,) and v.shape == (3,) and
                     np.isfinite(np.r_[p, v, received_monotonic_s]).all())
        self.sensor = dict(pose=p.tolist(), twist=v.tolist(), sim_stamp_ns=int(sim_stamp_ns),
            received=float(received_monotonic_s), valid=valid,
            transport_settled=bool(transport_settled))

    def on_slice(self, msg):
        """Preserve every geometry/content field; receipt/header times stay separate."""
        try:
            frame = msg.header.frame_id
            width, height = int(msg.width), int(msg.height)
            res, unknown = float(msg.resolution), float(msg.unknown_value)
            origin = [float(msg.origin.x), float(msg.origin.y), float(msg.origin.z)]
            data = np.asarray(msg.data, dtype=np.float32).copy()
            if frame != 'map':raise ValueError('slice_frame_requires_explicit_tf')
            if min(width,height)<2 or width*height>2_000_000 or data.size!=width*height:raise ValueError('slice_shape_invalid')
            if not np.isfinite([res,*origin]).all() or not 0<res<=.25:raise ValueError('slice_geometry_invalid')
            # Nonfinite and sentinel cells are occupied, never free. Half-cell
            # diagonal conservatively covers a zero surface inside a grid cell.
            known = np.isfinite(data) & (data != unknown)
            occupied = (~known | (data <= res/math.sqrt(2))).reshape(height,width)
            raw_hash = hashlib.sha256(data.tobytes(order='C')).hexdigest()
            metadata = dict(frame=frame, width=width, height=height, resolution=res,
                origin=origin, unknown_value_bits=np.float32(unknown).tobytes().hex(),
                data_sha256=raw_hash, axis_order='row_y_col_x')
            # The worker plans only from occupied/unknown cells and derives
            # its clearance from that mask. Preserve raw ESDF provenance, but
            # do not revoke a plan for harmless float updates in the same mask.
            geometry = dict(frame=frame,width=width,height=height,resolution=res,
                origin=origin,axis_order='row_y_col_x',
                occupied_sha256=hashlib.sha256(occupied.tobytes()).hexdigest(),
                unknown_sha256=hashlib.sha256((~known).tobytes()).hexdigest())
            revision = _hash(geometry)
            data.flags.writeable=False
            occupied.flags.writeable=False
            old = None if self.map is None else self.map['revision']
            self.map = dict(metadata, geometry=geometry, revision=revision, raw=data.reshape(height,width),
                occupied=occupied, stamp_ns=_stamp(msg), received=self.clock())
            if old != revision:
                self._clear('map_content_changed')
                self._invalidate_nav('map_content_changed')
            return dict(accepted=True, changed=old!=revision, revision=revision,
                        occupied_cells=int(occupied.sum()))
        except (ValueError, TypeError, OverflowError, AttributeError) as error:
            self.map = None
            self._clear('invalid_slice:'+str(error))
            self._invalidate_nav('invalid_slice')
            return dict(accepted=False, reason=self.reason)

    def _context_reason(self):
        now = self.clock()
        if self.sensor is None or not self.sensor['valid']:return 'sensor_invalid'
        if not 0<=now-self.sensor['received']<=self.max_sensor_age:return 'sensor_stale'
        if self.map is None:return 'live_slice_missing'
        if not 0<=now-self.map['received']<=self.max_map_age:return 'slice_receipt_stale'
        if not 0<=(self.sensor['sim_stamp_ns']-self.map['stamp_ns'])/1e9<=self.max_map_age:return 'slice_sim_time_stale'
        return None

    def on_path(self, msg, goal_index, *, nav_context=None):
        """Actual ComputePathToPose result Path; no route-file fallback."""
        if goal_index != self.goal_index:return dict(submitted=False, reason='stale_goal_callback')
        try:
            if msg.header.frame_id != 'map':raise ValueError('path_frame_invalid')
            p = np.asarray([[x.pose.position.x,x.pose.position.y] for x in msg.poses],float)
            if p.ndim!=2 or p.shape[1]!=2 or not 2<=len(p)<=10000 or not np.isfinite(p).all():raise ValueError('path_geometry_invalid')
            if any(x.header.frame_id not in ('','map') for x in msg.poses):raise ValueError('path_pose_frame_invalid')
            reason = self._context_reason()
            if reason:raise ValueError(reason)
            if not -.05<=(self.sensor['sim_stamp_ns']-_stamp(msg))/1e9<=1.:raise ValueError('path_stamp_stale')
            revision = _hash({'frame':'map','points':p.tolist()})
            # Stamp-only updates do not throw away a validated preparation or
            # tracking phase. The runtime avoids routine replans while valid.
            same = self.path is not None and revision==self.path['revision']
            if same and (self.plan is not None or self.active_request is not None):
                return dict(submitted=False, reason='same_path_retained', phase=self.phase)
            self._clear('new_live_path')
            self.path = dict(points=p.tolist(), revision=revision, stamp_ns=_stamp(msg))
            if self._future is not None:return dict(submitted=False, reason='old_worker_draining')
            self.sequence += 1
            request = dict(request_id=f'candidate-{self.sequence:06d}', goal_index=goal_index,
                map_revision=self.map['revision'], path_revision=revision,
                footprint_revision=self.footprint_revision, sensor_pose=self.sensor['pose'],
                created_monotonic_s=self.clock(), sensor_received_monotonic_s=self.sensor['received'],
                source_sensor_sim_stamp_ns=self.sensor['sim_stamp_ns'], path_stamp_ns=_stamp(msg),
                frame_id='map', origin=self.map['origin'], resolution=self.map['resolution'],
                position_margin_m=self.position_margin, yaw_error_rad=self.yaw_error,
                path=p.tolist(), footprint=self.footprint.tolist(),
                map_metadata={k:v for k,v in self.map.items() if k not in ('raw','occupied')})
            if nav_context is not None:request['nav_callback_context'] = dict(nav_context)
            directory = self.directory/request['request_id']
            # Callback owns immutable array references. Compression and all
            # disk serialization occur in the IPC thread, not this ROS callback.
            arrays = (self.map['raw'],self.map['occupied'])
            self.active_request = request
            self._future = self._executor.submit(self._run_worker,directory,request,arrays)
            self.reason = 'candidate_pending'
            return dict(submitted=True, request_id=request['request_id'])
        except (ValueError, TypeError, OverflowError, AttributeError) as error:
            self._clear('invalid_path:'+str(error),clear_path=True)
            return dict(submitted=False, reason=self.reason)

    def _run_worker(self, directory, request, arrays):
        try:
            directory.mkdir(exist_ok=False)
            _atomic(directory/'request.json',request)
            np.savez_compressed(directory/'slice.npz',raw_data=arrays[0],occupied=arrays[1])
            with (directory/'worker.log').open('w') as log:
                result = subprocess.run([self.worker_python,str(Path(__file__).with_name('live_oriented_worker.py')),str(directory)],
                    stdout=log,stderr=subprocess.STDOUT,timeout=self.worker_timeout,check=False)
            if result.returncode!=0:raise RuntimeError('worker_exit_'+str(result.returncode))
            return json.loads((directory/'result.json').read_text())
        except Exception as error:
            return {'error':type(error).__name__+':'+str(error)}

    def poll(self):
        if self._future is None or not self._future.done():return None
        result = self._future.result()
        self._future = None
        current = self.active_request
        reason = self._context_reason()
        if result.get('error'):reason = result['error']
        if not reason:
            check = accept_oriented_plan(result.get('plan'), result.get('request'), current,
                self.sensor['pose'], now_s=self.clock(), sensor_stamp_s=self.sensor['received'],
                max_sensor_age_s=self.max_sensor_age, max_plan_age_s=self.max_plan_age)
            reason = None if check['accepted'] else check['reason']
        if reason:
            self._clear('candidate_rejected:'+reason)
            verdict = dict(accepted=False, reason=self.reason, motion_authorized=False)
        else:
            self.plan, self.plan_local = result['plan'],result['plan_local']
            self.phase, self.reason = 'await_stop','candidate_accepted_for_gates'
            verdict = dict(accepted=True, reason=self.reason, motion_authorized=False,
                path=self.plan['path'], heading_reference=self.plan['yaw_rad'])
        self.audit.append({k:v for k,v in verdict.items() if k!='path'})
        _atomic(self.directory/'acceptance_audit.json', self.audit)
        return verdict

    def should_replan(self):
        return self.plan is None and self._future is None and self._pending_path is None and not self.path_request_in_flight()

    def gate(self):
        reason = self._context_reason()
        if reason or self.plan is None:
            self.reason = reason or self.reason
            return dict(phase=self.phase,allow_translation=False,desired_yaw_rate=0.,
                        replan=self.should_replan(),reason=self.reason,force_stop=True)
        # Only the initial accept uses request pose/time; tracking uses current
        # sensors and the live same-generation map, not the initial pose forever.
        local = np.asarray(self.sensor['pose']).copy()
        local[:2] -= np.asarray(self.map['origin'][:2])
        verdict = heading_gate(self.plan_local,local,self.sensor['twist'],self.footprint,
            self.map['occupied'],self.map['resolution'],phase=self.phase,
            observation_valid=True,transport_settled=self.sensor['transport_settled'])
        self.phase = verdict['phase']
        self.reason = verdict['reason']
        if verdict['replan']:self._clear(self.reason)
        verdict['force_stop'] = not verdict['allow_translation'] and verdict['desired_yaw_rate']==0.
        return verdict

    def reference_inputs(self):
        if self.plan is None:raise ValueError('No accepted live candidate')
        pose = np.asarray(self.sensor['pose']).copy()
        pose[:2] -= np.asarray(self.map['origin'][:2])
        return dict(path=self.plan_local['path'],xy=pose[:2],yaw=pose[2],
            occupied=self.map['occupied'],resolution=self.map['resolution'],
            footprint=self.plan['robust_footprint'],heading_reference=self.plan['yaw_rad'])

    def final_clear(self, command, horizon=.6):
        if self._context_reason() or self.plan is None:return False
        inputs = self.reference_inputs()
        clear = bool(motion_clear(inputs['xy'],inputs['yaw'],command,inputs['footprint'],
                                  inputs['occupied'],inputs['resolution'],horizon=horizon))
        return clear and self._context_reason() is None

    def hold_safe(self):
        if self._context_reason() or not self.sensor['transport_settled']:return False
        pose=np.asarray(self.sensor['pose']).copy();pose[:2]-=np.asarray(self.map['origin'][:2])
        foot=uncertainty_footprint(self.footprint,self.position_margin,self.yaw_error)
        clear = bool(motion_clear(pose[:2],pose[2],[0,0,0],foot,
                    self.map['occupied'],self.map['resolution'],horizon=0.))
        return clear and self._context_reason() is None

    def close(self):
        self._clear('closed',clear_path=True)
        self._invalidate_nav('closed')
        # subprocess.run has a bounded timeout and reaps its owned child.
        self._executor.shutdown(wait=True,cancel_futures=True)
