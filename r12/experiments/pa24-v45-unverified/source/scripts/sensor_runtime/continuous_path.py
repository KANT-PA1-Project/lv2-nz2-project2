"""Arc-length holonomic references with separate translation and chassis heading.

Only planned paths and sensor poses enter this module. The caller retains live
lidar, swept-footprint, freshness and actuator limits. This is an experimental
reference generator, not a collision or hardware performance certificate.
"""
import numpy as np


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


class ContinuousPath:
    def __init__(self, speed=.6, lateral_acceleration=.12, deceleration=.12,
                 yaw_speed=.2, arrival_radius=.10):
        self.speed = float(speed)
        self.lateral_acceleration = float(lateral_acceleration)
        self.deceleration = float(deceleration)
        self.yaw_speed = float(yaw_speed)
        self.arrival_radius = float(arrival_radius)
        if not np.isfinite([speed, lateral_acceleration, deceleration, yaw_speed, arrival_radius]).all() or min(speed, lateral_acceleration, deceleration, yaw_speed, arrival_radius) <= 0:
            raise ValueError('Positive finite trajectory limits required')
        self.path = None
        self.progress = 0.
        self.heading_offset = None
        self.last_heading = None
        self.generation = 0
        self.goal = None
        self.final_heading = None

    def update_path(self, path, xy):
        path = np.asarray(path, float)
        if path.ndim != 2 or path.shape[1] != 2 or not np.isfinite(path).all():
            raise ValueError('Finite Nx2 path required')
        if self.path is not None and np.array_equal(self.path, path):
            return
        self.path = path.copy()
        if len(path) and (self.goal is None or np.linalg.norm(path[-1]-self.goal)>.001):
            self.goal = path[-1].copy()
            self.heading_offset = self.final_heading = None
        self.points = path.copy()
        if len(path) < 2:
            self.arc = np.zeros(len(path)); self.progress = 0.
            return
        keep = np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-6]
        self.points = path[keep]
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.progress = 0.
        self.progress = self.project(xy)[0]
        self.generation += 1

    def point(self, s):
        return np.array([np.interp(s, self.arc, self.points[:, i]) for i in range(2)])

    def project(self, xy):
        if len(self.points) < 2:
            return 0., float(np.linalg.norm(np.asarray(xy)-self.points[0]))
        a, b = self.points[:-1], self.points[1:]
        v = b-a
        fraction = np.clip(np.sum((xy-a)*v, axis=1)/np.sum(v*v, axis=1), 0., 1.)
        projected = a + fraction[:, None]*v
        s = self.arc[:-1] + fraction*np.diff(self.arc)
        distance = np.linalg.norm(projected-xy, axis=1)
        # Avoid jumping backward across an adjacent leg of a hairpin.
        eligible = s >= self.progress-.08
        i = int(np.argmin(np.where(eligible, distance, np.inf)))
        return float(s[i]), float(distance[i])

    def tangent(self, s):
        a, b = max(0., s-.10), min(self.arc[-1], s+.10)
        d = self.point(b)-self.point(a)
        return float(np.arctan2(d[1], d[0]))

    def tangents(self, s):
        """Evaluate the same arc-length queries in a batch; no path/pose cache.

        The scalar tangent method remains the current-heading reference. This
        batches the four repeated np.interp calls used by the braking preview,
        preserving the query positions, endpoints and 0.10m difference window.
        """
        s = np.asarray(s, float)
        a, b = np.maximum(0., s-.10), np.minimum(self.arc[-1], s+.10)
        dx = np.interp(b, self.arc, self.points[:, 0]) - np.interp(a, self.arc, self.points[:, 0])
        dy = np.interp(b, self.arc, self.points[:, 1]) - np.interp(a, self.arc, self.points[:, 1])
        return np.arctan2(dy, dx)

    def propose(self, path, xy, yaw, measured_speed=0., *, heading_reference=None):
        xy = np.asarray(xy, float)
        if xy.shape != (2,) or not np.isfinite(np.r_[xy,yaw,measured_speed]).all():
            raise ValueError('Finite sensor pose/speed required')
        if heading_reference is not None and not np.isfinite(heading_reference):
            raise ValueError('Finite selected chassis heading required')
        self.update_path(path, xy)
        if len(self.arc) < 2 or self.arc[-1] < 1e-6:
            return np.zeros(3), {'reason':'empty_path'}
        s, cross_track = self.project(xy)
        self.progress = max(self.progress, s)
        remaining = max(0., self.arc[-1]-self.progress)
        lookahead = float(np.clip(.22 + .35*abs(measured_speed), .22, .40))
        target = self.point(self.progress+lookahead)
        direction = target-xy
        norm = float(np.linalg.norm(direction))
        tangent = self.tangent(self.progress+.10)
        if self.heading_offset is None:
            # Interior steering for both perpendicular module mount directions.
            candidates = np.pi/4 + np.arange(-4,5)*np.pi/2
            self.heading_offset = float(candidates[np.argmin(abs(wrap(tangent+candidates-yaw)))])
        heading = yaw+wrap(tangent+self.heading_offset-yaw)
        if heading_reference is not None:
            heading = yaw+wrap(float(heading_reference)-yaw)
        heading_error = float(wrap(heading-yaw))
        preview_s = np.arange(self.progress, min(self.arc[-1], self.progress+1.8)+.025, .05)
        preview_heading = np.unwrap(self.tangents(preview_s))
        curvature = np.gradient(preview_heading, .05) if len(preview_s)>1 else np.zeros(1)
        curve_limits = np.minimum(np.sqrt(self.lateral_acceleration/np.maximum(abs(curvature),1e-6)),
                                  .80*self.yaw_speed/np.maximum(abs(curvature),1e-6))
        # Back-propagate braking before bends, not when already in the corner.
        cap = min(self.speed, float(np.min(np.sqrt(curve_limits**2 + 2*self.deceleration*(preview_s-self.progress)))))
        goal_distance = float(np.linalg.norm(self.points[-1]-xy))
        # A coarse approach does not need a final tangent-heading correction.
        # Retain the heading reached on entry, across replans of this same goal.
        if heading_reference is None and self.final_heading is None and remaining<.30 and goal_distance<.30:
            self.final_heading = float(yaw)
        if heading_reference is None and self.final_heading is not None:
            heading = yaw+wrap(self.final_heading-yaw)
            heading_error = float(wrap(heading-yaw))
        cap = min(cap, np.sqrt(2*self.deceleration*max(0.,remaining-self.arrival_radius*.65)),
                  1.5*max(0.,goal_distance-self.arrival_radius*.65))
        # Let heading catch up before crossing a mechanical steering branch.
        cap *= float(np.clip((.65-abs(heading_error))/.35, .12, 1.))
        kappa = float(curvature[0])
        if self.final_heading is not None or heading_reference is not None:kappa=0.
        turn = float(np.clip(kappa*cap+.65*heading_error, -self.yaw_speed, self.yaw_speed))
        velocity = direction*(cap/max(norm,1e-9))
        c, sn = np.cos(yaw), np.sin(yaw)
        command = np.r_[np.array([[c,sn],[-sn,c]])@velocity, turn]
        self.last_heading = heading
        return command, {'reason':'continuous_reference','generation':self.generation,
                         'progress_m':self.progress,'remaining_m':remaining,'cross_track_m':cross_track,
                         'target_xy':target.tolist(),'heading_goal_rad':heading,'heading_error_rad':heading_error,
                         'selected_heading_active':heading_reference is not None,
                         'lookahead_m':lookahead,'curvature_per_m':kappa,'speed_cap_m_s':cap,
                         'goal_distance_m':goal_distance,'arrival_radius_m':self.arrival_radius}


def integrate_pose(xy, yaw, twist, dt):
    """Predicted SE2 pose from a body twist; no simulator state is accessed."""
    vx, vy, w = twist
    a = w*dt
    if abs(w) < 1e-8:
        local = np.array([vx,vy])*dt
    else:
        local = np.array([np.sin(a)*vx-(1-np.cos(a))*vy,
                          (1-np.cos(a))*vx+np.sin(a)*vy])/w
    c,s = np.cos(yaw),np.sin(yaw)
    return np.asarray(xy)+np.array([[c,-s],[s,c]])@local, yaw+a


def safe_reference(controller, path, xy, yaw, measured_speed, envelope,
                   gyro, accel, visual_tilt, scans, extrinsics, footprint, occupied, resolution, *, heading_reference=None):
    """Preview the *filtered* path motion before committing an envelope update.

    Live lidar and the final veto remain independent. A rollout is only a
    kinematic screen: it cannot prove tracking/clearance under plant error.
    """
    from copy import deepcopy
    from arm_practice.lidar_guard import constrain_velocity
    from swept_footprint import motion_clear
    target, state = controller.propose(path,xy,yaw,measured_speed,heading_reference=heading_reference)
    failures = []
    for scale in (1., .75, .5, .25, 0.):
        preview_controller, preview_envelope = deepcopy(controller), deepcopy(envelope)
        pos, heading = np.asarray(xy).copy(), float(yaw)
        first = None
        for k in range(16):
            desired, _ = preview_controller.propose(path,pos,heading,np.linalg.norm(preview_envelope.velocity[:2]),heading_reference=heading_reference)
            desired[:2] *= scale
            if k==0:
                # Current lidar is valid at this pose only; do not pretend the
                # old scan is a fresh future observation in the rollout.
                desired, guard = constrain_velocity(desired,scans,extrinsics,footprint=footprint)
                first = desired.copy()
                if guard == 'invalid_sensor':
                    return np.zeros(3), dict(state, preview_reason=guard)
            command, _ = preview_envelope.step(desired,gyro,accel,.05 if k==0 else .1,valid=True,visual_tilt=visual_tilt)
            horizon = .05 if k==0 else .1
            if not motion_clear(pos,heading,command,footprint,occupied,resolution,horizon=horizon):
                failures.append({'scale':scale,'step':k})
                break
            pos,heading = integrate_pose(pos,heading,command,horizon)
        else:
            return first, dict(state, preview_scale=scale, preview_failures=failures,preview_reason='filtered_rollout_clear',reference_lidar_guard=guard,guarded_reference=first.tolist())
    return np.zeros(3), dict(state,preview_scale=0.,preview_failures=failures,preview_reason='no_filtered_rollout_clear')
