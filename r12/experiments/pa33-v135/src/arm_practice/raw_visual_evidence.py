"""Isolated ROS-message snapshots; caller-supplied session labels are not authentication."""
from copy import deepcopy


def _identity(message, run_id, clock_epoch, received_ns, topic):
    sec, nanosec = message.header.stamp.sec, message.header.stamp.nanosec
    if type(sec) is not int or type(nanosec) is not int or sec < 0 or not 0 <= nanosec < 10**9:
        raise ValueError('invalid_source_stamp')
    if any(type(x) is not str or not x for x in (run_id, clock_epoch)):
        raise ValueError('invalid_session')
    if type(received_ns) is not int or received_ns < 0:
        raise ValueError('invalid_receipt_time')
    return dict(run_id=run_id, clock_epoch=clock_epoch,
                clock_domain='synthetic_packet_20hz', stamp_ns=sec*10**9+nanosec,
                received_monotonic_ns=received_ns, source_topic=topic)


def snapshot_raw_odometry(message, *, run_id, clock_epoch, received_ns):
    record = _identity(message, run_id, clock_epoch, received_ns,
                       '/visual_slam/tracking/odometry')
    pose = message.pose.pose
    record.update(frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                  position=[pose.position.x, pose.position.y, pose.position.z],
                  quaternion_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                  covariance=list(message.pose.covariance), unmodified=True)
    return deepcopy(record)


def snapshot_raw_status(message, *, run_id, clock_epoch, received_ns):
    record = _identity(message, run_id, clock_epoch, received_ns, '/visual_slam/status')
    if type(message.vo_state) is not int:
        raise ValueError('invalid_tracking_state')
    return dict(record, vo_state=message.vo_state)


def pair_raw_visual_evidence(odometry, status):
    """Exact-stamp pairing; never relabel coordinate frames or floor covariance."""
    keys = ('run_id', 'clock_epoch', 'clock_domain', 'stamp_ns')
    if any(odometry[k] != status[k] for k in keys):
        raise ValueError('visual_identity_mismatch')
    if odometry['source_topic'] != '/visual_slam/tracking/odometry' or status['source_topic'] != '/visual_slam/status':
        raise ValueError('visual_source_mismatch')
    record = deepcopy(odometry)
    # State 1 matches the existing local ROS consumer convention; not a pose certificate.
    record['tracking_valid'] = type(status['vo_state']) is int and status['vo_state'] == 1
    record['raw_status'] = deepcopy(status)
    return dict(raw_odometry=record, source_authenticity_verified=False,
                pose_join_verified=False, execution_authorized=False)
