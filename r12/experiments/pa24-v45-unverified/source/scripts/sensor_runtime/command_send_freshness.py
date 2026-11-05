"""Revoke base actuation if context expires while preparing the wire response.

This checks immediately before sendall; it does not certify socket transit or
arm/gripper freshness, which require their own receiving-side contracts.
"""
import json


def encode_base_response(result, context_reason, *, encode=None):
    if encode is None:
        encode = lambda value: (json.dumps(value) + '\n').encode()
    wire = encode(result)
    reason = context_reason()
    if reason is not None:
        result.update(command=[0., 0., 0.], wheel_action=[0.] * 4,
                      wheel_scale=1., stationary_wheel_hold_torque=[0.] * 4,
                      stationary_wheel_hold_enabled=False,
                      planner_stop_hold_allowed=False, stop_brake_valid=False,
                      stop_brake_gyro_z=None, lidar_guard='send_context_expired',
                      send_context_issue=reason)
        result['command_stages']['final'] = [0., 0., 0.]
        wire = encode(result)
    return wire, reason
