import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/sensor_runtime'))
from command_send_freshness import encode_base_response


def response():
    return dict(command=[.6, 0, 0], wheel_action=[1]*4,
                command_stages={'final':[.6,0,0]}, arm_target=[.1]*5,
                stationary_wheel_hold_torque=[.2]*4, stop_brake_valid=True)


def test_expiration_during_serialization_revokes_all_base_outputs():
    now=[.14]
    def encode(value):
        now[0]+=.02
        return json.dumps(value).encode()
    result=response()
    wire,reason=encode_base_response(result, lambda: 'sensor_stale' if now[0]>.15 else None, encode=encode)
    sent=json.loads(wire)
    assert reason=='sensor_stale'
    assert sent==result
    assert sent['command']==[0]*3 and sent['wheel_action']==[0]*4
    assert sent['stationary_wheel_hold_torque']==[0]*4
    assert not sent['stop_brake_valid'] and not sent['stationary_wheel_hold_enabled']
    assert sent['command_stages']['final']==[0]*3
    assert sent['arm_target']==[.1]*5  # This base-only guard does not certify arm safety.


def test_fresh_wire_unchanged():
    result=response()
    original=json.loads(json.dumps(result))
    wire,reason=encode_base_response(result, lambda:None)
    assert reason is None and json.loads(wire)==original


def test_map_invalidation_also_revokes():
    result=response()
    wire,reason=encode_base_response(result, lambda:'map_stale')
    assert reason=='map_stale' and json.loads(wire)['command']==[0]*3
