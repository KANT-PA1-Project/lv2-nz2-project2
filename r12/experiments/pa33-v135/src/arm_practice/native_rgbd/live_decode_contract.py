"""Same-host authenticated producer diagnostic, never motion admission."""
import hashlib

def validate(message,blobs,*,run_id,last_step,now_ns):
    b=message['certificate']['body'];v=message['live_issuer_verification']
    if message.get('kind')!='capture' or message['run_id']!=run_id or b['run_id']!=run_id:raise ValueError('run_mismatch')
    if v.get('rgbd_capture_association_verified') is not True or v.get('run_id')!=run_id or v.get('capture_physics_step')!=b['capture_physics_step'] or v.get('encoder_history')!=b['encoder_history']:raise ValueError('issuer_verification_binding')
    step=b['capture_physics_step'];capture=b['capture_monotonic_lower_bound_ns']
    if type(step)is not int or step<=last_step:raise ValueError('duplicate_or_reversed_epoch')
    if type(capture)is not int or not capture<=b['issued_ns']<=now_ns or now_ns-capture>500_000_000:raise ValueError('stale_capture_at_decode_input')
    if len(message['arrays'])!=len(blobs):raise ValueError('array_count')
    for d,blob in zip(message['arrays'],blobs):
        target=b['pixel_sha256'if d['kind']=='rgb'else'depth_sha256'][d['name']]
        if hashlib.sha256(blob).hexdigest()!=target:raise ValueError('raw_hash_mismatch')
    return b

def fresh_at_completion(capture_ns,now_ns):
    return type(capture_ns)is int and type(now_ns)is int and 0<=now_ns-capture_ns<=500_000_000
