"""Offline phase-accounting proposal. Never supplies live sensor freshness."""
import hashlib,json

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def plan(warmup_epochs,measurement_epochs):
    for value in (warmup_epochs,measurement_epochs):
        if type(value) is not int or value < 0:raise ValueError('invalid_epoch_budget')
    if measurement_epochs == 0:raise ValueError('measurement_budget_required')
    return dict(warmup_epochs=warmup_epochs,measurement_epochs=measurement_epochs,max_renders=4,ready_wall_ms=200,phase_policy='fixed_prefix_no_retry_until_success',sensor_clock_policy='original_only',performance_accepted=False)

def replay(raw,phase_plan,expected_plan_sha256):
    if digest(phase_plan)!=expected_plan_sha256:raise ValueError('phase_plan_changed')
    if phase_plan != plan(phase_plan['warmup_epochs'],phase_plan['measurement_epochs']):raise ValueError('unsupported_phase_plan')
    rows=json.loads(raw)
    if type(rows) is not list:raise ValueError('rows_required')
    budget=phase_plan['warmup_epochs']+phase_plan['measurement_epochs']
    if len(rows)>budget:raise ValueError('over_budget')
    audit=[]
    for i,row in enumerate(rows):
        if type(row.get('frame')) is not int or row['frame']!=i:raise ValueError('noncontiguous_original_frames')
        if type(row.get('captured')) is not bool:raise ValueError('invalid_capture_status')
        attempts=row.get('attempts')
        if type(attempts) is not int or not 1<=attempts<=4:raise ValueError('render_budget_violation')
        age=row.get('ready_age_ms')
        if row['captured'] and (type(age) not in (int,float) or not 0<=age<=200):raise ValueError('captured_outside_ready_budget')
        audit.append(dict(original_frame=i,phase='warmup' if i<phase_plan['warmup_epochs'] else 'measurement',original_row_sha256=digest(row),original_record=row))
    warm=[x for x in audit if x['phase']=='warmup'];measured=[x for x in audit if x['phase']=='measurement']
    complete=len(rows)==budget
    return dict(source_raw_sha256=hashlib.sha256(raw).hexdigest(),plan_sha256=expected_plan_sha256,provenance='retrospective_archive_replay_not_precommitted_live_run',records=audit,warmup_observed=len(warm),warmup_failures=[x['original_frame']for x in warm if not x['original_record']['captured']],measurement_observed=len(measured),measurement_complete=complete,measurement_capture_pass=complete and all(x['original_record']['captured']for x in measured),all_epoch_capture_pass=complete and all(x['original_record']['captured']for x in audit),cold_failures_preserved=True,live_freshness_authorized=False,performance_accepted=False,mission_success=False,motion_authorized=False)
