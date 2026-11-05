"""Measurement accounting is separate from capture or mission success."""
def summarize(rows, requested_epochs, *, aborted=False):
    if type(requested_epochs) is not int or requested_epochs < 0:
        raise ValueError('invalid_requested_epochs')
    if type(aborted) is not bool:
        raise ValueError('invalid_aborted_flag')
    complete = not aborted and len(rows) == requested_epochs and all(type(r.get('frame')) is int and r['frame'] == i for i,r in enumerate(rows))
    captured = sum(r.get('captured') is True for r in rows)
    capture_pass = complete and requested_epochs > 0 and captured == requested_epochs
    return dict(requested_control_epochs=requested_epochs,measured_control_epochs=len(rows),current_native_epochs=captured,measurement_complete=complete,completed_measurement=complete,capture_pass=capture_pass,mission_success=False,performance_accepted=False,decoder_started=False,recognized_ids=0,observation_admitted=False,motion_authorized=False)
