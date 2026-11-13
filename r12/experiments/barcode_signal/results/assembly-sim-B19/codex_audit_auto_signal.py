import json,sys,math,hashlib
from pathlib import Path
from collections import Counter

for arg in sys.argv[1:]:
    p=Path(arg)
    if not (p/'process_exit.json').exists():
        print(p.name,'not finished');continue
    result=json.loads((p/'result.json').read_text())
    process=json.loads((p/'process_exit.json').read_text())
    inputs=json.loads((p/'inputs.json').read_text())
    cfg=inputs['config']
    critical_names={'run.py','signal_fallback.py','tower_oblique.py','common.py','scene.py'}
    source_matches={str(Path(k).name): hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in inputs['source_hashes'].items() if Path(k).name in critical_names}
    rows=[json.loads(l) for l in (p/'frames.jsonl').open()]
    classified=[r for r in rows if r.get('color') is not None]
    route=result.get('signal_route',{})
    events=route.get('events',[])
    ready=next((e['t_s'] for e in events if e['event']=='ready'),None)
    first_barcode=next((r['t_s'] for r in rows if r['barcode'] is not None),None)
    fallback=[e for e in events if e['event']=='fallback_requested']
    post_barcode=[r for r in rows if first_barcode is not None and r['t_s']>=first_barcode]
    after_ready=[r for r in rows if ready is not None and r['t_s']>=ready]
    changes=[]
    last=None
    for r in rows:
        key=(r['phase'],r.get('signal_camera'))
        if key!=last:
            changes.append(dict(t_s=r['t_s'],phase=key[0],camera=key[1],barcode=r['barcode']))
            last=key
    physical_count=0;peak_native=peak_fd=0.;settle=[]
    for line in (p/'physics.jsonl').open():
        r=json.loads(line);physical_count+=1
        peak_native=max(peak_native,max(map(abs,r['native_velocity'])))
        peak_fd=max(peak_fd,max(map(abs,r['fd_velocity'])))
        if ready is not None and ready-.5<=r['t_s']<ready:
            settle.append(bool(r['settled']))
    checks=dict(result_passed=result['status']=='passed',process_exit_zero=process['returncode']==0,
        exact_sequence=result.get('observed_sequence')==cfg['sequence'],
        all_classified_frames_correct=bool(classified) and all(r['color']==r['truth_for_scoring_only'] for r in classified),
        barcode_preserved=bool(post_barcode) and all(r['barcode']==cfg['barcode']['code'] for r in post_barcode),
        no_camera_switch_after_READY=bool(after_ready) and len({r['signal_camera'] for r in after_ready})==1,
        fallback_only_before_READY=(not fallback or (ready is not None and len(fallback)==1 and fallback[0]['t_s']<ready)),
        no_partial_record_during_motion=all(not r['sequence'] for r in rows if r['phase'] in ('move', 'move_base', 'move_transport')),
        fresh_settle_before_READY=bool(settle) and all(settle),
        record_preserved_after_extra_lamp=bool(result.get('signal_saved') and result.get('later_lamp_ignored')))
    audit=dict(source_matches_current=source_matches,checks=checks,all_checks_passed=all(checks.values()),frame_count=len(rows),
        classified_frames=len(classified),correct_frames=sum(r['color']==r['truth_for_scoring_only'] for r in classified),
        phase_counts=dict(Counter(r['phase'] for r in rows)),phase_changes=changes,
        barcode_at_s=first_barcode,READY_at_s=ready,route=route,
        boundary_rejection_frames=sum(('frame edge' in str(r.get('reason')) or 'clipped' in str(r.get('reason'))) for r in rows),
        measured_native_peak_rad_s=peak_native,measured_fd_peak_rad_s=peak_fd,
        physics_samples=physical_count,settle_samples_before_READY=len(settle),
        geometry_assumptions=cfg.get('tower_scene'),
        limitations=['Kinematic chassis path; no wheel driving validation',
                     'Arm self collision disabled; 20 Nm simulation drive assumption retained',
                     'Rendered lighting/exposure not a real-camera calibration',
                     'Barcode forward=world +X and left=+Y are stated scene assumptions'])
    (p/'audit.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False)+'\n')
    print(p.name,json.dumps(dict(passed=audit['all_checks_passed'],checks=checks,frames=len(rows),
          classified=len(classified),barcode_s=first_barcode,ready_s=ready,route=route),ensure_ascii=False))
