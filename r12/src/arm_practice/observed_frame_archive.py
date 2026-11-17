"""Optional immutable observation history; archives are not motion certificates."""
import json,hashlib,os
from pathlib import Path
import numpy as np
from .frame_packet_identity import check_frame_packet_identity

def archive_observed_frame(frame_path,destination,packet,index):
    frame_path=Path(frame_path);destination=Path(destination)
    with np.load(frame_path,allow_pickle=False) as f:
        identity=check_frame_packet_identity(f,packet,index)
        if not identity['matched']:raise ValueError(identity['reason'])
        run=str(f['run_id'].item())
        if run!=packet.get('capture_run_id'):raise ValueError('run_mismatch')
    destination.mkdir(parents=True,exist_ok=True);stem=f'{index:06d}'
    target=destination/(stem+'.npz');meta=destination/(stem+'.json')
    if target.exists() or meta.exists():raise FileExistsError(stem)
    blob=frame_path.read_bytes();tmp=target.with_suffix('.tmp');tmp.write_bytes(blob);os.replace(tmp,target)
    record=dict(frame_index=index,run_id=run,stamp_ns=packet['stamp_ns'],sha256=hashlib.sha256(blob).hexdigest(),frame_file=target.name,arm_measured_position=packet['arm_measured_position'],arm_measured_velocity=packet.get('arm_measured_velocity'),arm_feedback_stamp_ns=packet.get('arm_feedback_stamp_ns'),capture_monotonic_ns=packet.get('frame_capture_monotonic_ns'),visual_pose=packet.get('visual_pose'),visual_pose_provenance=packet.get('visual_pose_provenance'),base_sensor_feedback=packet.get('base_sensor_feedback'),scope='Sensor observations only; visual pose may be unavailable. No ground truth, map fusion or clearance authorization.')
    meta.write_text(json.dumps(record,indent=2)+'\n');return record
