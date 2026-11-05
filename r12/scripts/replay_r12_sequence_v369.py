"""Stationary recorded sensor replay; original capture times remain historical."""
import hashlib,json,os,socket,sys,time
from pathlib import Path
import numpy as np
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'))
from arm_practice.shared_frame_prototype import SharedFrameSlot
from arm_practice.capture_bundle_identity import inspect_capture_bundle
out=Path(os.environ['R12_SENSOR_RUN']);source=repo.parents[1]/'outputs/r12/v368_stationary'
manifest=json.loads((source/'sensor_sequence/manifest.json').read_text());loaded=[]
assert len(manifest['frames'])==20
for i,item in enumerate(manifest['frames']):
 path=source/'sensor_sequence'/item['file'];assert path.parent==source/'sensor_sequence'
 assert hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256']
 with np.load(path,allow_pickle=False) as archive:arrays={k:archive[k] for k in archive.files}
 identity=inspect_capture_bundle(arrays,item['packet']);assert identity['matched'] and identity['frame_index']==i
 loaded.append((item['packet'],arrays))
(out/'sequence_validation.json').write_text(json.dumps(dict(frames=20,all_hashes_and_identities_matched=True,original_capture_times_preserved=True)))
for name in ['extrinsics.json','lidar_extrinsics.json']:(out/name).write_bytes((source/name).read_bytes())
records=[]
with SharedFrameSlot(directory=os.environ['R12_SHARED_FRAME_DIRECTORY']) as slot,socket.create_connection(('127.0.0.1',int(os.environ['R12_SENSOR_PORT'])),timeout=10) as conn:
 conn.settimeout(10);reader=conn.makefile('r')
 for packet,arrays in loaded:
  start=time.monotonic_ns();packet=dict(packet);packet['shared_frame_descriptor']=slot.publish(arrays)
  sent=time.monotonic_ns();conn.sendall((json.dumps(packet)+'\n').encode());response=json.loads(reader.readline())
  received=time.monotonic_ns();slot.acknowledge(response['shared_frame_ack'])
  assert response['command']==[0.,0.,0.] and response['arm_target'] is None
  records.append(dict(stamp_ns=packet['stamp_ns'],replay_start_ns=start,packet_send_ns=sent,response_ns=received,original_capture_ns=packet['frame_capture_monotonic_ns']))
  time.sleep(max(0,.05-(time.monotonic_ns()-start)/1e9))
 conn.shutdown(socket.SHUT_WR)
(out/'replay_timing.json').write_text(json.dumps(dict(scope='stationary temporal replay, not fresh camera or hardware',frames=records),indent=2))
print('R12_SENSOR_SIM_COMPLETE',flush=True)
(out/'completion.json').write_text(json.dumps(dict(frames=20,replay=True,mission_finished=False)))
