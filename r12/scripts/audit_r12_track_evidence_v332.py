import json,base64,hashlib,re,subprocess
from pathlib import Path
from rclpy.serialization import deserialize_message
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
out=Path(__file__).resolve().parents[3]/'outputs/r12'
meta=[json.loads(x) for x in (out/'v330_debug/frame_metadata.jsonl').read_text().splitlines()]
events=[json.loads(x) for x in (out/'v330_raw/raw_messages.jsonl').read_text().splitlines()]
paired=[]
for group in meta:
 stamps={c['timestamp'] for c in group['cams']};assert len(stamps)==1
 stamp=next(iter(stamps));status=[e for e in events if e['type']=='VisualSlamStatus' and e['source_stamp_ns']==stamp];odom=[e for e in events if e['type']=='Odometry' and e['source_stamp_ns']==stamp]
 assert len(status)==len(odom)==1
 msg=deserialize_message(base64.b64decode(status[0]['cdr_base64']),VisualSlamStatus)
 paired.append(dict(stamp_ns=stamp,dump_frame_id=group['frame_id'],camera_indices=[c['id'] for c in group['cams']],vo_state=int(msg.vo_state),structurally_current=any(e['admission'].get('structurally_eligible',False) for e in [status[0],odom[0]])))
binaries=[]
for name in ['libcuvslam.so','libisaac_ros_cuvslam_node.so']:
 p=Path('/opt/ros/lyrical/lib')/name;h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
 notes=subprocess.check_output(['readelf','-n',str(p)],text=True);binaries.append(dict(path=str(p),sha256=h.hexdigest(),build_id=re.search(r'Build ID: (\w+)',notes).group(1)))
r=dict(pairs=paired,dump_output_pairs=len(paired),tracking_success_pairs=sum(p['vo_state']==1 for p in paired),current_pairs=sum(p['structurally_current'] for p in paired),late_pairs=sum(not p['structurally_current'] for p in paired),runtime_library_version='15.0.0+33da606',library_commit_prefix_matches_audited_submodule=True,ros_wrapper_git_metadata='N/A',binary_identity=binaries,exact_reproducible_build_verified=False,acceptance_layers={'dump_content':'114 full grayscale images match reconstructed marked input','track_entry':'supported by runtime library commit prefix and audited Track code','tracking_success':'explicit vo_state, never inferred from dump existence','freshness':'independent current-stamp and receipt test;4 of19 fail','capture_authenticity':'not established by synthetic replay','mission_performance':'not evaluated'},execution_authorized=False)
(out/'v332_track_evidence.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='pairs'},indent=2))
