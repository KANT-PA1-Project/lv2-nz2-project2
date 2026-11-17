from pathlib import Path
import subprocess,os,json,time,traceback,sys
ROOT=Path(__file__).resolve().parent;OUT=Path(os.environ['FLEET_RUN_DIR']);child=None
def save(n,d):
 p=OUT/n;t=p.with_suffix('.tmp');t.write_text(json.dumps(d,indent=2));t.replace(p)
def disk_usage_bytes(root):
 total=0
 for path in root.rglob('*'):
  try:
   if path.is_file() and not path.is_symlink():total+=path.stat().st_size
  except FileNotFoundError:
   # Producer atomically renames temporary telemetry while we enumerate it.
   continue
 return total
try:
 with(OUT/'render.log').open('w')as log:
  child=subprocess.Popen([os.environ.get('PA24_ISAAC_PYTHON',sys.executable),str(ROOT/'render_axis.py')],stdout=log,stderr=subprocess.STDOUT)
  save('owned_pids.json',dict(supervisor=os.getpid(),renderer=child.pid));begin=time.monotonic();minimum=1e9
  while child.poll()is None:
   f,u=map(float,subprocess.check_output(['nvidia-smi','--query-gpu=memory.free,utilization.gpu','--format=csv,noheader,nounits'],text=True).splitlines()[0].split(','));minimum=min(minimum,f)
   with(OUT/'resources.jsonl').open('a')as out:out.write(json.dumps(dict(updated_at=time.time(),free_mib=f,gpu_util_percent=u))+'\n')
   if f<1024:raise RuntimeError('VRAM reserve below1024 MiB')
   if time.monotonic()-begin>360:raise RuntimeError('renderer deadline360s')
   disk_bytes=disk_usage_bytes(OUT)
   if disk_bytes>500*1024*1024:raise RuntimeError('disk budget500MiB')
   time.sleep(1)
  if child.returncode!=0:raise RuntimeError('renderer exit '+str(child.returncode))
  if (OUT/'render_error.json').exists():raise RuntimeError('renderer_declared_failure:'+json.loads((OUT/'render_error.json').read_text())['traceback'].splitlines()[-1])
  r=json.loads((OUT/'render_result.json').read_text())
  config=json.loads((ROOT/'run_manifest.json').read_text());expected_frames=int(config.get('frames_per_phase',3))
  if len(r['rows'])!=expected_frames:raise RuntimeError('capture_count_mismatch')
  if not all(x.get('captured') is True and x.get('verification',{}).get('matched') is True for x in r['rows']):raise RuntimeError('capture_rejected')
  proposals=[x['live_view_proposal'] for x in r['rows']]
  config=json.loads((ROOT/'run_manifest.json').read_text());challenge=config.get('identity_reobserve_challenge');motion=config.get('live_motion_diagnostic')
  if challenge:
   first,second,third=proposals
   conditions=[first.get('accepted') is False,first.get('reason')=='multiple_visible_markers',first.get('identity_confirmed') is False,first.get('reobserve_required') is True,len(first.get('identity_evidence',{}).get('tags',[]))==2,second.get('accepted') is True,second.get('identity_confirmed') is False,second.get('identity_streak')==1,third.get('accepted') is True,third.get('identity_confirmed') is True,third.get('identity_streak')==2,third.get('reobserve_required') is False,second.get('observation',{}).get('id')==third.get('observation',{}).get('id')==0,len({x['native_step'] for x in r['rows']})==3]
   if not all(conditions):raise RuntimeError('identity_reobservation_sequence_failed')
  elif not all(x.get('accepted') is True for x in proposals):raise RuntimeError('live_proposal_rejected')
  if any(x.get('motion_authorized') is not False for x in proposals):raise RuntimeError('unexpected_mission_motion_authority')
  if motion:
   moves=json.loads((OUT/'diagnostic_motion_results.json').read_text());geometry=json.loads((OUT/'live_geometry_admission.json').read_text())
   if len(moves)!=1 or r.get('movement_dispatch_count')!=1:raise RuntimeError('expected_one_diagnostic_move')
   m=moves[0]
   conditions=[m.get('diagnostic_move_executed') is True,m.get('settled_native_and_FD') is True,m.get('settled_continuous_s')==.5,m.get('contact_count')==0,m.get('fixture_goal_used') is False,m.get('native_step_tracking_region_verified') is True,m.get('continuous_joint_region_geometry_proven') is True,geometry.get('environment_bounds_proven') is True,geometry.get('native_CAD_alignment_verified') is True,geometry.get('live_collision_inventory_verified') is True,geometry.get('robot_collider_count')==447,r['rows'][2]['native_step']>m['end_step'],proposals[1].get('identity_confirmed') is True,proposals[2].get('identity_confirmed') is True]
   if not all(conditions):raise RuntimeError('diagnostic_motion_evidence_failed')
  elif r.get('movement_dispatch_count')!=0 or any(x.get('movement_veto') not in ('continuous_swept_clearance_not_verified','live_observation_rejected') for x in proposals):raise RuntimeError('unexpected_motion_dispatch')
  save('supervisor_result.json',dict(renderer_exit_code=child.returncode,frames=expected_frames,live_proposals=sum(x.get('accepted') is True for x in proposals),expected_ambiguous_rejections=int(bool(challenge)),identity_reobservation_verified=bool(challenge),diagnostic_motion_verified=bool(motion),movement_dispatch_count=int(bool(motion)),min_vram_free_mib=minimum,calibration_accepted=False,motion_authorized=False,scope='fresh_observe_safe_region_move_settle_reobserve' if motion else ('fresh_identity_ambiguity_rejection_and_reobservation' if challenge else 'fresh_in_process_proposal_only_no_movement')))
except BaseException as e:
 save('supervisor_error.json',dict(error=str(e),traceback=traceback.format_exc()));save('job_status.json',dict(run_id=os.environ['FLEET_RUN_ID'],host='pa24',updated_at=time.time(),status='failed',phase='native_axis_render_failure',progress=dict(current=0,total=6,unit='native stationary RGBD and analytic-plane frames'),error=str(e),motion_authorized=False));raise
finally:
 if child and child.poll()is None:
  child.terminate()
  try:child.wait(20)
  except subprocess.TimeoutExpired:child.kill();child.wait()
