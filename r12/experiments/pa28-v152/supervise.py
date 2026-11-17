from pathlib import Path
import os,subprocess,time,json,sys,hashlib,signal
r=Path(__file__).resolve().parent;o=Path(os.environ['FLEET_RUN_DIR']);mode=sys.argv[1]
assert mode in ('smoke','lift_only')
manifest=json.loads((r/'source_manifest.json').read_text())
def mismatches():return [n for n,h in manifest.items()if hashlib.sha256((r/n).read_bytes()).hexdigest()!=h]
assert not mismatches(),mismatches()
def save(name,value):
 p=o/name;t=p.with_suffix('.tmp');t.write_text(json.dumps(value,indent=2));t.replace(p)
save('source_manifest.json',manifest);save('experiment_manifest.json',json.loads((r/'experiment_manifest.json').read_text()))
python='/home/pa28/pa-course/robot_learning/IsaacLab-3.0-EA/.venv/bin/python'
e=os.environ.copy();e.pop('DISPLAY',None);e.pop('WAYLAND_DISPLAY',None);e.update(OMNI_KIT_ACCEPT_EULA='YES',PYTHONDONTWRITEBYTECODE='1',PYTHONUNBUFFERED='1')
error=None;minimum=1e9;begin=time.monotonic()
with(o/'native.log').open('w')as log:
 child=subprocess.Popen([python,str(r/'bench.py'),mode],cwd=r,env=e,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);save('owned_pids.json',dict(supervisor=os.getpid(),worker=child.pid,started_at=time.time()))
 while child.poll()is None:
  free=float(subprocess.check_output(['nvidia-smi','--id=0','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True).strip());minimum=min(minimum,free)
  size=sum(p.stat().st_size for p in o.rglob('*')if p.is_file())
  if free<1024:error='VRAM_reserve'
  elif time.monotonic()-begin>900:error='wall_budget'
  elif size>250*1024**2:error='output_budget'
  if error:
   os.killpg(child.pid,signal.SIGINT)
   try:child.wait(20)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(20)
   break
  time.sleep(1)
 code=child.wait()
 if '[Error]'in(o/'native.log').read_text(errors='replace'):error=error or 'native_error_log'
 if (o/'failure.json').exists():error=error or 'native_failure_record'
 if not(o/'after_close.json').exists():error=error or 'missing_clean_cleanup'
 if not(o/'result.json').exists():error=error or 'missing_bounded_completion'
 if mismatches():error=error or 'source_changed'
 save('supervisor_result.json',dict(exit_code=code,error=error,min_free_mib=minimum,source_unchanged=not mismatches(),performance_accepted=False))
 raise SystemExit(code or int(bool(error)))
