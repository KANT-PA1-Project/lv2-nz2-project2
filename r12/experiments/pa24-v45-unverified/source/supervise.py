from pathlib import Path
import subprocess,os,json,time,traceback
ROOT=Path(__file__).resolve().parent;OUT=Path(os.environ['FLEET_RUN_DIR']);child=None
def save(n,d):
 p=OUT/n;t=p.with_suffix('.tmp');t.write_text(json.dumps(d,indent=2));t.replace(p)
try:
 with(OUT/'render.log').open('w')as log:
  child=subprocess.Popen(['/home/pa24/pa-course/.venv-isaacsim/bin/python',str(ROOT/'render_axis.py')],stdout=log,stderr=subprocess.STDOUT)
  save('owned_pids.json',dict(supervisor=os.getpid(),renderer=child.pid));begin=time.monotonic();minimum=1e9
  while child.poll()is None:
   f,u=map(float,subprocess.check_output(['nvidia-smi','--query-gpu=memory.free,utilization.gpu','--format=csv,noheader,nounits'],text=True).splitlines()[0].split(','));minimum=min(minimum,f)
   with(OUT/'resources.jsonl').open('a')as out:out.write(json.dumps(dict(updated_at=time.time(),free_mib=f,gpu_util_percent=u))+'\n')
   if f<1024:raise RuntimeError('VRAM reserve below1024 MiB')
   if time.monotonic()-begin>360:raise RuntimeError('renderer deadline360s')
   time.sleep(1)
  if child.returncode!=0:raise RuntimeError('renderer exit '+str(child.returncode))
  r=json.loads((OUT/'render_result.json').read_text());assert len(r['rows'])==3
  save('supervisor_result.json',dict(renderer_exit_code=child.returncode,frames=3,min_vram_free_mib=minimum,calibration_accepted=False,motion_authorized=False))
except BaseException as e:
 save('supervisor_error.json',dict(error=str(e),traceback=traceback.format_exc()));save('job_status.json',dict(run_id=os.environ['FLEET_RUN_ID'],host='pa24',updated_at=time.time(),status='failed',phase='native_axis_render_failure',progress=dict(current=0,total=6,unit='native stationary RGBD and analytic-plane frames'),error=str(e),motion_authorized=False));raise
finally:
 if child and child.poll()is None:
  child.terminate()
  try:child.wait(20)
  except subprocess.TimeoutExpired:child.kill();child.wait()
