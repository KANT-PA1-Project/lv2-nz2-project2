"""Queue one GUI mission while unrelated GPU training is running."""
import os,time,json,subprocess,fcntl
from pathlib import Path
root=Path(__file__).resolve().parents[1];os.chdir(root)
report=root/'reports/local/latest_ppo_gui';report.mkdir(parents=True,exist_ok=True)
lock=(report/'queue.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
def status(state,**extra):
 p=report/'queue_status.json';t=p.with_suffix('.tmp');t.write_text(json.dumps({'state':state,'pid':os.getpid(),'time':time.time(),**extra},indent=2));t.replace(p)
while True:
 if (report/'CANCEL').exists():status('cancelled');raise SystemExit()
 free=int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True).splitlines()[0])
 processes=subprocess.check_output(['ps','-eo','comm,args'],text=True)
 other_sim=any(line.split()[0].startswith('python') and any(x in line for x in ['rl/train_swerve','scripts/evaluate_r12','scripts/r12_sensor_sim.py']) for line in processes.splitlines() if line.split())
 if free>=4500 and not other_sim:break
 status('waiting_for_gpu',free_memory_mib=free,required_memory_mib=4500);time.sleep(15)
env=dict(os.environ,R12_GUI='1',R12_SENSOR_RUN='latest_ppo_gui',R12_SENSOR_PORT='19617',R12_ROS_DOMAIN_ID='99',R12_USE_PPO='0',R12_USE_LATEST_PPO='1',R12_MISSION='1',R12_CRUISE_SPEED='.899795667',R12_ROUTE_FILE='models/navigation/survey_route.json',R12_SENSOR_FRAMES='40000')
status('starting_gui')
with (report/'runner.log').open('w') as log:result=subprocess.run(['bash','scripts/run_r12_mission.sh'],env=env,stdout=log,stderr=subprocess.STDOUT)
status('finished' if result.returncode==0 else 'failed',returncode=result.returncode)
