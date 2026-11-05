"""Own-process-only bounded diagnostic launcher. Never starts robot bridge/physics."""
import subprocess,time,os,json,signal
from pathlib import Path
import rclpy
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';children=[];logs=[];report={}
rclpy.init();node=rclpy.create_node('r12_domain_preflight',enable_rosout=False)
try:
 for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
 others=[n for n,ns in node.get_node_names_and_namespaces() if n!='r12_domain_preflight']
 if others:raise RuntimeError('Domain occupied:'+str(others))
finally:node.destroy_node();rclpy.shutdown()
def spawn(cmd,name):
 log=(out/name).open('x');logs.append(log);p=subprocess.Popen(cmd,cwd=repo,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
try:
 slam=spawn(['ros2','launch','scripts/cuvslam_debug_v329.launch.py'],'v329_cuvslam.log');time.sleep(8)
 if slam.poll() is not None:raise RuntimeError('cuVSLAM exited early')
 params=subprocess.run(['ros2','param','dump','/r12_visual_slam'],capture_output=True,text=True,timeout=10)
 (out/'v329_live_params.yaml').write_text(params.stdout);(out/'v329_param_stderr.log').write_text(params.stderr);report['param_dump_exit']=params.returncode
 if params.returncode:raise RuntimeError('Parameter dump failed')
 rec=spawn([str(repo/'.venv-r12/bin/python'),'scripts/record_r12_shadow_v329.py','--output',str(out/'v329_raw'),'--seconds','10'],'v329_recorder.log')
 pub=spawn([str(repo/'.venv-r12/bin/python'),'scripts/replay_r12_static_v329.py'],'v329_publisher.log');report['publisher_exit']=pub.wait(timeout=25);report['recorder_exit']=rec.wait(timeout=30)
finally:
 for p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=4)
 for log in logs:log.close()
 report.update(xml_profile=os.getenv('FASTDDS_DEFAULT_PROFILES_FILE'),builtin_transports=os.getenv('FASTDDS_BUILTIN_TRANSPORTS'),domain=os.getenv('ROS_DOMAIN_ID'),owned_children_reaped=all(p.poll() is not None for p in children),runtime_changed=False);(out/'v329_execution.json').write_text(json.dumps(report,indent=2)+'\n')
