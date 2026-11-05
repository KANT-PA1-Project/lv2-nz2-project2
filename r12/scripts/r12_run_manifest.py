"""Capture source/config/checkpoint hashes before a simulation run."""
from pathlib import Path
import hashlib,json,os,sys,subprocess
root=Path(__file__).resolve().parents[1]
paths=[]
for pattern in ['src/arm_practice/*.py','scripts/r12_sensor_*.py','configs/sensors/*','models/swerve/policy.pt','models/swerve/rl/policy.pt','configs/r12_navigation_contract.json','configs/r12_arm_lesson.json','scripts/mission_scene_objects.py','scripts/r12_perception_worker.py','models/navigation/mission_scene.usda','models/navigation/scene_fixture.json','models/navigation/textures/*.png','models/swerve/deployed/latest.json']:
    paths.extend(p for p in root.glob(pattern) if p.is_file())
route=root/os.getenv('R12_ROUTE_FILE','models/navigation/routes.json');paths.append(route)
data={'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'environment':{k:v for k,v in os.environ.items() if k.startswith('R12_')},'sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
(root/sys.argv[1]/'run_manifest.json').write_text(json.dumps(data,indent=2))
