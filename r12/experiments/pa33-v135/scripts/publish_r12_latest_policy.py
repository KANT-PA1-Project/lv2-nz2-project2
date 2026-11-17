from pathlib import Path
import sys,json
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.latest_policy import publish_completed
report=root/'reports/local/rl_highspeed_v2'
if 'PPO_COMPLETE' not in (report/'train.log').read_text():raise RuntimeError('Training not complete')
iteration=json.loads((report/'training.json').read_text())[-1]['iteration']
print(publish_completed(root/'models/swerve/rl_highspeed_v2/policy.pt',root/'models/swerve/deployed',iteration))
