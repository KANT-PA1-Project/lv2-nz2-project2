"""Live checkpoint kinematic preview, not physics training or executable trajectories."""
import time,json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'));bank=root.parent.parent/'outputs/r12/learned_ik_topdown_v176';out=root/'reports/local/arm_training_view_v177';out.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':800})
import torch,omni.usd,omni.ui as ui
from pxr import UsdGeom,UsdLux,Gf
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.learned_ik import ArmFK
stage=omni.usd.get_context().get_stage();add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12');UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
# Timeline remains stopped; USD poses are visualization only.
import omni.timeline
omni.timeline.get_timeline_interface().stop()
model=CollisionModel();cfg=json.loads((root/'configs/r12_robot.json').read_text());fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);ops={}
for prim in stage.Traverse():
 if prim.GetName() in model.bodies and str(prim.GetPath()).startswith('/World/R12/'):
  xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();ops[prim.GetName()]=xf.AddTransformOp()
def marker(path,color):
 s=UsdGeom.Sphere.Define(stage,path);s.CreateRadiusAttr(.008);s.CreateDisplayColorAttr([Gf.Vec3f(*color)]);return s.AddTranslateOp()
target_marker=marker('/World/Target',(0,1,0));tcp_marker=marker('/World/Prediction',(1,.2,.1))
set_camera_view(eye=np.array([.9,-1.1,.85]),target=np.array([.1,0,.30]))
window=ui.Window('R12 live IK learning | KINEMATIC PREVIEW',width=610,height=160)
with window.frame:
 with ui.VStack():
  ui.Label('LIVE CHECKPOINT VIEW - NOT PHYSICS TRAINING / NOT A GRASP')
  ui.Label('Green: target | Red: network TCP | animation is NOT a validated path')
  label=ui.Label('Loading training dataset...')
  details=ui.Label('')
q=np.zeros(5);goal=q.copy();last=0.;counter=0;start=time.monotonic();torch.set_num_threads(1)
while app.is_running() and time.monotonic()-start<1800 and not (out/'STOP').exists():
 now=time.monotonic()
 if now-last>4 and (bank/'dataset.npz').exists():
  last=now
  try:
   with np.load(bank/'dataset.npz') as data:
    ids=data['validation'];idx=int(ids[counter%len(ids)]);target=data['rz'][idx].copy()
   method='flow' if (bank/'flow.pt').exists() else 'mlp';path=bank/(method+'.pt');d=torch.load(path,weights_only=False,map_location='cpu');net=torch.nn.Sequential(torch.nn.Linear(6 if method=='flow' else 2,128),torch.nn.SiLU(),torch.nn.Linear(128,128),torch.nn.SiLU(),torch.nn.Linear(128,3));net.load_state_dict(d['weights']);net.eval();cond=torch.tensor((target-d['mean'])/d['std'],dtype=torch.float32).reshape(1,2)
   with torch.no_grad():
    if method=='mlp':v=net(cond)
    else:
     v=torch.randn((1,3),generator=torch.Generator().manual_seed(177+idx))
     for k in range(16):v=v+net(torch.cat([v,cond,torch.full((1,1),k/16)],1))/16
   pitch=np.clip(v.numpy()[0]*np.pi,d['lower'],d['upper']);goal=np.r_[0,pitch,0];target3=np.array([target[0],0,target[1]]);target_marker.Set(Gf.Vec3d(*target3));mm=float(np.linalg.norm(fk(goal)-target3)*1000);angle=float(abs(sum(pitch)+np.pi/2)*180/np.pi)
   completed=(bank/'evaluation.json').exists();label.text=f'{method.upper()} checkpoint step {d["step"]} | '+('TRAINING COMPLETE: checkpoint replay' if completed else 'TRAINING RUNNING: latest best checkpoint');details.text=f'Target {idx} | network endpoint {mm:.2f} mm | down-axis {angle:.2f} deg'
   (out/'status.json').write_text(json.dumps(dict(method=method,step=d['step'],training_complete=completed,target_index=idx,proposal_error_mm=mm,proposal_angle_deg=angle,scope='Network-only kinematic visualization, timeline stopped, no physics control; interpolation not a validated motion path'),indent=2)+'\n');print('PREVIEW',method,d['step'],idx,mm,flush=True);counter+=1
  except Exception as e:label.text='Waiting for complete checkpoint';print('CHECKPOINT_WAIT',str(e),flush=True)
 q+=np.clip(goal-q,-.025,.025)
 for body,T in model.transforms(q).items():
  if body in ops:ops[body].Set(Gf.Matrix4d(T.T.tolist()))
 tcp_marker.Set(Gf.Vec3d(*fk(q)));app.update();time.sleep(.016)
app.close()
