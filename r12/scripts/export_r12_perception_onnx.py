"""Export frozen task models without overwriting training artifacts."""
import argparse,json,sys,hashlib,shutil
from pathlib import Path
import torch,onnx
SOURCE=Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam')
sys.path.insert(0,str(SOURCE/'perception/scripts'))
from train_centerpose import load_model
from ultralytics import YOLO
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
run=SOURCE/'runs/perception/train-v11-domain';torch.set_num_threads(2)
shutil.copy2(run/'yolo/weights/best.pt',a.out/'yolo.pt')
yolo=YOLO(a.out/'yolo.pt');yolo.export(format='onnx',imgsz=800,opset=17,simplify=False,dynamic=False,device='cpu')
model=load_model(run/'centerpose/best.pt',device='cpu')
class ExportPose(torch.nn.Module):
 def __init__(self,m):super().__init__();self.model=m
 def forward(self,x):
  d=self.model(x)[0];return tuple(d[k] for k in ('hm','hps','reg','scale'))
with torch.inference_mode():
 torch.onnx.export(ExportPose(model).eval(),torch.zeros(3,3,128,128),a.out/'centerpose.onnx',input_names=['images'],output_names=['hm','hps','reg','scale'],opset_version=17,dynamo=False)
for name in ('yolo','centerpose'):onnx.checker.check_model(onnx.load(a.out/(name+'.onnx')))
manifest={'sources':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (run/'yolo/weights/best.pt',run/'centerpose/best.pt',run/'inference.json')},'names':yolo.names,'pose_metric_depth_head':model.metric_depth_head,'pose_views':'scale3_median','pose_batch':3,'imgsz':800,'performance_accepted':False}
(a.out/'manifest.json').write_text(json.dumps(manifest,indent=2));print('EXPORT_COMPLETE',flush=True)
