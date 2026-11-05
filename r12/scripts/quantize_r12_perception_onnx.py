"""INT8 Q/DQ calibration uses training split only; validation stays separate."""
import argparse,json,sys,random,hashlib
from pathlib import Path
import cv2,numpy as np
import onnx
from onnxruntime.quantization import quantize_static,CalibrationDataReader,QuantFormat,QuantType,CalibrationMethod
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
sys.path.insert(0,'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts')
from train_centerpose import crop_image
from arm_practice.trt_perception import detector_input
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
data=Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/runs/perception/data-v11-domain');manifest=data/'manifest.json';items=[s for s in json.loads(manifest.read_text())['samples'] if s['split']=='train'];random.Random(401).shuffle(items);items=items[:128]
class Reader(CalibrationDataReader):
 def __init__(self,kind):self.kind=kind;self.items=iter(items);self.used=[]
 def get_next(self):
  for sample in self.items:
   im=cv2.imread(str(data/sample['image']))
   if self.kind=='yolo':x=detector_input(im)
   else:
    boxes=[b for b in sample['boxes'] if b['class_id']<4]
    if not boxes:continue
    box=boxes[0]['xyxy'];x=np.stack([crop_image(im,box,crop_scale=s)[0].numpy() for s in (1.5,1.35,1.65)])
   self.used.append(sample['image']);return {'images':x}
  return None
for kind in ('centerpose','yolo'):
 reader=Reader(kind)
 quantize_static(str(a.out/(kind+'.onnx')),str(a.out/(kind+'_int8.onnx')),reader,quant_format=QuantFormat.QDQ,activation_type=QuantType.QInt8,weight_type=QuantType.QInt8,per_channel=True,calibrate_method=CalibrationMethod.MinMax,op_types_to_quantize=['Conv'],extra_options={'ActivationSymmetric':True,'WeightSymmetric':True,'QuantizeBias':False})
 onnx.checker.check_model(onnx.load(a.out/(kind+'_int8.onnx')))
 (a.out/(kind+'_calibration.json')).write_text(json.dumps({'split':'train','images':reader.used,'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),'method':'symmetric INT8 QDQ Conv per-channel weights; other ops retain float','performance_accepted':False},indent=2));print(kind,'QUANTIZED',len(reader.used),flush=True)
