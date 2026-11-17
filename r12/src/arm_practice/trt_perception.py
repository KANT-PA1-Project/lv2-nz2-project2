"""TensorRT backend retaining the custom crop CenterPose decoder and YOLO NMS."""
import json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from ultralytics.data.augment import LetterBox
from ultralytics.utils import ops
from ultralytics.utils.nms import non_max_suppression

def detector_input(image):
 padded=LetterBox((800,800),auto=False)(image=image)
 return np.ascontiguousarray(padded[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.

class Engine:
 def __init__(self,path):
  # Ubuntu package installed for the matching system Python ABI.
  if '/usr/lib/python3.12/dist-packages' not in sys.path:sys.path.append('/usr/lib/python3.12/dist-packages')
  import tensorrt as trt
  self.trt=trt;self.logger=trt.Logger(trt.Logger.WARNING);self.runtime=trt.Runtime(self.logger)
  self.engine=self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
  if self.engine is None:raise RuntimeError('Cannot load TensorRT engine '+str(path))
  self.context=self.engine.create_execution_context();self.stream=torch.cuda.Stream()
  self.inputs=[];self.outputs=[];self.buffers={}
  for i in range(self.engine.num_io_tensors):
   name=self.engine.get_tensor_name(i);shape=tuple(self.engine.get_tensor_shape(name))
   if any(v<1 for v in shape):raise ValueError('Only fixed-shape validated engines supported')
   dtype=torch.from_numpy(np.empty((),dtype=trt.nptype(self.engine.get_tensor_dtype(name)))).dtype
   self.buffers[name]=torch.empty(shape,device='cuda',dtype=dtype)
   (self.inputs if self.engine.get_tensor_mode(name)==trt.TensorIOMode.INPUT else self.outputs).append(name)
   self.context.set_tensor_address(name,self.buffers[name].data_ptr())
  if len(self.inputs)!=1:raise ValueError('Expected one image input')
 def __call__(self,x):
  target=self.buffers[self.inputs[0]]
  if tuple(x.shape)!=tuple(target.shape):raise ValueError(f'Input shape {tuple(x.shape)} != {tuple(target.shape)}')
  with torch.cuda.stream(self.stream):
   target.copy_(torch.as_tensor(x,device='cpu'),non_blocking=False)
   if not self.context.execute_async_v3(self.stream.cuda_stream):raise RuntimeError('TensorRT execution failed')
  self.stream.synchronize()
  return {k:self.buffers[k].float().cpu() for k in self.outputs}

class PoseEngine:
 def __init__(self,path,metric_depth_head):
  self.runner=Engine(path);self.metric_depth_head=metric_depth_head
 def __call__(self,x):return [self.runner(x)]

class DetectorEngine:
 def __init__(self,path,names):self.runner=Engine(path);self.names={int(k):v for k,v in names.items()}
 def predict(self,image,imgsz=800,conf=.25,device=None,verbose=False):
  if imgsz!=800:raise ValueError('Engine requires imgsz800')
  output=next(iter(self.runner(detector_input(image)).values()))
  pred=non_max_suppression(output,conf_thres=conf,iou_thres=.7,nc=len(self.names),max_det=300)[0]
  if len(pred):pred[:,:4]=ops.scale_boxes((800,800),pred[:,:4],image.shape[:2])
  return [SimpleNamespace(names=self.names,boxes=[SimpleNamespace(xyxy=row[None,:4],conf=row[4:5],cls=row[5:6]) for row in pred])]
