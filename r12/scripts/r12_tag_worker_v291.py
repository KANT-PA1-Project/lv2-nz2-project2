"""Isolated diagnostic tag worker. No model updates, controller, or motion API."""
import sys,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
from arm_practice.perception_transport import frame_identity,require_response
cv2.setNumThreads(1)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
print(json.dumps(dict(ready=True,opencv_version=cv2.__version__,execution_authorized=False)),flush=True)
for line in sys.stdin:
 try:
  phase='request';req=json.loads(line);started=time.monotonic_ns()
  with np.load(req['frame'],allow_pickle=False) as f:
   identity=frame_identity(f,req['camera']);rgb=f['rgb'].copy()
   camera=np.asarray(f['rgb_camera'])
   if camera.shape!=() or camera.dtype.kind not in 'US' or camera.item()!=req['camera']:raise ValueError('RGB camera identity mismatch')
  require_response(req,dict(identity,inference_started_monotonic_ns=started,inference_completed_monotonic_ns=time.monotonic_ns()),req['max_age_ns'])
  if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.shape[0]>2160 or rgb.shape[1]>3840:raise ValueError('Invalid RGB image')
  if rgb.shape!=(1440,1920,3):raise ValueError('Expected native1920x1440 RGB')
  phase='decode'
  small=cv2.resize(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),(1280,960),interpolation=cv2.INTER_AREA)
  gray=cv2.LUT(cv2.cvtColor(small,cv2.COLOR_BGR2GRAY),np.round((np.arange(256)/255.)**2*255).astype(np.uint8))
  tags=detect_tags(gray,det)
  for tag in tags:tag['corners']=(np.array(tag['corners'])*1.5).tolist()
  phase='post_decode'
  res=dict(identity,tags=tags,corner_coordinate_system="native1920x1440",processing_recipe="INTER_AREA1280_gamma2",inference_started_monotonic_ns=started,inference_completed_monotonic_ns=time.monotonic_ns(),execution_authorized=False)
  require_response(req,res,req['max_age_ns']);print(json.dumps(res),flush=True)
 except Exception as e:
  print(json.dumps(dict(error=type(e).__name__,detail=str(e),phase=phase,execution_authorized=False)),flush=True)
