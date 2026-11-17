"""Explicit mixed-resolution profile. Legacy640/v1 limits remain unchanged."""
import copy,hashlib,json
from pathlib import Path
import numpy as np

def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def sha(x):return hashlib.sha256(canonical(x)).hexdigest()
WIRE_V1='r12-rgbd-wire-v1'
WIRE_V2='r12-rgbd-wire-diagnostic-all640-v1'
VERSION='r12-camera-diagnostic-all640-v1'

def pinned_specs():
 specs={}
 for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
  w,h=(640,480)
  for side in ['left','right']:
   name=camera+'_'+side+'_optical'
   specs[name]={'camera':camera,'camera_frame':name,'resolution_wh':[w,h],'rgb_dtype':'uint8','rgb_shape':[h,w,3],'intrinsics':[float(w*.625),float(h*.625/.75),(w-1)/2.,(h-1)/2.],'optical_axes':'x_right_y_down_z_forward','pixel_convention':'isaac_pixel_center_v1','focal_length_mm':20.955*.625,'horizontal_aperture_mm':20.955,'vertical_aperture_mm':20.955*.75,'clipping_range_m':[.01,30.],'distortion':'none','hardware_calibrated':False,'uncertainty_certified':False}
 return specs

def build_config(root):
 sources={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in ('configs/sensors/camera_rig.json','models/r12/r12_arm.urdf','scripts/sensor_runtime/camera_rig.py','configs/r12_tcp_calibration.json')}
 depth={c+'_depth':{'rgb_camera':c+'_left_optical','dtype':'float32','shape':pinned_specs()[c+'_left_optical']['rgb_shape'][:2],'unit':'m','quantity':'distance_to_image_plane'} for c in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']}
 payload={'schema':VERSION,'wire_schema':WIRE_V2,'cameras':pinned_specs(),'depth':depth,'extrinsic_source_sha256':sources}
 return dict(payload,calibration_sha256=sha(payload))

class MixedCameraSchema:
 def __init__(self,config):
  self.config=copy.deepcopy(config)
  if self.config.get('schema')!=VERSION or self.config.get('wire_schema')!=WIRE_V2:raise ValueError('unsupported_mixed_camera_schema')
  if canonical(self.config.get('cameras'))!=canonical(pinned_specs()):raise ValueError('unpinned_camera_shape_K_or_optics')
  sources=self.config.get('extrinsic_source_sha256',{})
  required={'configs/sensors/camera_rig.json','models/r12/r12_arm.urdf','scripts/sensor_runtime/camera_rig.py','configs/r12_tcp_calibration.json'}
  if set(sources)!=required or any(type(v) is not str or len(v)!=64 or any(c not in '0123456789abcdef' for c in v) for v in sources.values()):raise ValueError('invalid_extrinsic_hashes')
  expected_depth={c+'_depth':{'rgb_camera':c+'_left_optical','dtype':'float32','shape':pinned_specs()[c+'_left_optical']['rgb_shape'][:2],'unit':'m','quantity':'distance_to_image_plane'} for c in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']}
  if canonical(self.config.get('depth'))!=canonical(expected_depth):raise ValueError('unpinned_depth_schema')
  body={k:v for k,v in self.config.items() if k!='calibration_sha256'}
  if set(body)!={'schema','wire_schema','cameras','depth','extrinsic_source_sha256'} or self.config.get('calibration_sha256')!=sha(body):raise ValueError('calibration_hash_mismatch')
  self.sha256=sha(self.config);self.calibration_sha256=self.config['calibration_sha256'];self.wire_schema=WIRE_V2
 @classmethod
 def from_file(cls,path):
  path=Path(path);obj=cls(json.loads(path.read_text()));root=path.parent.parent
  for n,v in obj.config['extrinsic_source_sha256'].items():
   if not (root/n).is_file() or hashlib.sha256((root/n).read_bytes()).hexdigest()!=v:raise ValueError('actual_extrinsic_source_hash_mismatch:'+n)
  return obj
 def fields(self):return {'camera_schema':copy.deepcopy(self.config),'camera_schema_sha256':self.sha256,'calibration_sha256':self.calibration_sha256,'wire_schema':self.wire_schema}
 def verify_body(self,body):
  for k,v in self.fields().items():
   if canonical(body.get(k))!=canonical(v):raise ValueError('mixed_certificate_schema_mismatch:'+k)
 def descriptor(self,name,kind):
  if kind=='rgb':s=self.config['cameras'][name];dtype,shape=s['rgb_dtype'],s['rgb_shape']
  elif kind=='depth':s=self.config['depth'][name];dtype,shape=s['dtype'],s['shape']
  else:raise ValueError('invalid_array_kind')
  return {'kind':kind,'name':name,'dtype':dtype,'shape':list(shape)}
 def digest(self,array,name,kind):
  d=self.descriptor(name,kind);a=np.asarray(array)
  if a.dtype!=np.dtype(d['dtype']) or list(a.shape)!=d['shape']:raise ValueError('mixed_array_shape_or_dtype:'+name)
  return hashlib.sha256(a.tobytes(order='C')).hexdigest()
 def validate_wire(self,data,lengths):
  if data.get('kind')!='capture':
   if lengths:raise ValueError('unexpected_mixed_control_blobs')
   return
  self.verify_body(data['certificate']['body'])
  ds=data['arrays']
  expected=[self.descriptor(k,'rgb') for k in sorted(self.config['cameras'])]+[self.descriptor(k,'depth') for k in sorted(self.config['depth'])]
  if canonical(ds)!=canonical(expected) or len(lengths)!=len(expected):raise ValueError('mixed_wire_descriptors_mismatch')
  sizes=[int(np.prod(d['shape']))*np.dtype(d['dtype']).itemsize for d in expected]
  if lengths!=sizes or sum(lengths)!=12_288_000:raise ValueError('mixed_wire_lengths_mismatch')
 def archive_fields(self):return {'camera_schema_json':np.array(canonical(self.config).decode()),'camera_schema_sha256':np.array(self.sha256)}
 def verify_archive(self,frame):
  for k in ('camera_schema_json','camera_schema_sha256'):
   a=np.asarray(frame[k])
   if a.shape!=() or a.dtype.kind not in 'US':raise ValueError('invalid_mixed_archive_schema_scalar')
  if str(frame['camera_schema_sha256'])!=self.sha256 or canonical(json.loads(str(frame['camera_schema_json'])))!=canonical(self.config):raise ValueError('mixed_archive_camera_schema_mismatch')
 def check_actual_optics(self,readback):
  if set(readback)!=set(self.config['cameras']):raise ValueError('actual_camera_set_mismatch')
  for name,actual in readback.items():
   spec=self.config['cameras'][name]
   if actual['resolution_wh']!=spec['resolution_wh']:raise ValueError('actual_resolution_mismatch:'+name)
   for key in ('focal_length_mm','horizontal_aperture_mm','vertical_aperture_mm'):
    if not np.isfinite(actual[key]) or abs(actual[key]-spec[key])>1e-5:raise ValueError('actual_lens_mismatch:'+name)
   if not np.allclose(actual['clipping_range_m'],spec['clipping_range_m'],rtol=0,atol=1e-6):raise ValueError('actual_clip_range_mismatch:'+name)
   w,h=actual['resolution_wh'];fx=w*actual['focal_length_mm']/actual['horizontal_aperture_mm'];fy=h*actual['focal_length_mm']/actual['vertical_aperture_mm']
   if max(abs(fx-spec['intrinsics'][0]),abs(fy-spec['intrinsics'][1]))>1e-3:raise ValueError('actual_intrinsics_mismatch:'+name)
  return True
