"""Registered RGB-D to points using calibrated simulation intrinsics, no scene truth."""
import numpy as np
from arm_practice.isaac_pixel_geometry import intrinsics

def project(rgb,depth,T,step=6,maximum=5.):
    rgb=np.asarray(rgb);depth=np.asarray(depth);T=np.asarray(T)
    if rgb.shape[:2]!=depth.shape or rgb.shape[2]!=3 or T.shape!=(4,4):raise ValueError('RGB/depth/transform mismatch')
    v,u=np.mgrid[0:depth.shape[0]:step,0:depth.shape[1]:step];z=depth[::step,::step]
    valid=np.isfinite(z)&(z>.05)&(z<maximum)
    fx,fy,cx,cy=intrinsics()
    points=np.stack([(u-cx)*z/fx,(v-cy)*z/fy,z],axis=-1)[valid]
    xyz=(points@T[:3,:3].T+T[:3,3]).astype('<f4');colors=rgb[::step,::step][valid].astype(np.uint32)
    packed=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
    return xyz,packed

def pack_rgb(xyz,rgb):
    data=np.empty(len(xyz),dtype=[('xyz','<f4',(3,)),('rgb','<u4')]);data['xyz']=xyz;data['rgb']=rgb;return data.tobytes()
