"""Live CAD self-sweep and measured RGB-D surface guard for simulation trials.

This checks the observed surfaces, not free space in occluded/unobserved areas.
It never reads fixture poses, segmentation labels, or simulator base truth.
Unknown-volume clearance and real-hardware safety remain unverified.
"""
import time
import numpy as np
import fcl
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.kinematic_displacement import collider_joint_radii
from arm_practice.isaac_pixel_geometry import intrinsics,require_archive_geometry


class WristSurfaceGuard:
    def __init__(self):
        self.model = CollisionModel()
        fixed = {'gripper_left_joint': .02768613, 'gripper_right_joint': -.02768613}
        self.radii = np.array([collider_joint_radii(self.model.joints, body, local,
            self.model.sizes[name], [f'arm_joint{i+1}' for i in range(5)], fixed)
            for name, body, local, _ in self.model.boxes])
        self.half = np.array([self.model.sizes[name]/2 for name, _, _, _ in self.model.boxes])
        self.pairs = np.array([(i,j) for i, (_,body,_,_) in enumerate(self.model.boxes)
            for j in range(i+1,len(self.model.boxes)) if body != self.model.boxes[j][1]
            and frozenset((body,self.model.boxes[j][1])) not in self.model.skip])
        self.points = None
        self.capture_ns = None
        self.audit = {'reason': 'await_live_depth', 'unknown_volume_clearance_verified': False}
        self.camera_names = ('r07_camera1','r07_camera2','r07_camera3','arm_camera')
        fx,fy,cx,cy=intrinsics()
        v,u=np.indices((480,640));self.rays=np.stack(((u-cx)/fx,(v-cy)/fy,np.ones_like(u)),axis=-1)

    def update(self, data, packet):
        self.points=None; self.capture_ns=None
        self.audit={'reason':'invalid_depth_provenance','unknown_volume_clearance_verified':False,
                    'scope':'Nominal CAD self-sweep and current measured surfaces only; occluded space and hardware are not certified.'}
        try:
            require_archive_geometry(data)
            if (data['run_id'].item()!=packet['capture_run_id'] or
                int(data['stamp_ns'])!=packet['stamp_ns'] or
                int(data['capture_monotonic_ns'])!=packet['frame_capture_monotonic_ns'] or
                not 0<=time.monotonic_ns()-packet['frame_capture_monotonic_ns']<=500_000_000 or
                abs(float(packet['gripper_aperture_m'])-.075)>.001):return
            points=[];counts={}
            for name in self.camera_names:
                depth=np.asarray(data['depth' if name=='r07_camera1' else name+'_depth'])
                T=np.asarray(packet['camera_transforms'][name+'_left_optical'],float)
                if depth.shape!=(480,640) or T.shape!=(4,4) or not np.isfinite(T).all():return
                valid=np.isfinite(depth)&(depth>.03)&(depth<3.)
                p=(self.rays[valid]*depth[valid,None])@T[:3,:3].T+T[:3,3]
                # Bound the query to the fixed-base arm workspace. Every valid
                # pixel in this region contributes; no stride drops surfaces.
                p=p[(np.linalg.norm(p[:,:2],axis=1)<1.1)&(p[:,2]>-.1)&(p[:,2]<1.3)]
                counts[name]=len(p);points.append(p)
            p=np.concatenate(points)
            if len(p)<100:return
            # A voxel center represents all its measured points. Half diagonal
            # is added to the clearance test, never treated as empty space.
            cells=np.floor(p/.01).astype(np.int64)
            lower=cells.min(axis=0);shape=cells.max(axis=0)-lower+1
            packed=np.ravel_multi_index((cells-lower).T,tuple(shape))
            unique=np.column_stack(np.unravel_index(np.unique(packed),tuple(shape)))+lower
            self.points=(unique+.5)*.01
            self.capture_ns=packet['frame_capture_monotonic_ns']
            self.audit.update(reason='depth_ready',surface_voxels=len(self.points),
                source_pixels=counts,stamp_ns=packet['stamp_ns'],run_id=packet['capture_run_id'])
        except (KeyError,TypeError,ValueError):return

    def __call__(self,q,target,direction):
        if self.points is None or not 0<=time.monotonic_ns()-self.capture_ns<=500_000_000:
            self.audit['reason']='missing_or_stale_depth';return False
        q=np.asarray(q,float);target=np.asarray(target,float)
        if q.shape!=(5,) or target.shape!=(5,) or not np.isfinite(np.r_[q,target]).all():return False
        transforms=self.model.transforms(q)
        M=np.array([transforms[body]@local for _,body,local,_ in self.model.boxes])
        motion=self.radii@np.abs(target-q)
        centers=M[:,:3,3];extents=np.einsum('nij,nj->ni',np.abs(M[:,:3,:3]),self.half)
        left,right=self.pairs.T
        budget=motion[left]+motion[right]
        margin=np.linalg.norm(np.maximum(np.abs(centers[left]-centers[right])-extents[left]-extents[right],0),axis=1)-budget
        for i,(_,_,_,obj) in enumerate(self.model.boxes):
            obj.setTransform(fcl.Transform(M[i,:3,:3],M[i,:3,3]))
        for index in np.flatnonzero(margin<=1e-7):
            i,j=self.pairs[index]
            distance=fcl.distance(self.model.boxes[i][3],self.model.boxes[j][3],fcl.DistanceRequest(),fcl.DistanceResult())
            margin[index]=max(0.,distance)-budget[index]
            if not np.isfinite(distance) or margin[index]<=1e-7:
                self.audit.update(reason='self_sweep_unconfirmed',pair=[int(i),int(j)],margin_m=float(margin[index]));return False
        surface_margin=np.inf
        for i,(_,body,_,_) in enumerate(self.model.boxes):
            if body=='base_link':continue
            local=(self.points-centers[i])@M[i,:3,:3]
            distance=np.linalg.norm(np.maximum(np.abs(local)-self.half[i],0),axis=1).min()
            # CAD3cm + voxel support +5mm provisional measurement allowance.
            remaining=distance-(.03+np.sqrt(3)*.005+.005+motion[i])
            surface_margin=min(surface_margin,float(remaining))
            if remaining<0:
                self.audit.update(reason='observed_surface_near_sweep',body=body,margin_m=float(remaining));return False
        if time.monotonic_ns()-self.capture_ns>500_000_000:
            self.audit['reason']='depth_expired_during_check';return False
        self.audit.update(reason='observed_surface_and_nominal_self_sweep_clear',direction=direction,
            observed_surface_margin_m=surface_margin,self_sweep_margin_m=float(margin.min()))
        return True
