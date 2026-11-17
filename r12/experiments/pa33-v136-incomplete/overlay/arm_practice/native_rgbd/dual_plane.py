"""Two observed tagged depth planes; no learned rotation, truth, or admission."""
import numpy as np,cv2

# Diagnostic consistency gates, NOT certified physical uncertainty bounds.
PLANE_RMS_LIMIT_M=.001
CENTER_COINCIDENCE_LIMIT_M=.002
ORTHOGONALITY_LIMIT_DEG=5.
CORNER_LOCALIZATION_ASSUMPTION_PX=2.

def plane(corners,depth,K):
 c=np.asarray(corners,float);D=np.asarray(depth);K=np.asarray(K,float)
 if c.shape!=(4,2)or D.ndim!=2 or K.shape!=(3,3)or not np.isfinite(c).all()or not np.isfinite(K).all():raise ValueError('invalid_plane_input')
 if not cv2.isContourConvex(c.astype(np.float32))or abs(cv2.contourArea(c.astype(np.float32)))<25:raise ValueError('degenerate_tag')
 H=cv2.getPerspectiveTransform(np.array([[-1,-1],[1,-1],[1,1],[-1,1]],np.float32),c.astype(np.float32));center=H[:2,2]/H[2,2];inner=center+(c-center)*.7
 x0,y0=np.floor(inner.min(axis=0)).astype(int);x1,y1=np.ceil(inner.max(axis=0)).astype(int)
 if x0<0 or y0<0 or x1>=D.shape[1]or y1>=D.shape[0]:raise ValueError('tag_out_of_depth_bounds')
 uv=np.array([(x,y)for y in range(y0,y1+1,3)for x in range(x0,x1+1,3)if cv2.pointPolygonTest(inner.astype(np.float32),(float(x),float(y)),False)>=0],float)
 if len(uv)<20:raise ValueError('insufficient_depth_samples')
 z=D[uv[:,1].astype(int),uv[:,0].astype(int)];valid=np.isfinite(z)&(z>.02)&(z<10)
 if valid.mean()<.9:raise ValueError('depth_patch_missing')
 points=(np.c_[uv[valid],np.ones(valid.sum())]@np.linalg.inv(K).T)*z[valid,None];mean=points.mean(axis=0);_,_,V=np.linalg.svd(points-mean,full_matrices=False);normal=V[-1]
 if normal@mean<0:normal=-normal
 rms=float(np.sqrt(np.mean(((points-mean)@normal)**2)))
 if rms>PLANE_RMS_LIMIT_M:raise ValueError('nonplanar_tag_depth')
 def intersect(uv):
  ray=np.linalg.solve(K,np.r_[uv,1.]);den=normal@ray
  if den<=1e-5:raise ValueError('invalid_plane_ray')
  return ray*(normal@mean)/den
 surface=intersect(center);xyz=np.array([intersect(uv)for uv in c]);edges=np.linalg.norm(np.roll(xyz,-1,axis=0)-xyz,axis=1)
 # CAD marker black boundary16mm; two endpoints each assumed localized within2px.
 tolerance=2*np.sqrt(2)*CORNER_LOCALIZATION_ASSUMPTION_PX*max(xyz[:,2])/min(K[0,0],K[1,1])
 if max(abs(edges-.016))>tolerance:raise ValueError('tag_metric_size_depth_inconsistent')
 return dict(surface=surface,normal=normal,rms=rms,center_px=center,edges=edges,size_tolerance_m=tolerance)

def from_planes(p1,p2):
 n1=np.asarray(p1['normal'],float);n2=np.asarray(p2['normal'],float);c1=np.asarray(p1['surface'],float);c2=np.asarray(p2['surface'],float)
 if any(x.shape!=(3,)or not np.isfinite(x).all()for x in [n1,n2,c1,c2])or any(abs(np.linalg.norm(x)-1)>1e-5 for x in [n1,n2]):raise ValueError('invalid_plane_geometry')
 dot=float(n1@n2)
 if abs(dot)>np.sin(np.radians(ORTHOGONALITY_LIMIT_DEG)):raise ValueError('parallel_or_nonorthogonal_faces')
 candidates=[]
 for label,d1,d2 in [('short_short',.01503,.01503),('short_long',.01503,.03003),('long_short',.03003,.01503)]:
  a=c1+n1*d1;b=c2+n2*d2;candidates.append(dict(face_classes=label,residual_m=float(np.linalg.norm(a-b)),center=(a+b)/2))
 eligible=[x for x in candidates if x['residual_m']<=CENTER_COINCIDENCE_LIMIT_M]
 if len(eligible)!=1:raise ValueError('ambiguous_or_inconsistent_face_thickness')
 chosen=eligible[0];kind=chosen['face_classes']
 z=np.cross(n1,n2)if kind=='short_short'else n2 if kind=='short_long'else n1;z=z/np.linalg.norm(z);x=n2 if kind=='long_short'else n1;x=x-z*(x@z);x=x/np.linalg.norm(x);y=np.cross(z,x);R=np.column_stack([x,y,z])
 return dict(center_camera_m=chosen['center'].tolist(),rotation_matrix=R.tolist(),face_classes=kind,center_coincidence_m=chosen['residual_m'],hypotheses=[dict(face_classes=x['face_classes'],residual_m=x['residual_m'])for x in candidates],normal_dot=dot,orientation_semantics='square prism representative, modulo 8 proper symmetries',pose_error_budget=None,control_admitted=False,motion_authorized=False)

def estimate(tags,depth,K):
 if len(tags)!=2:raise ValueError('exactly_two_visible_tags_required')
 if any(type(t['id'])is not int or not 0<=t['id']<12 for t in tags)or tags[0]['id']!=tags[1]['id']:raise ValueError('tag_identity_mismatch')
 a=plane(tags[0]['corners'],depth,K);b=plane(tags[1]['corners'],depth,K);r=from_planes(a,b)
 r.update(tag_id=tags[0]['id'],plane_RMS_m=[a['rms'],b['rms']],metric_tag_edges_m=[a['edges'].tolist(),b['edges'].tolist()],tag_corner_localization_assumption_px=CORNER_LOCALIZATION_ASSUMPTION_PX,diagnostic_gate_not_error_budget=True)
 return r
