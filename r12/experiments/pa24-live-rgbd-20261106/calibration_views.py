"""Sequential local stationary views; no trajectory or physical calibration."""
from pathlib import Path
import itertools
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
from pxr import Usd,UsdGeom
import launch_live_view as launch
from audit_swept_path import audit

BASE=Path(__file__).resolve().parent
SOURCE=BASE/'source'
POSES=[('train-a',-.1,0.),('train-b',.1,0.),('train-c',0.,-.15),
       ('train-d',0.,.15),('train-e',-.08,.12),('train-f',.08,-.12),
       ('holdout-a',.04,.08),('holdout-b',-.04,-.08)]


def prepare():
    manifest=json.loads((SOURCE/'run_manifest.json').read_text())
    stage=Usd.Stage.Open(str(SOURCE/'models/navigation/mission_scene.usda'))
    target=stage.GetPrimAtPath('/World/Pucks/Puck0');cache=UsdGeom.XformCache();points=[]
    for prim in Usd.PrimRange(target):
        if prim.IsA(UsdGeom.Cube):
            half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
            local=np.array(list(itertools.product([-half,half],repeat=3)))
        elif prim.IsA(UsdGeom.Mesh):local=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
        else:continue
        T=np.asarray(cache.ComputeRelativeTransform(prim,target)[0]).T
        points.extend((T@np.c_[local,np.ones(len(local))].T).T[:,:3])
    points=np.asarray(points);center=(points.max(axis=0)+points.min(axis=0))/2
    radius=float(np.linalg.norm(points-center,axis=1).max())
    # This truth is collision geometry only; estimator never reads this report.
    center_base=center+np.asarray(manifest['fixed_target_position_world_m'])-[0,0,.008]
    urdf=ET.parse(SOURCE/'models/r12/r12_arm.urdf').getroot()
    limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in urdf.findall('joint') if j.find('limit') is not None}
    views=[]
    for name,yaw,roll in POSES:
        q=list(manifest['q']);q[0]=yaw;q[4]=roll
        for i,value in enumerate(q):
            lo,hi=limits[f'arm_joint{i+1}']
            if not lo+.02<=value<=hi-.02:raise ValueError('joint_limit_margin:'+name)
        admission=audit(SOURCE,q,q,obstacle_spheres=[(center_base,radius)])
        if not admission['all_self_bounds_proven'] or not admission['environment_bounds_proven']:
            raise ValueError('static_pose_collision_screen_failed:'+name)
        views.append(dict(name='cal-view-'+name,split='holdout' if name.startswith('holdout') else 'train',q=q,admission=admission))
    report=dict(scope='static initial poses only; no traversal proved or executed',views=views,
                fit_must_not_read_this_collision_truth=True,motion_authorized=False,calibration_accepted=False)
    (BASE/'calibration_view_admissions.json').write_text(json.dumps(report,indent=2)+'\n')
    (BASE/'calibration_dataset_index.json').write_text(json.dumps(dict(
        views=[dict(run='runs/fresh-view-04',split='train')]+[dict(run='runs/'+v['name'],split=v['split']) for v in views],
        scope='simulation hand-eye candidate dataset; not physical calibration'),indent=2)+'\n')
    return views


def main():
    views=prepare()
    if '--prepare-only' in sys.argv:
        print(json.dumps([dict(name=v['name'],self_min=v['admission']['worst_pair']['lower_bound_m'],environment_min=v['admission']['environment_lower_bound_m']) for v in views],indent=2));return 0
    results=[]
    for view in views:
        launch.RUN=BASE/'runs'/view['name']
        launch.MANIFEST_OVERRIDES=dict(q=view['q'],identity_reobserve_challenge=False,
            calibration_view=True,calibration_split=view['split'],
            scope='independent stationary view for simulation-only camera calibration')
        launch.ADMISSION_REPORT=view['admission']
        code=launch.main()
        results.append(dict(run=view['name'],supervisor_exit_code=code))
        (BASE/'calibration_acquisition_progress.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(results[-1]),flush=True)
        if code:return code
    return 0


if __name__=='__main__':sys.exit(main())
