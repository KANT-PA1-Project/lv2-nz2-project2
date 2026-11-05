"""Scene setup only. Fixture poses never enter sensor observations."""
import numpy as np
import json
from pathlib import Path
CASE=json.loads((Path(__file__).resolve().parents[3]/'heldout_case.json').read_text())
def setup_pose(T_world_base,T_base_camera,distance=.25):
    T=np.asarray(T_world_base)@np.asarray(T_base_camera)
    p=(T@np.array([0.,0.,distance,1.]))[:3]
    return p

def build(stage,T_base_camera):
    from pxr import UsdGeom,UsdPhysics,Gf
    T=np.array(UsdGeom.Xformable(stage.GetPrimAtPath('/World/Spawn')).ComputeLocalToWorldTransform(0)).T
    center=setup_pose(T,T_base_camera,distance=CASE['distance_m'])
    for name in ['Maze','Tables','StartPanel']:
        prim=stage.GetPrimAtPath('/World/Mission/'+name)
        if prim.IsValid():prim.SetActive(False)
    for i in range(12):
        if i!=0:stage.GetPrimAtPath('/World/Mission/Pucks/Puck'+str(i)).SetActive(False)
    puck=stage.GetPrimAtPath('/World/Mission/Pucks/Puck0')
    puck.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*center))
    UsdGeom.Xformable(puck).AddRotateZOp().Set(CASE['yaw_deg'])
    top=center[2]-.03-.0001;height=top
    if height<=.1:raise ValueError('invalid_fixture_height')
    support=UsdGeom.Cube.Define(stage,'/World/SinglePuckSupport');support.CreateSizeAttr(1.)
    support.AddTranslateOp().Set(Gf.Vec3d(float(center[0]),float(center[1]),float(height/2)))
    support.AddScaleOp().Set(Gf.Vec3f(.14,.14,float(height)))
    support.CreateDisplayColorAttr([Gf.Vec3f(.3,.3,.3)]);UsdPhysics.CollisionAPI.Apply(support.GetPrim())
    return dict(scope='Evaluator/setup only; not a perception or controller input',asset='/World/Mission/Pucks/Puck0',nominal_shape='existing green cuboid',setup_world_center=center.tolist(),support_top_m=float(top),optical_fixture_distance_m=CASE['distance_m'],setup_yaw_deg=CASE['yaw_deg'],body_attachment=False,object_pose_reads_for_control=False,motion_authorized=False)
