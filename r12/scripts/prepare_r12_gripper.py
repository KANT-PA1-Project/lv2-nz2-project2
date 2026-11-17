"""Split supplied CAD palms into symmetric moving fingers; preserve vendor assets.

Parallel jaw kinematic approximation of the confirmed OpenMANIPULATOR-X linkage.
Crank/rod animation, reflected inertia and force conversion are not calibrated.
"""
from pathlib import Path
import json,os,sys,copy,xml.etree.ElementTree as ET
import numpy as np,trimesh
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
src=root/'assets/vendor/R12_NO_USB_URDF_USD';out=root/'models/r12';md=out/'gripper_meshes';md.mkdir(exist_ok=True)
parts=trimesh.load(src/'meshes/arm_link5_polymer.stl').split(only_watertight=False)
groups={'fixed':[],'left':[],'right':[]}
for part in parts:
    # Both full palms and their rubber pads extend past x=80mm; no other component does.
    group=('left' if part.bounds.mean(axis=0)[1]>0 else 'right') if part.bounds[1,0]>.08 else 'fixed'
    groups[group].append(part)
meshes={k:trimesh.util.concatenate(v) for k,v in groups.items()}
for k,m in meshes.items():m.export(md/(k+'.stl'))
pads={}
for side in ['left','right']:
    candidates=[p for p in groups[side] if p.bounds[0,0]>.08]
    assert candidates,'CAD rubber pad absent'
    pads[side]=trimesh.util.concatenate(candidates).bounds
closed_gap=float(pads['left'][0,1]-pads['right'][1,1]);lower=(.020-closed_gap)/2;upper=(.075-closed_gap)/2
path=out/'r12_mobile_gripper.usda';path.unlink(missing_ok=True);stage=Usd.Stage.CreateNew(str(path))
r=UsdGeom.Xform.Define(stage,'/Robot');stage.SetDefaultPrim(r.GetPrim());r.GetPrim().GetReferences().AddReference(os.path.relpath(src/'usd/r12_no_usb.usdc',out));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
original=Usd.Stage.Open(str(src/'usd/r12_no_usb.usdc'));T=UsdGeom.Xformable(original.GetPrimAtPath('/Robot/arm_link5')).ComputeLocalToWorldTransform(0)
# Remove the old whole-hand collider that filled the gripping aperture.
stage.GetPrimAtPath('/Robot/arm_link5/collision_000').SetActive(False)
def mesh(path,m):
    g=UsdGeom.Mesh.Define(stage,path);g.CreatePointsAttr(m.vertices.tolist());g.CreateFaceVertexCountsAttr([3]*len(m.faces));g.CreateFaceVertexIndicesAttr(m.faces.flatten().tolist());g.CreateDisplayColorAttr([Gf.Vec3f(.16,.17,.19)]);return g
mesh('/Robot/arm_link5/visual_polymer',meshes['fixed'])
def box(path,bounds):
    b=np.asarray(bounds);g=UsdGeom.Cube.Define(stage,path);g.CreateSizeAttr(1);g.AddTranslateOp().Set(Gf.Vec3d(*b.mean(axis=0)));g.AddScaleOp().Set(Gf.Vec3f(*(b[1]-b[0])));g.CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(g.GetPrim());return g
box('/Robot/arm_link5/gripper_base_collision',[[-.003,-.065,-.03],[.065,.065,.04]])
box('/Robot/arm_link5/wrist_camera_collision',[[.01,-.045,.04],[.048,.045,.078]])
for side,sign in [('left',1),('right',-1)]:
    base='/Robot/gripper_'+side;body=UsdGeom.Xform.Define(stage,base);body.AddTransformOp().Set(T);UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());mass=UsdPhysics.MassAPI.Apply(body.GetPrim());mass.CreateMassAttr(.025);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-5))
    mesh(base+'/visual',meshes[side])
    # Tip contact pad follows the actual CAD; rear palm collision approximation documented.
    box(base+'/pad_collision',pads[side]);b=meshes[side].bounds.copy();b[1,0]=.084;b[:,1]=[.014,.039] if sign==1 else [-.039,-.014];box(base+'/palm_collision',b)
    j=UsdPhysics.PrismaticJoint.Define(stage,'/Robot/Joints/gripper_'+side+'_joint');j.CreateBody0Rel().SetTargets(['/Robot/arm_link5']);j.CreateBody1Rel().SetTargets([base]);j.CreateAxisAttr('Y');j.CreateLocalPos0Attr(Gf.Vec3f(0));j.CreateLocalPos1Attr(Gf.Vec3f(0))
    j.CreateLowerLimitAttr(lower if sign==1 else -upper);j.CreateUpperLimitAttr(upper if sign==1 else -lower)
    drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(1000);drive.CreateDampingAttr(20);drive.CreateMaxForceAttr(10);drive.CreateTargetPositionAttr(sign*upper)
    # Same hardware input, mirrored simulation targets. No separate independent finger commands.
    j.GetPrim().CreateAttribute('physxJoint:maxJointVelocity',Sdf.ValueTypeNames.Float).Set(.04)
    UsdPhysics.FilteredPairsAPI.Apply(body.GetPrim()).CreateFilteredPairsRel().SetTargets(['/Robot/arm_link5'])
stage.GetRootLayer().Save()
# Mobile URDF uses identical CAD pieces and axis conventions.
tree=ET.parse(src/'urdf/r12_no_usb.urdf');robot=tree.getroot()
for el in robot.iter('mesh'):
    el.set('filename',os.path.relpath((src/'urdf'/el.get('filename')).resolve(),out))
hand=robot.find("link[@name='arm_link5']")
for visual in hand.findall('visual'):
    m=visual.find('geometry/mesh')
    if m is not None and 'polymer' in m.get('filename'):m.set('filename','gripper_meshes/fixed.stl')
for col in hand.findall('collision'):hand.remove(col)
def urdf_box(link,b):
    b=np.array(b);c=ET.SubElement(link,'collision');ET.SubElement(c,'origin',xyz=' '.join(map(str,b.mean(axis=0))),rpy='0 0 0');ET.SubElement(ET.SubElement(c,'geometry'),'box',size=' '.join(map(str,b[1]-b[0])))
urdf_box(hand,[[-.003,-.065,-.03],[.065,.065,.04]]);urdf_box(hand,[[.01,-.045,.04],[.048,.045,.078]])
for side,sign in [('left',1),('right',-1)]:
    link=ET.SubElement(robot,'link',name='gripper_'+side);visual=ET.SubElement(link,'visual');ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename='gripper_meshes/'+side+'.stl')
    urdf_box(link,pads[side]);b=meshes[side].bounds.copy();b[1,0]=.084;b[:,1]=[.014,.039] if sign==1 else [-.039,-.014];urdf_box(link,b)
    inertia=ET.SubElement(link,'inertial');ET.SubElement(inertia,'mass',value='.025');ET.SubElement(inertia,'inertia',ixx='.00001',iyy='.00001',izz='.00001',ixy='0',ixz='0',iyz='0')
    j=ET.SubElement(robot,'joint',name='gripper_'+side+'_joint',type='prismatic');ET.SubElement(j,'parent',link='arm_link5');ET.SubElement(j,'child',link='gripper_'+side);ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0');ET.SubElement(j,'axis',xyz='0 1 0');ET.SubElement(j,'limit',lower=str(lower if sign==1 else -upper),upper=str(upper if sign==1 else -lower),effort='10',velocity='.04')
    if sign<0:ET.SubElement(j,'mimic',joint='gripper_left_joint',multiplier='-1',offset='0')
ET.SubElement(robot,'link',name='grasp_tcp');j=ET.SubElement(robot,'joint',name='grasp_tcp_fixed',type='fixed');ET.SubElement(j,'parent',link='arm_link5');ET.SubElement(j,'child',link='grasp_tcp');ET.SubElement(j,'origin',xyz='.12475 0 0',rpy='0 0 0')
ET.indent(tree);tree.write(out/'r12_mobile_gripper.urdf',encoding='utf-8',xml_declaration=True)
cfg=json.loads((root/'configs/r12_gripper.json').read_text());cfg.update({'cad_zero_aperture_m':closed_gap,'simulation_finger_displacement_m':[lower,upper],'simulation_force_limit_per_finger_N':10,'simulation_speed_per_finger_m_s':.04,'simulation_mass_per_finger_kg':.025,'assumptions':'CAD palms/pads retained, parallel-slide approximation; crank/rods remain visual only. Force, speed and mass are provisional, not measured motor equivalents. Existing whole robot assumed mass not reidentified.','usd':'models/r12/r12_mobile_gripper.usda','urdf':'models/r12/r12_mobile_gripper.urdf','model_prepared':True,'simulation_integration_complete':False});(root/'configs/r12_gripper.json').write_text(json.dumps(cfg,indent=2));print(json.dumps({k:cfg[k] for k in ['cad_zero_aperture_m','simulation_finger_displacement_m']}))
