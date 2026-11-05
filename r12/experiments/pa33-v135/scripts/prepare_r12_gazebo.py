"""Convert authored scene and mobile CAD model for Gazebo scan/drive trials.

Fixture coordinates enter the simulator only. Controllers consume sensors.
"""
import argparse
import copy
import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as E
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from r12_gazebo_env import environment

repo = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(repo/'src'), str(repo/'scripts/sensor_runtime')]
from camera_rig import PreparedCameraRig
from arm_practice.kinematics import forward_transform


def sub(parent, tag, text=None, **attrs):
    item = E.SubElement(parent, tag, attrs)
    if text is not None:
        item.text = str(text)
    return item


def nums(items):
    return ' '.join(format(float(x), '.12g') for x in items)


def pose(parent, matrix):
    sub(parent, 'pose', nums([*matrix[:3,3], *Rotation.from_matrix(matrix[:3,:3]).as_euler('xyz')]))


def plugin(parent, filename, name):
    return sub(parent, 'plugin', filename=filename, name=name)


def material(gprim, visual, out):
    mat = UsdShade.MaterialBindingAPI(gprim).ComputeBoundMaterial()[0]
    color = [.5,.5,.5]
    tex = None
    if mat:
        shader = UsdShade.Shader(mat.GetPrim().GetChild('Shader'))
        val = shader.GetInput('diffuseColor').Get() if shader else None
        if val is not None:
            color = list(val)
        texture = UsdShade.Shader(mat.GetPrim().GetChild('Texture'))
        if texture:
            asset = texture.GetInput('file').Get()
            if asset:
                tex = (repo/'models/navigation'/asset.path).resolve()
                if not tex.is_file():
                    raise ValueError('Missing texture '+str(tex))
    else:
        colors = UsdGeom.Gprim(gprim).GetDisplayColorAttr().Get()
        if colors:
            color = list(colors[0])
    m = sub(visual, 'material')
    sub(m, 'ambient', nums([*color, 1]))
    sub(m, 'diffuse', nums([*color, 1]))
    sub(m, 'specular', '0.05 0.05 0.05 1')
    if tex:
        metal = sub(sub(m,'pbr'),'metal')
        sub(metal,'albedo_map',str(tex))
        sub(metal,'roughness',.55)
        sub(metal,'metalness',0)


def geometry(gprim, parent, reference, cache, out, name):
    t = np.asarray(cache.GetLocalToWorldTransform(gprim),float).T
    t = np.linalg.inv(reference) @ t
    scale = np.linalg.norm(t[:3,:3],axis=0)
    rigid = t.copy(); rigid[:3,:3] /= scale
    geom = sub(parent,'geometry')
    if gprim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(gprim)
        points = np.asarray(mesh.GetPointsAttr().Get(),float)
        points = points @ t[:3,:3].T + t[:3,3]
        counts = list(mesh.GetFaceVertexCountsAttr().Get())
        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
        uvvar = UsdGeom.PrimvarsAPI(mesh).GetPrimvar('st')
        uv = uvvar.ComputeFlattened() if uvvar else None
        faceuv = uv is not None and uvvar.GetInterpolation() == 'faceVarying'
        lines = ['v '+nums(p) for p in points]
        if uv is not None:
            lines += ['vt '+nums(p) for p in uv]
        start = 0
        normal_count = 0
        for face_index,count in enumerate(counts):
            polygon = points[np.asarray(indices[start:start+count])]
            normal = np.cross(polygon, np.roll(polygon,-1,axis=0)).sum(axis=0)
            if np.linalg.norm(normal)<1e-14:
                start += count
                continue  # Zero-area source faces have no surface or collision volume.
            normal /= np.linalg.norm(normal)
            normal_count += 1
            lines.append('vn '+nums(normal))
            entries = []
            for k in range(start,start+count):
                index = indices[k]+1
                entries.append(f'{index}//{normal_count}' if uv is None else f'{index}/{k+1 if faceuv else index}/{normal_count}')
            lines.append('f '+' '.join(entries)); start += count
        path = out/'meshes'/f'{name}.obj'; path.write_text('\n'.join(lines)+'\n')
        sub(sub(geom,'mesh'),'uri',str(path))
    else:
        if gprim.IsA(UsdGeom.Cube):
            sub(sub(geom,'box'),'size',nums(scale*UsdGeom.Cube(gprim).GetSizeAttr().Get()))
        elif gprim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(gprim)
            axis = str(shape.GetAxisAttr().Get())
            if axis != 'Z':
                rotation = Rotation.from_euler('y' if axis=='X' else 'x',math.pi/2).as_matrix()
                rigid[:3,:3] = rigid[:3,:3]@rotation
            direction = {'X':0,'Y':1,'Z':2}[axis]
            radial = [i for i in range(3) if i!=direction]
            if not np.isclose(scale[radial[0]],scale[radial[1]],rtol=1e-5):
                raise ValueError('Elliptical cylinder requires mesh conversion')
            cyl = sub(geom,'cylinder')
            sub(cyl,'radius',shape.GetRadiusAttr().Get()*scale[radial[0]])
            sub(cyl,'length',shape.GetHeightAttr().Get()*scale[direction])
        elif gprim.IsA(UsdGeom.Sphere):
            if not np.allclose(scale,scale[0]):
                raise ValueError('Ellipsoid requires mesh conversion')
            sub(sub(geom,'sphere'),'radius',UsdGeom.Sphere(gprim).GetRadiusAttr().Get()*scale[0])
        else:
            raise ValueError('Unsupported scene primitive '+str(gprim.GetTypeName()))
        pose(parent,rigid)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True)
    args = ap.parse_args(); out = args.output.resolve(); out.mkdir(parents=True,exist_ok=False)
    (out/'meshes').mkdir()
    urdf_path = repo/'models/r12/r12_mobile_gripper.urdf'
    robot = E.parse(urdf_path).getroot()
    robot.set('name','r12')
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename',str((urdf_path.parent/mesh.get('filename')).resolve()))
    for joint in robot.findall('joint'):
        name = joint.get('name'); lim = joint.find('limit')
        if name.endswith('_wheel_joint'):
            lim.set('effort','.108'); lim.set('velocity','29.3215314335')
        if name.endswith('_steer_joint'):
            lim.set('effort','.54'); lim.set('velocity','7.330382858')
    E.indent(robot); E.ElementTree(robot).write(out/'robot.urdf',encoding='unicode')
    conversion = subprocess.run(['gz','sdf','-p',str(out/'robot.urdf')],env=environment(),capture_output=True,text=True)
    (out/'urdf_conversion.log').write_text(conversion.stderr)
    conversion.check_returncode()
    sdf_robot = E.fromstring(conversion.stdout); model = sdf_robot.find('model')
    if model is None: raise RuntimeError('No converted robot model')
    expected = {j.get('name') for j in robot.findall('joint') if j.get('type')!='fixed'}
    if {j.get('name') for j in model.findall('joint')} != expected:
        raise RuntimeError('Conversion lost active joints')
    sub(model,'pose','.734 .950 .008 0 0 3.14159265359')
    plugin(model,'gz-sim-joint-state-publisher-system','gz::sim::systems::JointStatePublisher')
    # Force-based PID: no direct joint velocity setting or model teleport control.
    for joint in model.findall('joint'):
        name = joint.get('name')
        wheel = name.endswith('_wheel_joint')
        p = plugin(model,'gz-sim-joint-controller-system' if wheel else 'gz-sim-joint-position-controller-system',
                   'gz::sim::systems::JointController' if wheel else 'gz::sim::systems::JointPositionController')
        sub(p,'joint_name',name)
        sub(p,'topic','/r12/gz/command/'+name)
        if wheel:
            sub(p,'use_force_commands','true'); gains=(.02,0,0,.108); sub(p,'initial_velocity',0)
        elif name.endswith('_steer_joint'):
            gains=(5,0,.25,.54); sub(p,'initial_position',0)
        elif name.startswith('arm_joint'):
            gains=(80,0,8,2); sub(p,'initial_position',0)
        else:
            gains=(1000,0,20,10)
            sub(p,'initial_position',.027686130628 if 'left' in name else -.027686130628)
        for key,value in zip(['p_gain','i_gain','d_gain','cmd_max'],gains): sub(p,key,value)
        sub(p,'cmd_min',-gains[-1]); sub(p,'i_max',0); sub(p,'i_min',0)
    links = {link.get('name'):link for link in model.findall('link')}
    cameras = PreparedCameraRig(repo).transforms({f'arm_joint{i}':0. for i in range(1,6)})
    arm = forward_transform(repo/'models/r12/r12_arm.urdf','arm_link5',{f'arm_joint{i}':0. for i in range(1,6)})
    opt_to_gz = np.eye(4); opt_to_gz[:3,:3] = [[0,-1,0],[0,0,-1],[1,0,0]]
    topics = []
    for name,transform in cameras.items():
        parent = 'arm_link5' if name.startswith('arm_camera') else 'base_link'
        local = np.linalg.inv(arm)@transform if parent=='arm_link5' else transform
        left = name.endswith('left_optical')
        sensor = sub(links[parent],'sensor',name=name,type='rgbd_camera' if left else 'camera')
        pose(sensor,local@opt_to_gz); sub(sensor,'topic','/r12/gz/'+name+('' if left else '/image'))
        sub(sensor,'frame_id',name); sub(sensor,'update_rate',20); sub(sensor,'always_on','true')
        camera = sub(sensor,'camera'); sub(camera,'horizontal_fov',2*math.atan(.8))
        image = sub(camera,'image'); sub(image,'width',640); sub(image,'height',480); sub(image,'format','R8G8B8')
        clip = sub(camera,'clip'); sub(clip,'near',.01); sub(clip,'far',10)
        topics.append('/r12/gz/'+name+'/image')
        if left: topics += ['/r12/gz/'+name+'/depth_image','/r12/gz/'+name+'/points']
    for i,x in enumerate([.057,-.057]):
        sensor=sub(links['base_link'],'sensor',name=f'imu{i+1}',type='imu')
        sub(sensor,'pose',f'{x} 0 .16597 0 0 {0 if i==0 else math.pi}')
        sub(sensor,'topic',f'/r12/gz/imu{i+1}'); sub(sensor,'update_rate',120);sub(sensor,'always_on','true')
        sub(sensor,'frame_id',f'opencr_{i+1}');sub(sensor,'imu'); topics.append(f'/r12/gz/imu{i+1}')
    for name in ['front_right','rear_left','low_front_left','low_rear_right']:
        sensor=sub(links['base_link'],'sensor',name='lidar_'+name,type='gpu_lidar')
        pose(sensor,forward_transform(repo/'models/r12/r12_arm.urdf','lidar_'+name,{}))
        sub(sensor,'frame_id','lidar_'+name);sub(sensor,'topic','/r12/gz/lidar/'+name);sub(sensor,'update_rate',20);sub(sensor,'always_on','true')
        lidar=sub(sensor,'lidar');horizontal=sub(sub(lidar,'scan'),'horizontal')
        sub(horizontal,'samples',360);sub(horizontal,'resolution',1);sub(horizontal,'min_angle',-math.pi);sub(horizontal,'max_angle',math.pi)
        limit=sub(lidar,'range');sub(limit,'min',.03);sub(limit,'max',8);sub(limit,'resolution',.005)
        topics.append('/r12/gz/lidar/'+name)
    sdf = E.Element('sdf',version='1.12');world=sub(sdf,'world',name='r12_scan_drive')
    physics=sub(world,'physics',name='r12_physics',type='ignored');sub(physics,'max_step_size',1/960);sub(physics,'real_time_factor',1.)
    sub(world,'gravity','0 0 -9.81')
    for lib,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('imu','Imu')]:
        plugin(world,'gz-sim-'+lib+'-system','gz::sim::systems::'+name)
    sensors=plugin(world,'gz-sim-sensors-system','gz::sim::systems::Sensors');sub(sensors,'render_engine','ogre2')
    light=sub(world,'light',name='sun',type='directional');sub(light,'pose','0 0 8 0 0 0');sub(light,'diffuse','1 1 1 1');sub(light,'direction','-.3 -.2 -1');sub(light,'cast_shadows','true')
    scene=sub(world,'scene');sub(scene,'ambient','.6 .6 .6 1');sub(scene,'background','.7 .7 .7 1')
    floor=sub(world,'model',name='ground');sub(floor,'static','true');fl=sub(floor,'link',name='ground_link')
    for kind in ['visual','collision']:
        it=sub(fl,kind,name='ground_'+kind);plane=sub(sub(it,'geometry'),'plane');sub(plane,'normal','0 0 1');sub(plane,'size','200 200')
        if kind=='visual':m=sub(it,'material');sub(m,'diffuse','.45 .45 .45 1');sub(m,'ambient','.45 .45 .45 1')
    source=Usd.Stage.Open(str(repo/'models/navigation/mission_scene.usda'));cache=UsdGeom.XformCache()
    scene_model=sub(world,'model',name='mission_environment');sub(scene_model,'static','true');scene_link=sub(scene_model,'link',name='environment')
    puck_links={};counts={'visuals':0,'collisions':0,'pucks':0};initial=np.eye(4)
    for prim in source.Traverse():
        if not prim.IsA(UsdGeom.Gprim):continue
        name=str(prim.GetPath()).strip('/').replace('/','_')
        link=scene_link;reference=initial
        if str(prim.GetPath()).startswith('/World/Pucks/'):
            puck_name=str(prim.GetPath()).split('/')[3]
            if puck_name not in puck_links:
                pp=source.GetPrimAtPath('/World/Pucks/'+puck_name)
                reference=np.asarray(cache.GetLocalToWorldTransform(pp),float).T
                pm=sub(world,'model',name=puck_name);pose(pm,reference);pl=sub(pm,'link',name='body')
                inertial=sub(pl,'inertial');sub(inertial,'mass',.025);inertia=sub(inertial,'inertia')
                for key,value in [('ixx',9.375e-6),('iyy',9.375e-6),('izz',3.75e-6)]:sub(inertia,key,value)
                puck_links[puck_name]=(pl,reference);counts['pucks']+=1
            link,reference=puck_links[puck_name]
        visible=UsdGeom.Imageable(prim).ComputeVisibility()!=UsdGeom.Tokens.invisible and '/collision/' not in str(prim.GetPath())
        if visible:
            visual=sub(link,'visual',name=name);geometry(prim,visual,reference,cache,out,name);material(prim,visual,out);counts['visuals']+=1
        if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()!=False:
            collision=sub(link,'collision',name=name+'_collision');geometry(prim,collision,reference,cache,out,name);counts['collisions']+=1
    world.append(model)
    E.indent(sdf);E.ElementTree(sdf).write(out/'world.sdf',encoding='unicode',xml_declaration=True)
    report={'scope':'Gazebo model conversion, not mission validation','scene_counts':counts,'active_joints':sorted(expected),
            'robot_links':len(links),'camera_bodies':4,'stereo_images':8,'depth_cameras':4,'dual_imu':2,'lidars':4,'expected_sensor_topics':topics,
            'target_real_time_factor':1.,'physics_hz':960,'source_urdf_sha256':hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
            'arm_wrist_cycle_enabled':False,'grasp_enabled':False,'hardware_enabled':False,
            'remaining':['Live sensor readiness','Torque-speed envelope and watchdog adapter','Existing ROS navigation and inference adapter','Start light sequence controller'],
            'mass_and_inertia':'Simulation assumptions, not hardware certification','sources':str(repo/'models/navigation/mission_scene.usda')}
    (out/'conversion.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
