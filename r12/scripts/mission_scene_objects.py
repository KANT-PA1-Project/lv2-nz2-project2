"""Authored simulation fixtures; these labels are never controller observations."""
import json, math, sys
import numpy as np
from PIL import Image
from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

def add_mission_objects(stage, root, boxes):
    sys.path.insert(0, str(root/'work/perception_deps'))
    import barcode
    from barcode.writer import ImageWriter
    fixture=json.loads((root/'configs/mission/random_20261030.json').read_text())
    textures=root/'models/navigation/textures';textures.mkdir(exist_ok=True)
    patterns=json.loads(open('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/config/perception/tag36h11.json').read())
    def material(name,color,texture=None):
        mat=UsdShade.Material.Define(stage,'/World/FixtureMaterials/'+name)
        sh=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Shader');sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(1.)
        sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
        if texture:
            uv=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            tex=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Texture');tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set('textures/'+texture.name)
            tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
            sh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
        return mat,sh
    def bind(geom,mat):UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
    # Explicit opaque wall material; preserve the supplied blue acrylic colour.
    wall,_=material('OpaqueAcrylic',(.0611,.0807,.163))
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Gprim):
            old=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if old and old.GetPath().name=='wall':
                bind(UsdGeom.Gprim(prim),wall)
                UsdGeom.Gprim(prim).CreateDisplayOpacityAttr([1.])
    def quad(path,half,mat,curved=False):
        mesh=UsdGeom.Mesh.Define(stage,path);points=[];uv=[];strips=24 if curved else 1
        for j in range(strips):
            for u,v in [(j/strips,0),((j+1)/strips,0),((j+1)/strips,1),(j/strips,1)]:
                x,y=(2*u-1)*half[0],(2*v-1)*half[1]
                points.append((.01503*math.sin(x/.01503) if curved else x,y,.01503*(math.cos(x/.01503)-1) if curved else 0))
                uv.append((u,v))
        mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr([4]*strips);mesh.CreateFaceVertexIndicesAttr(list(range(4*strips)));mesh.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set(uv)
        bind(mesh,mat)
        return mesh
    colors={c:material(c,rgb)[0] for c,rgb in [('R',(.7,.025,.025)),('G',(.025,.55,.04))]}
    byname={b['name']:b for b in boxes};counts={};placements=[]
    for p in fixture['pucks']:
        n=counts.get(p['source'],0);counts[p['source']]=n+1
        center=np.array(byname[p['source']]['center']);center[0]+=[-.045,.045][n];center[2]=.1001
        path='/World/Pucks/Puck'+str(p['id']);body=UsdGeom.Xform.Define(stage,path);body.AddTranslateOp().Set(Gf.Vec3d(*center))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.025)
        if p['shape']=='cylinder':
            geom=UsdGeom.Cylinder.Define(stage,path+'/Body');geom.CreateRadiusAttr(.015);geom.CreateHeightAttr(.06);geom.CreateAxisAttr('Z')
        else:
            geom=UsdGeom.Cube.Define(stage,path+'/Body');geom.CreateSizeAttr(1);geom.AddScaleOp().Set(Gf.Vec3f(.03,.03,.06))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim());bind(geom,colors[p['color']])
        texture=textures/f'tag36h11-{p["id"]}.png'
        Image.fromarray(np.pad(np.array(patterns[str(p['id'])],dtype=np.uint8)*255,1,constant_values=255)).resize((500,500),Image.Resampling.NEAREST).save(texture)
        mat,_=material('Tag'+str(p['id']),(1,1,1),texture)
        faces=[('top',(0,0,1),.03),('bottom',(0,0,-1),.03)]
        faces+= [('px',(1,0,0),.015),('nx',(-1,0,0),.015),('py',(0,1,0),.015),('ny',(0,-1,0),.015)] if p['shape']=='cuboid' else [('side',(1,0,0),.015)]
        for name,normal,distance in faces:
            face=UsdGeom.Xform.Define(stage,path+'/'+name)
            rot=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*normal)))
            if name=='side':rot=Gf.Matrix4d(0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1)
            face.AddTransformOp().Set(rot*Gf.Matrix4d().SetTranslate(Gf.Vec3d(*normal)*(distance+.00003)))
            quad(str(face.GetPath())+'/Surface',(.01,.01),mat,name=='side')
        placements.append({**p,'center_m':center.tolist(),'mass_assumed_kg':.025})
    # Vertical fixture's lower reference is exactly the mission coordinate.
    # Panel dimensions/relative component heights are provisional: PDF specifies no dimensions.
    panel=UsdGeom.Xform.Define(stage,'/World/StartPanel');panel.AddTranslateOp().Set(Gf.Vec3d(.08,.95,0))
    board=UsdGeom.Cube.Define(stage,'/World/StartPanel/Board');board.CreateSizeAttr(1);board.AddTranslateOp().Set(Gf.Vec3d(0,0,.40));board.AddScaleOp().Set(Gf.Vec3f(.015,.50,.80));UsdPhysics.CollisionAPI.Apply(board.GetPrim());bind(board,material('Panel',(.08,.08,.08))[0])
    codepath=textures/'mission_code128.png'
    barcode.get('code128',fixture['barcode'],writer=ImageWriter()).write(codepath.open('wb'),options={'module_width':.5,'module_height':22.,'quiet_zone':7.,'write_text':False})
    bc=UsdGeom.Xform.Define(stage,'/World/StartPanel/Barcode');bc.AddTransformOp().Set(Gf.Matrix4d(0,1,0,0,0,0,1,0,1,0,0,0,.008,0,.30,1))
    quad(str(bc.GetPath())+'/Surface',(.21,.085),material('Code128',(1,1,1),codepath)[0])
    for c,y in [('R',-.14),('Y',0),('G',.14)]:
        lamp=UsdGeom.Sphere.Define(stage,'/World/StartPanel/Lamp'+c);lamp.CreateRadiusAttr(.04);lamp.AddTranslateOp().Set(Gf.Vec3d(.014,y,.58))
        mat,sh=material('Lamp'+c,(.015,.015,.015));sh.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0));bind(lamp,mat)
    (root/'models/navigation/scene_fixture.json').write_text(json.dumps({'pucks':placements,'barcode':fixture['barcode'],'sequence':fixture['sequence'],'signal_period_s':2,'signal_on_s':1,'end_off_s':3,'central_box_count':1,'panel_reference_m':[.08,.95,0],'panel_size_assumed_m':[.015,.50,.80],'evidence_kind':'simulation_fixture_not_perception'},indent=2))
