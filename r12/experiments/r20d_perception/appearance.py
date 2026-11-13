"""Simulation appearance overrides; no collider or perception changes."""
from pathlib import Path


def asset_paths(cfg):
    source = cfg.get('floor_visual_usd')
    if not source:
        return []
    source = Path(source)
    return [source, *(source.parent / 'textures/class_room_floor' / name for name in (
        'class_room_floor_albedo.png', 'class_room_floor_normal.png'))]


def apply_floor(stage, ground, source):
    from pxr import Usd, UsdGeom, UsdShade, UsdPhysics
    # Retain the tested physical plane. Hide its visual geometry only.
    hidden = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ground.prim_path), Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            if prim.IsInstanceProxy():
                raise ValueError('Default ground visual unexpectedly instanced')
            UsdGeom.Imageable(prim).MakeInvisible()
            hidden.append(str(prim.GetPath()))
    if not hidden:
        raise ValueError('Default ground has no visual mesh to replace')
    material = UsdShade.Material.Define(stage, '/World/Looks/Module6Carpet')
    material.GetPrim().GetReferences().AddReference(
        source, '/World/Environment/GroundPlane/class_room_floor')
    mesh = stage.DefinePrim('/World/Module6CarpetVisual', 'Mesh')
    mesh.GetReferences().AddReference(source, '/World/Environment/GroundPlane/CollisionMesh')
    if mesh.HasAPI(UsdPhysics.CollisionAPI):
        raise ValueError('Floor appearance reference must be visual only')
    UsdShade.MaterialBindingAPI.Apply(mesh).Bind(material)
    if not UsdGeom.Mesh(mesh).GetPointsAttr().Get():
        raise ValueError('Source carpet mesh is empty')
    return dict(source=source, hidden_visuals=hidden, physics='unchanged_default_ground',
                material=str(material.GetPath()), visual=str(mesh.GetPath()))


def apply_gripper(stage, robot_prim):
    from pxr import Usd, UsdShade, Sdf, Gf
    material = UsdShade.Material.Define(stage, '/World/Looks/BlackGripper')
    shader = UsdShade.Shader.Define(stage, '/World/Looks/BlackGripper/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.01))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.6)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    links = [p for p in Usd.PrimRange(robot_prim)
             if p.GetName() in {'arm_link5', 'gripper_left', 'gripper_right'}]
    if len(links) != 3:
        raise ValueError('Expected wrist and two finger links for black override')
    for prim in links:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    return dict(links=[str(p.GetPath()) for p in links], diffuse_linear=[.01]*3,
                roughness=.6, note='Nominal black wrist/fingers, no geometry change')
