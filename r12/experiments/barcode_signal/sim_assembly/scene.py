"""Fixture authoring and conservative proximity veto for the isolated trial.

r20d_perception/scene.py 사본. 팔 링크 이름만 최종 조립 URDF(ARM_LINKS)로 바꿈.
"""
import numpy as np

from common import ARM_LINKS


def build_fixture(stage, cfg, rule):
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, UsdPhysics
    from common import R12
    from barcode_support import add_label

    stage.DefinePrim('/World/Mission', 'Xform').GetReferences().AddReference(
        str(R12 / 'models/navigation/mission_scene.usda'))
    for suffix in ('Board', 'Barcode', 'LampR', 'LampY', 'LampG'):
        prim = stage.GetPrimAtPath('/World/Mission/StartPanel/' + suffix)
        if prim.IsValid():
            prim.SetActive(False)
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(cfg['dome'])
    if cfg['barcode'].get('render', 'texture') == 'geometry':
        add_label_geometry(stage, cfg['barcode']['code'], cfg['barcode']['center_m'])
    else:
        add_label(stage, cfg['barcode']['code'], cfg['barcode']['center_m'])

    def material(name, rgb, emission=0.):
        mat = UsdShade.Material.Define(stage, '/World/Looks/' + name)
        sh = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Shader')
        sh.CreateIdAttr('UsdPreviewSurface')
        color = np.asarray(rule.srgb_to_linear(rgb))
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(color * emission)))
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.5)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        return mat

    tower, look = dict(rule.TOWER), rule.TOWER_LOOK
    if cfg.get('tower_scene'):
        tower['xy_m'] = cfg['tower_scene']['xy_m']
        tower['base_z_m'] = cfg['tower_scene'].get('base_z_m', tower['base_z_m'])
    tx, ty = tower['xy_m']
    base = tower['base_z_m']
    radius = tower['lens_diameter_m'] / 2
    lo = min(pair[0] for pair in tower['tiers'].values())
    hi = max(pair[1] for pair in tower['tiers'].values())
    root = UsdGeom.Xform.Define(stage, '/World/Tower')

    def cylinder(name, z, height, r, mat):
        geom = UsdGeom.Cylinder.Define(stage, str(root.GetPath()) + '/' + name)
        geom.CreateRadiusAttr(r)
        geom.CreateHeightAttr(height)
        geom.CreateAxisAttr('Z')
        geom.AddTranslateOp().Set(Gf.Vec3d(tx, ty, base + z))
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        return geom

    for i, (sx, sy) in enumerate([look['base_size_m'], look['base_size_m'][::-1]]):
        geom = UsdGeom.Cube.Define(stage, f'/World/Tower/Base{i}')
        geom.CreateSizeAttr(1.)
        geom.AddTranslateOp().Set(Gf.Vec3d(tx, ty, base + rule._BASE_H / 2))
        geom.AddScaleOp().Set(Gf.Vec3f(sx, sy, rule._BASE_H))
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material(f'Base{i}', look['base_srgb']))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    cylinder('Pole', (lo + rule._BASE_H) / 2, lo - rule._BASE_H,
             look['pole_radius_m'], material('Pole', look['pole_srgb']))
    cylinder('Cap', hi + look['cap_h_m'] / 2, look['cap_h_m'], radius,
             material('Cap', look['cap_srgb']))
    lamps, materials = {}, {}
    for c in 'RYG':
        z0, z1 = tower['tiers'][c]
        off = material('Off' + c, look['lens_off_srgb'][c])
        on = material('On' + c, look['lens_on_srgb'][c], cfg['emissive'])
        lamps[c] = cylinder('Lamp' + c, (z0 + z1) / 2, z1 - z0, radius, off)
        materials[c] = (off, on)
    previous = [None]

    def set_signal(color):
        if color == previous[0]:
            return
        for c in 'RYG':
            UsdShade.MaterialBindingAPI.Apply(lamps[c].GetPrim()).Bind(materials[c][int(c == color)])
        previous[0] = color

    return set_signal


def add_label_geometry(stage, code, center):
    """텍스처 대신 막대 형상으로 만든 75x25 mm 라벨 (+X 방향, barcode_support.label_corners 와 같은 배치).

    B04/B05: 1200 px 텍스처를 약 180 px 로 축소 렌더하면 가는 막대 대비가 이상값의 약 1/3 로 떨어져
    해독 불가(밉맵 필터링 [추정]). 막대 위치는 references/<code>.png 가운데 행에서 그대로 읽는다.
    """
    import cv2
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    from barcode_support import LABEL_M, ROOT
    image = cv2.imread(str(ROOT / 'references' / (code + '.png')), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(code)
    height_px, width_px = image.shape
    dark = image < 128
    rows = np.where(dark.any(1))[0]
    row = dark[(rows.min() + rows.max()) // 2]
    runs, x = [], 0
    while x < width_px:
        if row[x]:
            start = x
            while x < width_px and row[x]:
                x += 1
            runs.append((start, x))
        else:
            x += 1
    cx, cy, cz = map(float, center)
    w, h = LABEL_M

    def y_of(px):
        return cy - w/2 + px / width_px * w      # 텍스처 u(왼→오) = 월드 +Y

    def z_of(py):
        return cz + h/2 - py / height_px * h     # 이미지 위쪽 = 월드 +Z

    def quad_mesh(path, quads, x_off, rgb):
        mesh = UsdGeom.Mesh.Define(stage, path)
        points, counts, indices = [], [], []
        for y0, y1, z0, z1 in quads:
            base = len(points)
            points += [Gf.Vec3f(cx + x_off, y0, z0), Gf.Vec3f(cx + x_off, y1, z0),
                       Gf.Vec3f(cx + x_off, y1, z1), Gf.Vec3f(cx + x_off, y0, z1)]
            counts.append(4)
            indices += [base, base + 1, base + 2, base + 3]
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mat = UsdShade.Material.Define(stage, path + 'Look')
        shader = UsdShade.Shader.Define(stage, path + 'Look/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    # 흰 종이(텍스처 흰색 sRGB 255 = 선형 1.0), 그 앞 0.2 mm 에 검은 막대(선형 0)
    quad_mesh('/World/TestBarcodePaper', [(cy - w/2, cy + w/2, cz - h/2, cz + h/2)], 0., (1., 1., 1.))
    bars = [(y_of(s), y_of(e), z_of(rows.max() + 1), z_of(rows.min())) for s, e in runs]
    quad_mesh('/World/TestBarcodeBars', bars, 2e-4, (0., 0., 0.))
    return dict(code=code, center_world_m=list(center), label_m=list(LABEL_M), bars=len(runs),
                render='geometry', source=str(ROOT / 'references' / (code + '.png')))


class ClearanceVeto:
    """Conservative CAD AABB separation from non-robot colliders, 3 cm.

    This is a diagnostic stop, not a trajectory planner or full self-collision
    proof. Self collision remains enabled in PhysX. Empty bounds fail closed.
    """
    def __init__(self, stage, robot_path, margin=.03):
        from pxr import UsdGeom, UsdPhysics
        self.UsdGeom, self.stage, self.margin = UsdGeom, stage, margin
        self.arm, self.environment = [], []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            if enabled is False:
                continue
            path = str(prim.GetPath())
            if path.startswith(robot_path + '/'):
                if any('/' + name + '/' in path for name in ARM_LINKS):
                    self.arm.append(prim)
            else:
                self.environment.append(prim)
        if not self.arm or not self.environment:
            raise ValueError('Clearance veto requires arm and environment colliders')
        self.obstacles = [(p, self.bounds(p), self.surface_bounds(p)) for p in self.environment]

    def surface_bounds(self, prim):
        """Disjoint static triangle walls must not become one solid maze box.

        Each polygon's AABB still conservatively contains its entire surface.
        Retain whole bounds for hull/primitive collision approximations.
        """
        from pxr import UsdPhysics
        geom = self.UsdGeom
        if not prim.IsA(geom.Mesh):
            return None
        approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        if approximation not in (None, 'none'):
            return None
        mesh = geom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=int)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
        if not len(counts) or np.any(counts < 3) or counts.sum() != len(indices):
            raise ValueError('Invalid static mesh collision topology: ' + str(prim.GetPath()))
        matrix = np.array(geom.Xformable(prim).ComputeLocalToWorldTransform(0.))
        world = points @ matrix[:3, :3] + matrix[3, :3]
        vertices = world[indices]
        starts = np.r_[0, np.cumsum(counts)[:-1]]
        return np.minimum.reduceat(vertices, starts), np.maximum.reduceat(vertices, starts)

    def bounds(self, prim):
        geom = self.UsdGeom
        cache = geom.BBoxCache(0., [geom.Tokens.default_, geom.Tokens.render,
                                   geom.Tokens.proxy, geom.Tokens.guide])
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            raise ValueError('Empty collision bounds: ' + str(prim.GetPath()))
        return np.array(box.GetMin()), np.array(box.GetMax())

    def check(self):
        geom = self.UsdGeom
        cache = geom.BBoxCache(0., [geom.Tokens.default_, geom.Tokens.render,
                                   geom.Tokens.proxy, geom.Tokens.guide])
        for arm in self.arm:
            box = cache.ComputeWorldBound(arm).ComputeAlignedRange()
            if box.IsEmpty():
                raise ValueError('Empty collision bounds: ' + str(arm.GetPath()))
            lo, hi = np.array(box.GetMin()), np.array(box.GetMax())
            for obstacle, (other_lo, other_hi), surfaces in self.obstacles:
                if np.all(hi + self.margin >= other_lo) and np.all(other_hi + self.margin >= lo):
                    if surfaces is not None:
                        face_lo, face_hi = surfaces
                        near = np.all(hi + self.margin >= face_lo, axis=1) & np.all(face_hi + self.margin >= lo, axis=1)
                        if not near.any():
                            continue
                    raise ValueError(f'CAD clearance <3cm (conservative AABB): {arm.GetPath()} / {obstacle.GetPath()}')
