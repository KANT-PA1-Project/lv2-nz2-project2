"""Full-frame Code 128 decoding and a photographic USD label, with no YOLO."""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'vendor'))
import zxingcpp

ALLOWED = {'P2-01', 'P2-02', 'P2-03', 'P2-04'}
LABEL_M = (0.075, 0.025)
BARS_M = (0.060, 0.015)


def decode_rgb(rgb, upscale=1):
    """Decode the whole camera image. Scene coordinates/expected code are never inputs."""
    data = np.asarray(rgb)
    if data.dtype != np.uint8 or data.ndim != 3 or data.shape[2] != 3:
        raise ValueError('Expected HxWx3 uint8 RGB')
    if upscale not in (1, 2):
        raise ValueError('upscale must be 1 or 2')
    start = time.perf_counter()
    image = np.ascontiguousarray(data[:, :, ::-1])
    found = zxingcpp.read_barcodes(image, formats=zxingcpp.BarcodeFormat.Code128)
    raw_count, scale = len(found), 1
    if not found and upscale == 2:
        import cv2
        scale = 2
        enlarged = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        found = zxingcpp.read_barcodes(enlarged, formats=zxingcpp.BarcodeFormat.Code128)
    reads = []
    for result in found:
        p = result.position
        reads.append(dict(text=result.text, format=str(result.format), allowed=result.text in ALLOWED,
                          corners=[[v.x/scale, v.y/scale] for v in (p.top_left, p.top_right, p.bottom_right, p.bottom_left)]))
    return dict(reads=reads, raw_read_count=raw_count, attempted_upscale=upscale,
                decode_scale=scale, decode_ms=(time.perf_counter() - start) * 1000)


class StableCode:
    """Require three consecutive unambiguous frames; clear on a missing/ambiguous frame."""
    def __init__(self, required=3):
        if required < 1:
            raise ValueError('required must be positive')
        self.required = required
        self.last = None
        self.count = 0

    def observe(self, reads):
        texts = {r['text'] for r in reads}
        code = next(iter(texts)) if len(texts) == 1 and texts <= ALLOWED else None
        self.count = self.count + 1 if code is not None and code == self.last else int(code is not None)
        self.last = code
        return code if self.count >= self.required else None


def label_corners(center):
    """Upright label facing +world X; horizontal width runs along world Y."""
    x, y, z = map(float, center)
    w, h = LABEL_M
    return np.array([[x, y-w/2, z-h/2], [x, y+w/2, z-h/2],
                     [x, y+w/2, z+h/2], [x, y-w/2, z+h/2]])


def projected_bars_px(T_cam, K, center):
    """Truth-only geometric diagnostics; never supplied to the decoder."""
    points = label_corners(center)
    local = (points - np.asarray(center)) * np.array([1., BARS_M[0]/LABEL_M[0], BARS_M[1]/LABEL_M[1]])
    points = local + center
    xyz = (points - T_cam[:3, 3]) @ T_cam[:3, :3]
    if np.any(xyz[:, 2] <= 0):
        return None
    uv = xyz @ K.T
    uv = uv[:, :2] / uv[:, 2:3]
    return dict(corners=uv.tolist(), width_px=float(np.linalg.norm(uv[1]-uv[0])),
                height_px=float(np.linalg.norm(uv[3]-uv[0])))


def add_label(stage, code, center):
    """Create one 75x25mm photographic label; no old gray board is restored."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    if code not in ALLOWED:
        raise ValueError(code)
    asset = ROOT / 'references' / (code + '.png')
    if not asset.is_file():
        raise FileNotFoundError(asset)
    mesh = UsdGeom.Mesh.Define(stage, '/World/TestBarcode')
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in label_corners(center)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    uv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    mat = UsdShade.Material.Define(stage, '/World/Looks/TestBarcode')
    surface = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Surface')
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.)
    surface.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    texture = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Texture')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(asset)))
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    reader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/UV')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    surface.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return dict(code=code, center_world_m=list(center), label_m=list(LABEL_M), bars_m=list(BARS_M),
                mount_rotation_deg=0, texture=str(asset), placement_confirmed=False,
                manifest=json.loads((ROOT / 'references/barcodes-manifest.json').read_text()))
