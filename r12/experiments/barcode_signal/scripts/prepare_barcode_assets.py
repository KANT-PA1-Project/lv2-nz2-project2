"""Extract the four supplied photographic Code 128 patterns; never regenerate bars."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np

from barcode_support import decode_rgb

ROOT = Path(__file__).resolve().parents[1]
# Clockwise TL/TR/BR/BL, measured on the supplied 500 x 720 photograph.
QUADS = {
    'P2-01': [[51, 548], [215, 538], [215, 583], [52, 594]],
    'P2-02': [[285, 535], [429, 527], [429, 570], [285, 578]],
    'P2-03': [[53, 628], [215, 616], [215, 658], [54, 671]],
    'P2-04': [[285, 610], [429, 601], [429, 643], [286, 653]],
}


def prepare(source):
    dest = ROOT / 'references/barcodes-photo.png'
    shutil.copyfile(source, dest)
    rgb = cv2.cvtColor(cv2.imread(str(dest)), cv2.COLOR_BGR2RGB)
    rows = []
    target = np.float32([[0, 0], [959, 0], [959, 239], [0, 239]])
    for code, points in QUADS.items():
        matrix = cv2.getPerspectiveTransform(np.float32(points), target)
        bars = cv2.warpPerspective(rgb, matrix, (960, 240), flags=cv2.INTER_CUBIC)
        # 16 texture pixels / mm: bars 60x15 mm; white label 75x25 mm.
        label = np.full((400, 1200, 3), 255, np.uint8)
        label[80:320, 120:1080] = bars
        output = ROOT / 'references' / (code + '.png')
        cv2.imwrite(str(output), cv2.cvtColor(label, cv2.COLOR_RGB2BGR))
        reads = decode_rgb(label)['reads']
        if {r['text'] for r in reads} != {code}:
            raise RuntimeError(f'Photographic texture did not decode as {code}: {reads}')
        rows.append(dict(code=code, quad_source_px=points, homography=matrix.tolist(),
                         texture=output.name, sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                         decoded=reads))
    manifest = dict(source=str(dest.name), source_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                    source_pixels=[500, 720], bars_mm=[60, 15], label_mm=[75, 25],
                    label_size_approximate=True, rotation_on_mount_deg=0,
                    processing='manual quadrilateral; perspective rectification INTER_CUBIC; white margins; no regenerated pattern',
                    limitation='Low resolution photo; rectification and resampling cannot restore lost print detail.',
                    assets=rows)
    (ROOT / 'references/barcodes-manifest.json').write_text(json.dumps(manifest, indent=2))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source), indent=2))
