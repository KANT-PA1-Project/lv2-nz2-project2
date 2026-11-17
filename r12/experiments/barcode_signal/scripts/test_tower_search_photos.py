"""Offline acquisition diagnostic on supplied stills; no fabricated OFF reference.

Only visually labelled OFF images may be provisionally confirmed for this test.
Lit images test the refusal contract, not initial acquisition on lit towers.
Manual ROI shape is given as a coarse prior; this is not full-frame detection.
"""
import argparse
import hashlib
import importlib.util
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from test_tower_signal_photos import lens_rois_from_axis

ROOT=Path(__file__).resolve().parents[1]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(out):
    out=Path(out)
    out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'src/arm_practice/tower_signal_search.py'
    new=load('search_new',source)
    old=load('search_old',ROOT/'reports/20261110-body-search-v2/baseline_search.py')
    manifest=json.loads((ROOT/'reports/20261110-real-validation61/samples.json').read_text())
    annotations=json.loads((ROOT/'references/tower-photo-body-axis-validation61.json').read_text())
    rows=[]
    protocols=[]
    shifts=[(0,0),(10,-10),(-20,20),(30,-30),(-40,40)]
    for index,sample in enumerate(manifest['samples']):
        path=Path(sample['image'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=sample['image_sha256']:
            raise ValueError('Changed source image')
        rgb=np.asarray(Image.open(path).convert('RGB'))
        if sample['truth']!='OFF':
            result=new.locate_body(rgb,None,None,confirmed_off=False)
            protocols.append(dict(id=index,truth=sample['truth'],status='unconfirmed_lit_acquisition_refused',reason=result['reason']))
            continue
        key=str(index)
        if key not in annotations['points']:
            protocols.append(dict(id=index,status='manual_shape_unavailable'))
            continue
        if annotations['image_sha256'][key]!=sample['image_sha256']:
            raise ValueError('ROI/image mismatch')
        boxes,polygons=lens_rois_from_axis(sample,annotations['points'][key])
        centers=np.array([np.mean(polygons[c],axis=0) for c in 'RYG'])
        tolerance=max(2.,min(np.linalg.norm(np.diff(centers,axis=0),axis=1))*.25)
        protocols.append(dict(id=index,truth='OFF',status='provisional_visual_off_used_for_acquisition_only'))
        for dx,dy in shifts:
            coarse={c:(np.asarray(a)+[dx,dy]).tolist() for c,a in polygons.items()}
            b={c:(v[0]+dx,v[1]+dy,v[2]+dx,v[3]+dy,v[4]) for c,v in boxes.items()}
            for name,module in [('previous',old),('boundary_fixed_size',new)]:
                result=module.locate_body(rgb,b,coarse,confirmed_off=True)
                error=None
                if result['found']:
                    aligned=np.array([np.mean(result['polygons'][c],axis=0) for c in 'RYG'])
                    error=float(np.max(np.linalg.norm(aligned-centers,axis=1)))
                rows.append(dict(id=index,method=name,input_shift=[dx,dy],found=result['found'],reason=result['reason'],error_px=error,tolerance_px=float(tolerance),recovered=bool(error is not None and error<=tolerance),registration=result))
                if name=='boundary_fixed_size' and (dx,dy)==(-40,40):
                    im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
                    for c,a in result.get('polygons',{}).items():
                        pts=[tuple(v) for v in a];draw.line(pts+[pts[0]],fill={'R':'red','Y':'yellow','G':'lime'}[c],width=2)
                    draw.text((10,10),f'{index} found={result["found"]} error={error}',fill='magenta')
                    im.save(out/f'photo_{index:03d}_offset40.jpg')
    methods={}
    for name in ['previous','boundary_fixed_size']:
        rr=[r for r in rows if r['method']==name]
        methods[name]=dict(cases=len(rr),found=sum(r['found'] for r in rr),recovered=sum(r['recovered'] for r in rr),found_but_outside_tolerance=sum(r['found'] and not r['recovered'] for r in rr),reasons=dict(Counter(r['reason'] or 'found' for r in rr)))
    summary=dict(images=61,provisional_off_images=sum(r['status']=='provisional_visual_off_used_for_acquisition_only' for r in protocols),lit_images_refused=sum(r['status']=='unconfirmed_lit_acquisition_refused' for r in protocols),offsets=shifts,methods=methods,off_reference_real_evaluated=0,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scope='Still-image diagnostic with provisional visual OFF labels and manually supplied body geometry, translation perturbation only; no automatic OFF or full-frame detection accuracy.',limitations=['45 lit images are protocol rejection tests, not successful acquisition attempts.','No three-frame matched OFF baseline or real timed sequence was fabricated.','Manual geometry is approximate and tolerance is a diagnostic 0.25 tier pitch, minimum2px.','These images may become development evidence; parameters were frozen before this run.'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    (out/'protocols.json').write_text(json.dumps(protocols,indent=2,ensure_ascii=False))
    (out/'predictions.jsonl').write_text(''.join(json.dumps(r)+chr(10) for r in rows))
    print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    run(ap.parse_args().out)
