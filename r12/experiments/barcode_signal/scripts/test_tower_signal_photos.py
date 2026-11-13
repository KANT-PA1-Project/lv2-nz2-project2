"""Seeded real-photo diagnostic using existing location labels, never YOLO.

Provisional colors are visually reviewed by the assistant before exposing predictions. Static photo
replay has no real capture timing and is not a signal sequence/video validation.
"""
import argparse
import hashlib
import importlib.util
import json
import random
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageOps

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT.parents[1] / 'data/real/review'


def load_signal():
    path = ROOT / 'src/arm_practice/tower_signal.py'
    spec = importlib.util.spec_from_file_location('photo_signal', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare(out, seed=33, per_group=4, exclude_samples=None, min_frame_gap=0):
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    groups, inventory = {}, Counter()
    for session in ['1025_tower', '1026_tower']:
        for annotation in sorted((DATA / session).glob('*.json')):
            data = json.loads(annotation.read_text())
            boxes = [s for s in data.get('shapes', []) if s['label'] == 'tower_light']
            inventory[session + ':tower_boxes'] += len(boxes)
            for index, shape in enumerate(boxes):
                image = annotation.with_name(Path(data['imagePath']).name)
                if not image.is_file():
                    inventory['missing_images'] += 1
                    continue
                points = np.asarray(shape['points'], float)
                bbox = [float(points[:, 0].min()), float(points[:, 1].min()),
                        float(points[:, 0].max()), float(points[:, 1].max())]
                group = session + '/' + annotation.stem.split('__')[0]
                groups.setdefault(group, []).append(dict(image=str(image), annotation=str(annotation),
                    bbox=bbox, shape_index=index, group=group))
    if min_frame_gap < 0:
        raise ValueError('min_frame_gap must be nonnegative')
    excluded = json.loads(Path(exclude_samples).read_text())['samples'] if exclude_samples else []
    excluded_hashes = {r['image_sha256'] for r in excluded}
    excluded_paths = {str(Path(r['image']).resolve()) for r in excluded}
    def index_of(row):
        return int(Path(row['image']).stem.rsplit('_',1)[-1])
    rng = random.Random(seed)
    selected = []
    selected_hashes = set()
    selected_group_counts = {}
    for group, rows in sorted(groups.items()):
        if not excluded and not min_frame_gap:
            picks = rng.sample(rows, min(per_group, len(rows)))
        else:
            blocked = [index_of(r) for r in excluded if r['group']==group]
            pool = [r for r in rows if str(Path(r['image']).resolve()) not in excluded_paths and
                    all(abs(index_of(r)-i)>=min_frame_gap for i in blocked)]
            rng.shuffle(pool)
            picks = []
            for r in pool:
                if any(abs(index_of(r)-index_of(p))<min_frame_gap for p in picks):
                    continue
                digest = hashlib.sha256(Path(r['image']).read_bytes()).hexdigest()
                if digest in excluded_hashes or digest in selected_hashes:
                    continue
                picks.append(r)
                selected_hashes.add(digest)
                if len(picks)==per_group:
                    break
        selected.extend(picks)
        selected_group_counts[group]=len(picks)
    rng.shuffle(selected)
    for index, row in enumerate(selected):
        row.update(id=f'photo_{index:03d}', truth=None, truth_source='pending_visual_review',
                   truth_note='', lens_bbox=None, annotation_valid=None, three_tiers_visible=None,
                   image_sha256=hashlib.sha256(Path(row['image']).read_bytes()).hexdigest())
    manifest = dict(seed=seed, per_group=per_group, inventory=dict(inventory),
                    excluded_samples=str(exclude_samples) if exclude_samples else None,
                    min_capture_index_gap=min_frame_gap, selected_group_counts=selected_group_counts,
                    frozen_signal_sha256=hashlib.sha256((ROOT/'src/arm_practice/tower_signal.py').read_bytes()).hexdigest(),
                    group_sizes={g:len(rows) for g,rows in sorted(groups.items())},
                    scope='Diagnostic with provisional assistant-reviewed still-image colors, existing label boxes split in thirds; not user-confirmed truth, detector, camera projection, timing or independent holdout validation.',
                    rule_status='후보 규칙, 실사 정답 영상 검증 전', samples=selected)
    (out / 'samples.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    contact_sheets(out, selected)
    return manifest


def contact_sheets(out, rows):
    for page, offset in enumerate(range(0, len(rows), 16)):
        sheet = Image.new('RGB', (1200, 1280), '#dddddd')
        draw = ImageDraw.Draw(sheet)
        for j, row in enumerate(rows[offset:offset+16]):
            im = Image.open(row['image']).convert('RGB')
            x0,y0,x1,y1 = row['bbox']
            w,h = x1-x0,y1-y0
            crop_box = (max(0,int(x0-w*.6)),max(0,int(y0-h*.25)),
                        min(im.width,int(x1+w*.6)),min(im.height,int(y1+h*.25)))
            crop = im.crop(crop_box)
            fitted = ImageOps.contain(crop,(280,260),Image.Resampling.NEAREST)
            x,y=(j%4)*300,(j//4)*320
            sheet.paste(fitted,(x+10,y+42))
            draw.text((x+10,y+5),row['id'],fill='black')
            draw.text((x+10,y+20),row['group'].split('/')[-1][:30],fill='black')
            # Record crop transform for independent manual lens-boundary review.
            row['contact_crop'] = list(crop_box)
            row['contact_scale'] = fitted.width/crop.width
            row['contact_page'] = page
        sheet.save(out/f'contact_{page:02d}.png')
    # Include coordinate transforms, but do not store any candidate predictions here.
    manifest=json.loads((out/'samples.json').read_text())
    manifest['samples']=rows
    (out/'samples.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False))


def tier_boxes_from_bbox(bbox, inner=.6):
    x0,y0,x1,y1=map(float,bbox)
    w,h=x1-x0,(y1-y0)/3
    boxes={}
    for index,c in enumerate('RYG'):
        a,b=y0+index*h,y0+(index+1)*h
        dx,dy=w*(1-inner)/2,h*(1-inner)/2
        boxes[c]=(int(round(x0+dx)),int(round(a+dy)),int(round(x1-dx)),int(round(b-dy)),0.)
    return boxes


def lens_rois_from_axis(sample, local_axis, inner=.6, source_axis=None):
    """Map visually fitted body endpoints to three tilted interior polygons."""
    top=np.array(local_axis[:2],float)-[10,42]
    bottom=np.array(local_axis[2:4],float)-[10,42]
    scale=sample['contact_scale']
    origin=np.array(sample['contact_crop'][:2])
    top,bottom=top/scale+origin,bottom/scale+origin
    wt,wb=np.array(local_axis[4:6],float)/scale
    if source_axis is not None:
        top,bottom=np.array(source_axis[:2],float),np.array(source_axis[2:4],float)
        wt,wb=source_axis[4:6]
    axis=bottom-top
    normal=np.array([axis[1],-axis[0]])/np.linalg.norm(axis)
    boxes,polygons={},{}
    for i,c in enumerate('RYG'):
        a=(i+(1-inner)/2)/3
        b=(i+1-(1-inner)/2)/3
        p,q=top+axis*a,top+axis*b
        wa=(wt+(wb-wt)*a)*inner/2
        wb_=(wt+(wb-wt)*b)*inner/2
        points=np.array([p-normal*wa,p+normal*wa,q+normal*wb_,q-normal*wb_])
        x0,y0=np.floor(points.min(axis=0)).astype(int)
        x1,y1=np.ceil(points.max(axis=0)).astype(int)
        boxes[c]=(int(x0),int(y0),int(x1),int(y1),0.)
        polygons[c]=points.tolist()
    return boxes,polygons


def _photo_summary(rows, method):
    eligible=[r for r in rows if r['truth'] in ['R','Y','G','OFF'] and r['annotation_valid'] is True and r['three_tiers_visible'] is True]
    confusion=Counter(r['truth']+'->'+r['methods'][method]['color'] for r in eligible)
    return dict(evaluated=len(eligible),confusion=dict(confusion),
                correct=sum(r['truth']==r['methods'][method]['color'] for r in eligible),
                unknown=sum(r['methods'][method]['color']=='UNKNOWN' for r in eligible),
                wrong_color=sum(r['methods'][method]['color'] not in [r['truth'],'UNKNOWN'] for r in eligible))


def evaluate(out, roi_path=None):
    module=load_signal()
    manifest=json.loads((out/'samples.json').read_text())
    frozen=manifest.get('frozen_signal_sha256')
    if frozen and frozen!=hashlib.sha256((ROOT/'src/arm_practice/tower_signal.py').read_bytes()).hexdigest():
        raise ValueError('Frozen signal rule changed after sample selection')
    roi_path=Path(roi_path) if roi_path is not None else ROOT/'references/tower-photo-body-axis-seed33.json'
    fitted=json.loads(roi_path.read_text())
    old_params=dict(module.PARAMS,white_unique_enabled=False)
    rows=[]
    for sample in manifest['samples']:
        im=np.asarray(Image.open(sample['image']).convert('RGB'))
        if hashlib.sha256(Path(sample['image']).read_bytes()).hexdigest()!=sample['image_sha256']:
            raise RuntimeError('Input photo changed: '+sample['id'])
        old_boxes=tier_boxes_from_bbox(sample['bbox'],module.PARAMS['inner'])
        invalid=dict(color='UNKNOWN',reason=sample.get('roi_unavailable_reason','invalid_annotation'),ratios={},boxes={})
        methods={'old_box_old_rule':(invalid if sample['annotation_valid'] is False else module.color_from_boxes(im,old_boxes,old_params))}
        index=str(int(sample['id'].split('_')[-1]))
        if index in fitted['points'] and fitted.get('image_sha256',{}).get(index)!=sample['image_sha256']:
            raise ValueError('Lens ROI annotation belongs to another image: '+sample['id'])
        polygons=None
        if index in fitted['points'] and sample.get('three_tiers_visible') is True:
            boxes,polygons=lens_rois_from_axis(sample,fitted['points'][index],module.PARAMS['inner'],source_axis=fitted.get('source_coordinate_overrides',{}).get(index))
            methods['lens_roi_old_rule']=module.color_from_boxes(im,boxes,old_params,roi_polygons=polygons)
            methods['lens_roi_new_rule']=module.color_from_boxes(im,boxes,roi_polygons=polygons)
        else:
            boxes=old_boxes
            invalid=dict(color='UNKNOWN',reason=sample.get('roi_unavailable_reason','three_tiers_not_visible'),ratios={},boxes={})
            methods['lens_roi_old_rule']=invalid
            methods['lens_roi_new_rule']=invalid
        row=dict(sample_id=sample['id'],group=sample['group'],truth=sample['truth'],
                 truth_source=sample['truth_source'],annotation_valid=sample['annotation_valid'],
                 three_tiers_visible=sample.get('three_tiers_visible'),methods=methods,
                 roi_annotation_confidence=sample.get('roi_annotation_confidence','visually_fitted'),
                 off_reference_available=False,
                 off_reference_reason='No confirmed OFF frames with identical pose/exposure; no fabricated pairing')
        rows.append(row)
        picture=Image.fromarray(im)
        draw=ImageDraw.Draw(picture)
        for c,points in (polygons or {}).items():
            draw.line([tuple(p) for p in points]+[tuple(points[0])],fill={'R':'red','Y':'yellow','G':'lime'}[c],width=2)
        res=methods['lens_roi_new_rule']
        draw.text((10,10),sample['id']+' truth='+str(sample['truth'])+' pred='+res['color'],fill='magenta',stroke_width=1,stroke_fill='white')
        picture.save(out/(sample['id']+'_prediction.jpg'))
    summary=dict(scope='Development-photo diagnostic with assistant provisional truth and visually fitted lens ROIs; not held-out or user-confirmed accuracy.',
                 rule_status='후보 규칙, 실사 정답 영상 검증 전',seed=manifest['seed'],params=module.PARAMS,
                 images=len(rows),truth_review=manifest.get('truth_review'),
                 frozen_signal_sha256=manifest.get('frozen_signal_sha256'),
                 excluded_samples=manifest.get('excluded_samples'),
                 overlap_previous_sha256=manifest.get('overlap_previous_sha256'),
                 invalid_or_ambiguous_samples=[dict(id=r['sample_id'],truth=r['truth'],reason=r['methods']['lens_roi_new_rule']['reason']) for r in rows if r['truth'] is None or r['three_tiers_visible'] is not True or r['annotation_valid'] is not True],
                 low_confidence_roi=[r['sample_id'] for r in rows if r['roi_annotation_confidence']=='low_due_to_bloom'],
                 methods={name:_photo_summary(rows,name) for name in ['old_box_old_rule','lens_roi_old_rule','lens_roi_new_rule']},
                 off_reference_real_evaluated=0,
                 off_reference_status='Implemented; no pose/exposure-matched confirmed OFF captures available for these photos',
                 limitations=manifest.get('limitations',[])+['Manual body ROIs are diagnostic annotations, not an automatic detector or a solved camera calibration.'],
                 signal_source_sha256=hashlib.sha256((ROOT/'src/arm_practice/tower_signal.py').read_bytes()).hexdigest(),
                 manifest_sha256=hashlib.sha256((out/'samples.json').read_bytes()).hexdigest(),
                 roi_source_sha256=hashlib.sha256(roi_path.read_bytes()).hexdigest())
    (out/'predictions.jsonl').write_text(''.join(json.dumps(row,ensure_ascii=False)+'\n' for row in rows))
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    print(json.dumps(summary,indent=2,ensure_ascii=False))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--prepare',action='store_true')
    parser.add_argument('--roi-annotations',type=Path,help='Image-hash-bound lens body annotations; default is the reviewed seed33 set')
    parser.add_argument('--seed',type=int,default=33)
    parser.add_argument('--per-group',type=int,default=4)
    parser.add_argument('--exclude-samples',type=Path,help='Exclude previous image paths/SHA256 and nearby capture indices')
    parser.add_argument('--min-frame-gap',type=int,default=0,help='Minimum capture-index gap within a group; not a time unit')
    args=parser.parse_args()
    if args.prepare:
        prepare(args.out,args.seed,args.per_group,args.exclude_samples,args.min_frame_gap)
    else:
        evaluate(args.out,args.roi_annotations)
