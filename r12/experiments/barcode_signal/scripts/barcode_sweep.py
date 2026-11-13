"""Upright label sweep in the existing maze; hypothetical mounting positions."""
import json
import time

import numpy as np
from PIL import Image, ImageDraw

from barcode_support import ALLOWED, StableCode, add_label, decode_rgb, projected_bars_px

# Label remains level in world coordinates; only its position and camera view change.
PLACEMENTS = [
    dict(name='low_front', center=[.15, 1.03, .14], camera_offset=[0., 0.03]),
    dict(name='left_higher', center=[.15, .82, .25], camera_offset=[.05, .10]),
    dict(name='right_higher', center=[.15, 1.15, .30], camera_offset=[-.08, .20]),
]
DISTANCES_M = [.20, .30, .40, .60]


def wait_for_render(render, minimum=4, maximum=32):
    """Wait for nonuniform RGB before static-label scoring; never count black buffers."""
    for count in range(1, maximum+1):
        rgb, _ = render()
        if count >= minimum and np.max(rgb) > np.min(rgb):
            return count
    raise RuntimeError(f'No nonuniform RGB after {maximum} warmup renders; invalid barcode condition')


def run_sweep(stage, render, move_camera, look_at, K, out, note, upscale=1):
    # Initialize the render product before the first scored camera transform.
    # Initial captures are diagnostics, not labeled/scored barcode frames.
    initial_center = np.array(PLACEMENTS[0]['center'])
    add_label(stage, 'P2-01', initial_center)
    move_camera(look_at(initial_center + [.30, 0., .03], initial_center))
    prime_uniform = 0
    for _ in range(4):
        initial_rgb, _ = render()
        prime_uniform += int(np.max(initial_rgb) <= np.min(initial_rgb))
    note(f'Render-product initialization: 4 unscored captures, uniform={prime_uniform}')
    rows = []
    trace = (out / 'barcode-sweep-frames.jsonl').open('w')
    try:
        for code in sorted(ALLOWED):
            for placement in PLACEMENTS:
                center = np.array(placement['center'])
                add_label(stage, code, center)
                for distance in DISTANCES_M:
                    dy, dz = placement['camera_offset']
                    T = look_at(center + [distance, dy, dz], center)
                    move_camera(T)
                    # Warm renderer after camera/texture changes; no scored images are shifted.
                    warmup_renders = wait_for_render(render)
                    tracker, samples = StableCode(), []
                    rgb = None
                    for frame in range(8):
                        rgb, _ = render()
                        if np.max(rgb) <= np.min(rgb):
                            raise RuntimeError(f'Uniform RGB during scoring: {code} {placement["name"]} {distance}; invalid run')
                        found = decode_rgb(rgb, upscale=upscale)
                        found.update(frame=frame, stable=tracker.observe(found['reads']))
                        samples.append(found)
                        trace.write(json.dumps(dict(code=code, placement=placement['name'], distance_m=distance, **found)) + '\n')
                    correct = sum({r['text'] for r in sample['reads']} == {code} for sample in samples)
                    wrong = sum(any(r['text'] != code for r in sample['reads']) for sample in samples)
                    stable = sum(sample['stable'] == code for sample in samples)
                    name = f"barcode_{code}_{placement['name']}_{distance:.2f}m.png"
                    im = Image.fromarray(rgb)
                    draw = ImageDraw.Draw(im)
                    for read in samples[-1]['reads']:
                        draw.line([tuple(p) for p in read['corners']] + [tuple(read['corners'][0])], fill=(255, 0, 255), width=2)
                    im.save(out / name)
                    row = dict(code=code, placement=placement, distance_x_m=distance,
                               optical_range_m=float(np.linalg.norm(T[:3, 3] - center)),
                               camera_world=T.tolist(), bars_projection=projected_bars_px(T, K, center),
                               frames=len(samples), warmup_renders=warmup_renders, rgb_content_checked=True, upscale=upscale,
                               raw_read_frames=sum(s['raw_read_count'] > 0 for s in samples), correct_frames=correct, wrong_frames=wrong,
                               stable_correct_frames=stable, passed=correct == len(samples) and wrong == 0 and stable > 0,
                               decode_ms_mean=float(np.mean([s['decode_ms'] for s in samples])), image=name)
                    rows.append(row)
                    note(f"Barcode {code} {placement['name']} x-distance={distance:.2f}m: {correct}/8 correct, wrong={wrong}, stable={stable}")
                    summary = dict(rule_status='바코드 해독 렌더 시험; 실물 검증 전',
                                   signal_rule_status='후보 규칙, 실사 정답 영상 검증 전',
                                   scope='Photographic resampling and idealized maze rendering; not physical-camera performance or verified arm reach.',
                                   resolution=[640, 480], K=K.tolist(), bars_mm=[60, 15], label_mm=[75, 25],
                                   label_mount_rotation_deg=0, placement_confirmed=False,
                                   camera_pose_assumptions='Fixed robot origin; hypothetical camera positions; arm reach not checked.',
                                   initial_unscored_captures=4, initial_uniform_captures=prime_uniform, stabilization_renders_min=4, stabilization_renders_max=32, rgb_content_checked=True, frames_per_case=8,
                                   conditions=len(rows), passed=sum(r['passed'] for r in rows),
                                   wrong_frames=sum(r['wrong_frames'] for r in rows), results=rows)
                    (out / 'barcode-sweep-summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    finally:
        trace.close()
    return rows
