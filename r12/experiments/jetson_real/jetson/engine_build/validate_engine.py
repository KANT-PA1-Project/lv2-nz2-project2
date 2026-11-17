import json
import sys
import time
from pathlib import Path

import numpy as np
import tensorrt as trt

from jetson_pipeline import Cuda, TrtModel, cp_decode_view, yolo_decode


ROOT = Path(__file__).resolve().parent
model_name, precision = sys.argv[1:3]
metadata = json.loads((ROOT / 'fixtures.json').read_text())
logger = trt.Logger(trt.Logger.WARNING)
cuda = Cuda()
model = TrtModel(trt, logger, cuda, ROOT / f'{model_name}_{precision}.engine')
report = {'model': model_name, 'precision': precision,
          'input_shape': list(model.in_shape), 'outputs': model.outputs,
          'engine_mb': model.file_mb, 'device_memory_mb': model.device_mem_mb,
          'cases': [], 'finite': True}
arrays = np.load(ROOT / 'fixtures.npz')
max_center_difference_mm = 0.0
flag_mismatches = 0
matched_detections = 0
reference_detections = 0
detection_count_mismatches = 0
max_box_difference_px = 0.0
max_score_difference = 0.0
fp32_parity = True
heads = ['hm', 'hps', 'reg', 'scale'] if model_name == 'centerpose' else ['output0']
for fixture in metadata[model_name]:
    index = fixture['index']
    prefix = f'{model_name}_{index}'
    images = arrays[f'{prefix}_input']
    actual = {name: value.copy() for name, value in model(images).items()}
    expected = {name: arrays[f'{prefix}_{name}'] for name in heads}
    row = {'index': index, 'raw_max_abs': {}}
    for name in heads:
        finite = bool(np.isfinite(actual[name]).all())
        report['finite'] = report['finite'] and finite
        row['raw_max_abs'][name] = float(np.max(np.abs(actual[name] - expected[name])))
        fp32_parity = fp32_parity and bool(np.allclose(actual[name], expected[name],
                                                      atol=0.005, rtol=0.001))
    if model_name == 'centerpose':
        intrinsics = np.asarray(fixture['K'], dtype=np.float64)
        case_difference = 0.0
        for view, matrix in enumerate(fixture['matrices']):
            matrix = np.asarray(matrix, dtype=np.float32)
            native_pose = cp_decode_view(expected, view, matrix, intrinsics)
            observed_pose = cp_decode_view(actual, view, matrix, intrinsics)
            difference = float(np.linalg.norm(native_pose['center_camera_m'] -
                                               observed_pose['center_camera_m']) * 1000)
            case_difference = max(case_difference, difference)
            for flag in ('translation_valid', 'rotation_valid', 'valid'):
                flag_mismatches += int(native_pose[flag] != observed_pose[flag])
        row['center_difference_mm'] = case_difference
        max_center_difference_mm = max(max_center_difference_mm, case_difference)
    else:
        height, width = fixture['image_shape'][:2]
        scale = min(800 / height, 800 / width)
        left = int(round((800 - round(width * scale)) / 2 - 0.1))
        top = int(round((800 - round(height * scale)) / 2 - 0.1))
        native = yolo_decode(expected['output0'], .25, .7, scale, left, top, (height, width))
        observed = yolo_decode(actual['output0'], .25, .7, scale, left, top, (height, width))
        reference_detections += len(native)
        detection_count_mismatches += int(len(native) != len(observed))
        available = list(range(len(observed)))
        for detection in native:
            candidates = [item for item in available if int(observed[item][5]) == int(detection[5])]
            if not candidates:
                continue
            nearest = min(candidates, key=lambda item: float(np.linalg.norm(
                np.asarray(observed[item][:4]) - np.asarray(detection[:4]))))
            available.remove(nearest)
            matched_detections += 1
            max_box_difference_px = max(max_box_difference_px, float(np.max(np.abs(
                np.asarray(observed[nearest][:4]) - np.asarray(detection[:4])))))
            max_score_difference = max(max_score_difference,
                                       abs(float(observed[nearest][4]) - float(detection[4])))
        row.update(reference_detections=len(native), observed_detections=len(observed))
    report['cases'].append(row)

for _ in range(10):
    model(images)
timings = []
for _ in range(50):
    start = time.perf_counter()
    model(images)
    timings.append((time.perf_counter() - start) * 1000)
report['latency_including_copies_ms'] = {
    'mean': float(np.mean(timings)), 'median': float(np.median(timings)),
    'p95': float(np.percentile(timings, 95)), 'iterations': len(timings)}
report['fp32_allclose'] = fp32_parity
if model_name == 'centerpose':
    report.update(center_difference_max_mm=max_center_difference_mm,
                  decoder_flag_mismatches=flag_mismatches)
    report['review_passed'] = report['finite'] and flag_mismatches == 0 and max_center_difference_mm <= 1
    report['review_limits'] = {'center_difference_mm': 1, 'decoder_flag_mismatches': 0}
else:
    report.update(reference_detections=reference_detections,
                  matched_detections=matched_detections,
                  detection_count_mismatches=detection_count_mismatches,
                  box_difference_max_px=max_box_difference_px,
                  score_difference_max=max_score_difference)
    report['review_passed'] = (report['finite'] and detection_count_mismatches == 0
                               and matched_detections == reference_detections
                               and max_box_difference_px <= 1 and max_score_difference <= .02)
    report['review_limits'] = {'box_difference_px': 1, 'score_difference': .02,
                              'detection_count_mismatches': 0}
if precision == 'fp32':
    report['review_passed'] = report['review_passed'] and fp32_parity
destination = ROOT / f'{model_name}_{precision}_validation.json'
destination.write_text(json.dumps(report, indent=2))
print(json.dumps({key: value for key, value in report.items() if key != 'cases'}), flush=True)
if not report['finite']:
    raise SystemExit('Nonfinite engine output')
if precision == 'fp32' and not report['review_passed']:
    raise SystemExit('FP32 output parity failed')
