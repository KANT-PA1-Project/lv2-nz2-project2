"""Read saved monotonic timing samples; no observations or control writes."""
import argparse
import collections
import json
import statistics
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
args = parser.parse_args()
metrics = collections.defaultdict(list)


def interval(name, row, start, end):
    if start in row and end in row and row[end] >= row[start]:
        metrics[name].append((row[end] - row[start]) / 1e6)


rows = json.loads((args.run / 'sim_results.json').read_text())
for row in rows:
    stages = row.get('acquisition_stages_monotonic_ns', {})
    for name, start, end in [
        ('command_setup', 'frame_loop_start', 'physics_start'),
        ('physics_and_imu', 'physics_start', 'capture_start'),
        ('render', 'capture_start', 'render_done'),
        ('readback_and_mask', 'render_done', 'readback_mask_done'),
        ('frame_npz_write', 'readback_mask_done', 'npz_done'),
        ('perception_and_packet_build', 'npz_done', 'packet_ready'),
    ]:
        interval(name, stages, start, end)
    interval('ros_response_roundtrip', {**stages, **row}, 'packet_ready', 'sim_response_received_monotonic_ns')
    interval('ros_image_block', row.get('publication_stages_monotonic_ns', {}), 'image_block_start', 'image_block_done')
    if 'visual_pair_wait' in row:
        metrics['visual_pair_wait'].append(row['visual_pair_wait']['wait_seconds'] * 1000)
    operations = row.get('publication_operations', [])
    for key in ['to_bytes_ns', 'message_assignment_ns', 'publish_ns']:
        if operations:
            metrics['image_' + key.removesuffix('_ns')].append(sum(x[key] for x in operations) / 1e6)

for line in (args.run / 'perception.jsonl').read_text().splitlines():
    try:
        row = json.loads(line)
    except json.JSONDecodeError:
        continue  # A live writer may not have finished its last line.
    stages = row.get('stage_times_ns', {})
    interval('worker_frame_load', {**row, **stages}, 'request_received_monotonic_ns', 'frame_loaded')
    for name, start, end in [('barcode', 'barcode_start', 'barcode_done'), ('tag_decode', 'tags_start', 'tags_done'), ('detector', 'detector_start', 'detector_done')]:
        interval(name, stages, start, end)
    if 'pose_calls_total_ns' in stages:
        metrics['pose_calls_per_detection_frame'].append(stages['pose_calls_total_ns'] / 1e6)

summary = {}
for name, values in metrics.items():
    ordered = sorted(values)
    summary[name] = dict(n=len(values), mean=statistics.mean(values), median=statistics.median(values), p95=ordered[int((len(values)-1)*.95)], max=max(values))
report = dict(frames=len(rows), units='milliseconds', metrics=summary, scope='Host wall-clock intervals. Worker/ROS component metrics are nested, not additive to enclosing stages. Physics includes IMU reads; render includes synchronous render wait. No hardware latency or mission acceptance claim.')
target = args.run / 'timing_summary.json'
temporary = target.with_suffix('.tmp')
temporary.write_text(json.dumps(report, indent=2) + '\n')
temporary.replace(target)
print(json.dumps(report, indent=2))
