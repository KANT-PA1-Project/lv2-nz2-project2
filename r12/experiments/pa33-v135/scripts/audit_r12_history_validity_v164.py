"""Offline history validity audit; never grants motion or clearance authority."""
import json
from pathlib import Path
import numpy as np

root = Path(__file__).resolve().parents[1]
metas = [json.loads(p.read_text()) for p in sorted(
    (root / 'reports/local/observed_history_v159/observation_history').glob('*.json'))]
last = metas[-1]
rows = []
for m in metas:
    b = m['base_sensor_feedback']
    twist = np.asarray(b['twist'], dtype=float)
    cov = np.asarray(m['visual_pose']['covariance'], dtype=float).reshape(6, 6)
    rows.append({
        'frame_index': m['frame_index'],
        'synthetic_age_s': (last['stamp_ns'] - m['stamp_ns']) / 1e9,
        'capture_age_s': (last['base_sensor_feedback']['capture_monotonic_ns'] - b['capture_monotonic_ns']) / 1e9,
        'base_xy_speed_m_s': float(np.linalg.norm(twist[:2])),
        'base_abs_yaw_rate_rad_s': float(abs(twist[2])),
        'base_stationary': bool(b['valid'] and np.isfinite(twist).all() and np.linalg.norm(twist[:2]) < .01 and abs(twist[2]) < .02),
        'position_covariance_diagonal': np.diag(cov)[:3].tolist(),
        'rotation_covariance_diagonal': np.diag(cov)[3:].tolist(),
        'covariance_psd': bool(np.isfinite(cov).all() and np.linalg.eigvalsh((cov+cov.T)/2).min() >= -1e-10),
    })
result = {
    'complete': True, 'frames': len(rows), 'reference_frame': last['frame_index'],
    'oldest_synthetic_age_s': max(r['synthetic_age_s'] for r in rows),
    'oldest_capture_age_s': max(r['capture_age_s'] for r in rows),
    'stationary_archive_frames': sum(r['base_stationary'] for r in rows),
    'reference_state': rows[-1],
    'covariance_provenance': 'r12_sensor_ros.py imposes diagonal floors: position 0.0025 m^2, rotation 0.01 rad^2. Not measured localization error bounds.',
    'relative_pose_uncertainty': 'Unbounded: cross-time covariance/correlation and calibration uncertainty unavailable; do not assume independent poses or use floors as certified margins.',
    'scene_change_validation': 'Unavailable: no temporal occupancy invalidation or dynamic-scene evidence.',
    'execution_authorized': False,
    'next_required': 'Capture fresh stationary final observation and validate relative transforms, scene changes and swept volume before motion. Historical union coverage alone cannot grant clearance.',
    'rows': rows,
}
out = root.parent.parent / 'outputs/r12/v164_history_validity.json'
out.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k:v for k,v in result.items() if k != 'rows'}, indent=2))
