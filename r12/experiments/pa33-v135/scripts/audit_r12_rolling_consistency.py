"""Offline kinematic consistency; no slip/force estimate or control input."""
import argparse
import json
from pathlib import Path
import numpy as np
from arm_practice.swerve import wheel_odometry, BASE_DIRECTIONS, RADIUS, teacher


def audit(path):
    rows = json.loads((path / 'sim_results.json').read_text())
    samples = []
    for row in rows:
        if not row.get('transport_posture_ok') or row.get('finished'):
            continue
        t = row['drive_telemetry']
        steer = np.asarray(t['steer_position_rad'])
        wheels = np.asarray(t['wheel_target_rad_s'])
        fitted, residual = wheel_odometry(steer, wheels)
        heading = BASE_DIRECTIONS + steer
        vectors = RADIUS * wheels[:, None] * np.column_stack([np.cos(heading), np.sin(heading)])
        desired = teacher(np.asarray(row['command'])).reshape(4, 2)
        samples.append({'frame': row['frame'], 'target_consistency_residual_m_s': residual,
                        'command_vector_mismatch_m_s': float(np.sqrt(np.mean((vectors-desired)**2))),
                        'fitted_target_twist': fitted.tolist(), 'command': row['command']})
    values = np.array([s['target_consistency_residual_m_s'] for s in samples])
    return {'samples': len(samples), 'median_residual_m_s': float(np.median(values)),
            'p95_residual_m_s': float(np.percentile(values, 95)),
            'worst': sorted(samples, key=lambda s: s['target_consistency_residual_m_s'], reverse=True)[:10],
            'scope': 'Logged end-of-window steering and held wheel targets; not synchronized pre-command state. Eight planar rolling-vector components fit one rigid-body twist. Residual assumes no lateral slip, ignores steering scrub/contact forces, and is not a measured slip or causal proof.'}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('runs', nargs='+')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    result = {Path(r).name: audit(Path(r)) for r in a.runs}
    Path(a.output).write_text(json.dumps(result, indent=2))
    print(json.dumps({k: {x:y for x,y in v.items() if x != 'worst'} for k,v in result.items()}, indent=2))
