#!/usr/bin/env python3
"""최종 조립 URDF 를 시뮬용으로 정리해 새 폴더에 저장한다 (Isaac 불필요).

원본 ~/Desktop/assembly_puck_slots./ 는 읽기만 한다. 결과 폴더는 새로 만들어야 한다.
"""
import argparse
import json
from pathlib import Path

from common import ARM_JOINTS, CAMERA_FRAME, WRIST_LINK, arm_limits, load_config, sanitize_urdf, sha256


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    ap.add_argument('--out', required=True, type=Path, help='새 준비 폴더')
    a = ap.parse_args()
    cfg, model = load_config(a.config)
    out = a.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    source = Path(cfg['assembly_dir']) / 'assembly.urdf'
    lower, upper = arm_limits(cfg, model)
    manifest = sanitize_urdf(source, out / 'assembly_sim.urdf', (lower, upper))
    manifest.update(
        config=cfg, status='prepared_not_isaac_validated',
        sensors_json_sha256=sha256(Path(cfg['assembly_dir']) / 'sensors.json'),
        wrist_camera=dict(parent=WRIST_LINK, frame=CAMERA_FRAME,
                          parent_to_optical=model.static_frame(WRIST_LINK, CAMERA_FRAME).tolist()),
        mast_camera=dict(parent='base_link', frame='mast_d435_color_optical_frame',
                         pitch_down_deg=cfg.get('mast_pitch_down_deg') or 60.0,
                         parent_to_optical=model.mast_to_optical(cfg.get('mast_pitch_down_deg')).tolist()),
        arm_joints=ARM_JOINTS, arm_lower=lower.tolist(), arm_upper=upper.tolist(),
        gravity_torque_nm=dict(
            barcode=model.gravity_torque(cfg['barcode_arm_q_rad'], cfg['barcode_base_xyz_yaw']),
            tower=model.gravity_torque(cfg['tower_arm_waypoints_rad'][-1], cfg['tower_base_xyz_yaw'])))
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print(out / 'assembly_sim.urdf')


if __name__ == '__main__':
    main()
