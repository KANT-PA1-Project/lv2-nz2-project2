"""Cross-check authored robot configuration before starting a simulator."""
import argparse
import json
from pathlib import Path

from .config import load_robot_config
from .urdf import inspect_urdf


def validate_robot(urdf_path, config_path, require_usd=False):
    """Validate independent actuators and limits; preserve configured action order.

    Conservative position, effort and velocity limits are allowed. XML joint
    declaration order does not define the policy action order.
    """
    config = load_robot_config(config_path)
    model = inspect_urdf(urdf_path)
    joints = {j['name']: j for j in model['movable_joints']}
    independent = {name for name, j in joints.items() if not j['mimic']}
    configured = set(config['joint_names'])
    errors = []
    if configured != independent:
        errors.append(f"Independent joints differ: missing={sorted(independent - configured)}, "
                      f"unexpected={sorted(configured - independent)}")
    if config['tcp_link'] not in model['links']:
        errors.append(f"TCP link not found: {config['tcp_link']}")
    for i, name in enumerate(config['joint_names']):
        if name not in joints:
            continue
        joint = joints[name]
        if joint['type'] != 'continuous':
            if config['lower'][i] < joint['lower'] or config['upper'][i] > joint['upper']:
                errors.append(f"{name}: configured position limits exceed URDF limits")
        for key in ('effort', 'velocity'):
            if config[key][i] > joint[key]:
                errors.append(f"{name}: configured {key} exceeds URDF limit")
    usd_exists = Path(config['usd_path']).is_file()
    if require_usd and not usd_exists:
        errors.append(f"USD file not found: {config['usd_path']}")
    warnings = list(model['warnings'])
    if not usd_exists and not require_usd:
        warnings.append('USD file is absent; convert the URDF before training')
    return {'valid': not errors, 'errors': errors, 'warnings': warnings,
            'joint_order': config['joint_names'], 'tcp_link': config['tcp_link'],
            'usd_path': config['usd_path'], 'usd_exists': usd_exists}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('config', type=Path)
    parser.add_argument('--require-usd', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        report = validate_robot(args.urdf, args.config, args.require_usd)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        report = {'valid': False, 'errors': [str(exc)], 'warnings': []}
    result = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + '\n', encoding='utf-8')
    print(result)
    return 0 if report['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
