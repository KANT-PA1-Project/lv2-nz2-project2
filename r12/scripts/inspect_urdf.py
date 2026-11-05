#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from arm_practice.urdf import inspect_urdf

parser = argparse.ArgumentParser(description="URDF tree and joint-limit preflight")
parser.add_argument("urdf", type=Path)
parser.add_argument("--output", type=Path)
args = parser.parse_args()
result = json.dumps(inspect_urdf(args.urdf), ensure_ascii=False, indent=2)
if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(result + "\n", encoding="utf-8")
print(result)
