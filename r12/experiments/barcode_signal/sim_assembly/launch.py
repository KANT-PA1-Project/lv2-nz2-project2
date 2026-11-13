#!/usr/bin/env python3
"""Run one local Isaac child, retaining its real process exit and time budget."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from signal_fallback import wall_budget


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--isaac-python', type=Path, default=Path('~/pa-course/.venv-isaacsim/bin/python').expanduser())
    a = p.parse_args()
    budget_s = wall_budget(json.loads(a.config.expanduser().read_text()))
    out = a.out.expanduser().resolve()
    if out.exists():
        p.error('Output must be a new directory')
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [str(a.isaac_python), '-B', str(Path(__file__).with_name('run.py')),
               '--config', str(a.config.resolve()), '--prepared', str(a.prepared.resolve()), '--out', str(out)]
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    started = time.monotonic()
    with Path(str(out)+'.log').open('xb') as log:
        child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        timed_out = False
        try:
            code = child.wait(timeout=budget_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            child.terminate()
            try:
                code = child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                child.kill()
                code = child.wait()
    record = dict(command=command, pid=child.pid, returncode=code,
                  timed_out=timed_out, wall_s=time.monotonic()-started, wall_budget_s=budget_s)
    passed = False
    if out.exists():
        (out/'process_exit.json').write_text(json.dumps(record, indent=2)+'\n')
        result_path = out/'result.json'
        if not result_path.exists():
            result_path.write_text(json.dumps(dict(status='failed', error='Child exited without final result; no success inferred from exit code'))+'\n')
        if result_path.exists():
            result = json.loads(result_path.read_text())
            result['process_exit_code'] = code
            if code != 0 or timed_out:
                result['status'] = 'failed'
            result_path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
            record['result_status'] = result.get('status')
            passed = result.get('status') == 'passed'
            (out/'process_exit.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record), flush=True)
    raise SystemExit(0 if code == 0 and not timed_out and passed else 1)


if __name__ == '__main__':
    main()
