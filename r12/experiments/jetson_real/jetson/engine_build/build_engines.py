import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
status = {'state': 'running', 'stages': []}
environment = dict(os.environ, CUDA_MODULE_LOADING='LAZY', PYTHONDONTWRITEBYTECODE='1')
try:
    manifest = json.loads((ROOT / 'manifest.json').read_text())
    for name, expected in manifest['outputs_sha256'].items():
        observed = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        if observed != expected:
            raise RuntimeError(f'Transfer hash mismatch: {name}')
    for precision in ('fp32', 'fp16'):
        for name in ('yolo', 'centerpose'):
            command = ['/usr/bin/trtexec', f'--onnx={ROOT / (name + ".onnx")}',
                       f'--saveEngine={ROOT / (name + "_" + precision + ".engine")}',
                       '--skipInference', '--memPoolSize=workspace:512M', '--noTF32']
            if precision == 'fp16':
                command.append('--fp16')
            row = {'model': name, 'precision': precision, 'command': command}
            status['current'] = row
            (ROOT / 'build_status.json').write_text(json.dumps(status, indent=2))
            started = time.monotonic()
            with (ROOT / f'{name}_{precision}.build.log').open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                        env=environment, timeout=1200)
            row['build_seconds'] = time.monotonic() - started
            row['build_exit_code'] = result.returncode
            status['stages'].append(row)
            if result.returncode:
                raise RuntimeError(f'Engine build failed: {name} {precision}')
            validation = subprocess.run([sys.executable, '-B', str(ROOT / 'validate_engine.py'),
                                         name, precision], env=environment, timeout=180)
            row['validation_exit_code'] = validation.returncode
            if validation.returncode:
                raise RuntimeError(f'Engine validation failed: {name} {precision}')
    status['state'] = 'complete'
except Exception as error:
    status['state'] = 'failed'
    status['error'] = repr(error)
finally:
    (ROOT / 'build_status.json').write_text(json.dumps(status, indent=2))
print(json.dumps(status), flush=True)
if status['state'] != 'complete':
    raise SystemExit(1)
