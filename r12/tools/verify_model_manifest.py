"""Verify downloaded model bytes without loading pickle/torch models."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'MODEL_MANIFEST.json').read_text())
failures = []
for entry in manifest['models']:
    path = root / entry['path']
    if not path.is_file():
        failures.append(entry['path'] + ': missing')
        continue
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if path.stat().st_size != entry['bytes'] or digest != entry['sha256']:
        failures.append(entry['path'] + ': size/hash mismatch (check git lfs pull)')
if failures:
    raise SystemExit('\n'.join(failures))
print(f"Verified {len(manifest['models'])} model/asset/config files; not a performance certification.")
