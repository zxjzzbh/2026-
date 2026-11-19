"""Verify the published release bytes without connecting to vehicle hardware."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'SOURCE-MANIFEST.json').read_text(encoding='utf-8'))
expected = {entry['path']: entry for entry in manifest['files']}
errors = []
for relative, entry in expected.items():
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        errors.append(relative + ': missing')
        continue
    raw = path.read_bytes()
    if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']:
        errors.append(relative + ': bytes differ')
for directory in ('cpp', 'vision', 'deployment'):
    actual = {
        p.relative_to(root).as_posix()
        for p in (root / directory).rglob('*')
        if p.is_file() and not any(part in ('build', '__pycache__') for part in p.relative_to(root / directory).parts)
    }
    wanted = {p for p in expected if p.startswith(directory + '/')}
    errors.extend(sorted(actual ^ wanted))
if len(expected) != manifest['file_count']:
    errors.append('manifest file count mismatch')
print(json.dumps({'version': manifest['version'], 'files': len(expected), 'errors': errors}, ensure_ascii=True))
raise SystemExit(bool(errors))
