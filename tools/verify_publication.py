"""Read-only integrity check for this public snapshot; no hardware or networking."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root/'docs/reviews/publication-manifest-20261009.json').read_text(encoding='utf-8'))
    errors = []
    for item in manifest['files']:
        path = (root/item['path']).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            errors.append(item['path'])
            continue
        data = path.read_bytes()
        if item['hash_format'] == 'utf8-lf':
            data = data.replace(b'\r\n', b'\n')
        if hashlib.sha256(data).hexdigest() != item['sha256']:
            errors.append(item['path'])
    # This is the same byte check used by the waiting page; no prepare/confirm.
    checks = json.loads((root/'vision/deployment/boot-test-checks.json').read_text(encoding='utf-8'))
    for item in checks['files']:
        path = (root/item['path']).resolve()
        if not path.is_relative_to(root) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            errors.append('boot checks: '+item['path'])
    print(json.dumps({'files_checked': len(manifest['files']), 'boot_files_checked': len(checks['files']),
                      'hardware_output': False, 'mismatches': errors}, ensure_ascii=False, indent=2))
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
