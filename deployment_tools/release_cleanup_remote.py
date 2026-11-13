"""Scoped deployment cleanup. Executed over SSH; never starts vehicle services."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import tarfile


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def checked_record(root, record):
    root, record = Path(root), Path(record)
    if (not root.is_absolute() or root.resolve() != root or record.resolve() != record
            or record.parent != root / 'run'
            or not re.fullmatch(r'[a-z0-9][a-z0-9-]*-[0-9]{10,20}', record.name)
            or not record.is_dir()):
        raise ValueError('Cleanup must use one canonical deployment record below live/run')
    return root, record


def snapshot(root, record):
    _, record = checked_record(root, record)
    # Only artifacts created by the deployment script are eligible. Logs,
    # calibration, configuration and other run folders are never selected.
    targets = sorted(p for p in record.iterdir() if p.name in ('candidate', 'candidate.tar.gz')
                     or p.name.endswith(('.before', '.field-runtime-tmp')))
    entries = {}
    for target in targets:
        paths = [target]
        if target.is_dir() and not target.is_symlink():
            for parent, dirs, files in os.walk(target, followlinks=False):
                paths.extend(Path(parent) / name for name in dirs + files)
        for path in paths:
            info = path.lstat()
            if path.resolve() != path or not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode)):
                raise ValueError('Cleanup refuses links and special files: ' + str(path))
            row = {'kind': 'dir' if path.is_dir() else 'file', 'mode': stat.S_IMODE(info.st_mode)}
            if row['kind'] == 'file':
                row.update(size=info.st_size, sha256=sha(path))
            entries[path.relative_to(record).as_posix()] = row
    return {'targets': [p.name for p in targets], 'entries': entries}


def source_hashes(root):
    result = {}
    for sub in ('vision/src', 'vision/web'):
        for path in (root / sub).rglob('*'):
            if path.is_file() and path.suffix not in ('.pyc', '.pyo'):
                if path.resolve() != path:
                    raise ValueError('Source link is not allowed: ' + str(path))
                result[path.relative_to(root).as_posix()] = sha(path)
    return result


def verify_source(root, expected):
    if not expected or source_hashes(root) != expected:
        raise ValueError('Installed source file set or hashes differ from the reviewed release')


def code_roots(search_roots):
    roots = []
    for base in search_roots:
        for parent, dirs, _ in os.walk(base, followlinks=False):
            dirs[:] = [n for n in dirs if n not in ('.venv', '.git', 'wheels', 'test-wheels',
                       '.cache', 'node_modules') and not (Path(parent) / n).is_symlink()]
            path = Path(parent)
            if path.name == 'carvision' and path.parent.name == 'src':
                roots.append(str(path))
    return sorted(set(roots))


def deployment_artifacts(root):
    """Find even an interrupted upload that left no complete source tree."""
    found = []
    runs = root / 'run'
    if runs.is_dir():
        for record in runs.iterdir():
            if not record.is_dir() or record.is_symlink():
                continue
            for path in record.iterdir():
                if path.name in ('candidate', 'candidate.tar.gz') or path.name.endswith(('.before', '.field-runtime-tmp')):
                    found.append(str(path))
    return sorted(found)


def delete_verified(root, record, expected_snapshot, expected_source):
    root, record = checked_record(root, record)
    verify_source(root, expected_source)
    if snapshot(root, record) != expected_snapshot:
        raise ValueError('Temporary files changed after the PC backup; nothing removed')
    for name in expected_snapshot['targets']:
        path = record / name
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    verify_source(root, expected_source)
    if snapshot(root, record)['targets']:
        raise ValueError('Temporary code artifacts remain')


def main():
    action, root, record = sys.argv[1:4]
    if action == 'verify':
        root = Path(root)
        if not root.is_absolute() or root.resolve() != root or not root.is_dir():
            raise ValueError('Runtime root must be an existing canonical directory')
    else:
        root, record = checked_record(root, record)
    data = json.load(sys.stdin)
    if action == 'snapshot':
        print(json.dumps(snapshot(root, record)))
    elif action == 'archive':
        if snapshot(root, record) != data['snapshot']:
            raise ValueError('Temporary files changed before backup')
        with tarfile.open(fileobj=sys.stdout.buffer, mode='w|gz') as archive:
            for name in sorted(data['snapshot']['entries']):
                archive.add(record / name, arcname=name, recursive=False)
    elif action == 'delete':
        delete_verified(root, record, data['snapshot'], data['source'])
        print(json.dumps({'temporary_code_removed': True}))
    elif action == 'verify':
        verify_source(root, data['source'])
        roots = code_roots([root.parent, Path('/tmp'), Path('/opt'), Path('/srv')])
        if roots != [str(root / 'vision/src/carvision')]:
            raise ValueError('Unexpected code copies require review: ' + repr(roots))
        artifacts = deployment_artifacts(root)
        if artifacts:
            raise ValueError('Earlier deployment artifacts require verified cleanup: ' + repr(artifacts))
        version = data.get('version')
        if version is not None:
            if not isinstance(version, str) or not re.fullmatch(r'[a-zA-Z0-9._-]{1,100}', version):
                raise ValueError('Invalid release version')
            temporary = root / 'CURRENT_VERSION.txt.tmp'
            if temporary.is_symlink() or (root / 'CURRENT_VERSION.txt').is_symlink():
                raise ValueError('Version record must not be a symlink')
            temporary.write_text(version + '\nProduction code: ' + str(root) +
                '\nFull source hashes verified; historical code archived on operator PC.\n', encoding='utf-8')
            temporary.replace(root / 'CURRENT_VERSION.txt')
        print(json.dumps({'source_matches': True, 'source_files': len(data['source']),
                          'code_roots': roots, 'temporary_code_artifacts': artifacts,
                          'single_release_verified': True}))
    else:
        raise ValueError('Unknown cleanup operation')


if __name__ == '__main__':
    main()
