"""Scoped deployment cleanup. Executed over SSH; never starts vehicle services."""
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
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


DEFAULT_SOURCE_ROOTS = ('vision/src', 'vision/web')
CPP_SOURCE_ROOTS = ('cpp/src', 'cpp/include', 'cpp/config', 'cpp/tests', 'cpp/tools',
                    'cpp/CMakeLists.txt')


def checked_source_roots(root, source_roots=None):
    """Never allow an explicit file subset to replace mandatory complete trees."""
    supplied = DEFAULT_SOURCE_ROOTS if source_roots is None else source_roots
    if not isinstance(supplied, (list, tuple)) or not supplied:
        raise ValueError('source_roots must be a nonempty list of relative paths')
    selected = set()
    for name in supplied:
        if not isinstance(name, str):
            raise ValueError('Invalid source root')
        path = PurePosixPath(name)
        if (not name or path.is_absolute() or '..' in path.parts or ':' in name
                or '\\' in name or path.as_posix() != name or name == '.'):
            raise ValueError('Source root must be a canonical relative path: ' + name)
        selected.add(name)
    # Requiring explicit directory entries avoids accidentally checking just a
    # few selected files while claiming a complete release was verified.
    required = set(DEFAULT_SOURCE_ROOTS)
    if (root / 'cpp').exists():
        required.update(CPP_SOURCE_ROOTS)
    if not required.issubset(selected):
        raise ValueError('Missing complete source roots: ' + repr(sorted(required - selected)))
    return tuple(sorted(selected))


def source_hashes(root, source_roots=None):
    root = Path(root)
    result = {}
    for sub in checked_source_roots(root, source_roots):
        base = root / sub
        if not base.exists() or base.resolve() != base:
            raise ValueError('Missing or linked source root: ' + str(base))
        paths = [base]
        if base.is_dir():
            for parent, dirs, files in os.walk(base, followlinks=False):
                paths.extend(Path(parent) / name for name in dirs + files)
        for path in paths:
            info = path.lstat()
            if path.resolve() != path or not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode)):
                raise ValueError('Source links and special files are not allowed: ' + str(path))
            if stat.S_ISREG(info.st_mode) and path.suffix not in ('.pyc', '.pyo'):
                result[path.relative_to(root).as_posix()] = sha(path)
    return result


def verify_source(root, expected, source_roots=None):
    if not expected or source_hashes(root, source_roots) != expected:
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


def cpp_code_copies(search_roots):
    """Inventory C++ projects and runtime binaries, without deleting anything.

    CMake's generated build metadata is not a second source project. Source
    copies are recognized by the project declaration or the smartcar header
    layout; ELF binaries named smartcar (including backup suffixes) are also
    reported even when copied away from a complete source tree.
    """
    projects, binaries = set(), set()
    for base in search_roots:
        for parent, dirs, files in os.walk(base, followlinks=False):
            dirs[:] = [n for n in dirs if n not in ('.venv', '.git', 'wheels', 'test-wheels',
                       '.cache', 'node_modules', 'CMakeFiles', '_deps')
                       and not (Path(parent) / n).is_symlink()]
            path = Path(parent)
            project = (path / 'include/smartcar').is_dir() and (path / 'src/main.cpp').is_file()
            cmake = path / 'CMakeLists.txt'
            if 'CMakeLists.txt' in files and not cmake.is_symlink():
                text = cmake.read_text(encoding='utf-8', errors='replace')
                project = project or bool(re.search(r'\bproject\s*\(\s*smartcar(?:[\s_)\-]|$)',
                                                     text, re.IGNORECASE))
            if project:
                projects.add(str(path))
            for name in files:
                if not re.fullmatch(r'smartcar(?:[._-].*)?', name):
                    continue
                binary = path / name
                if binary.is_symlink():
                    continue
                with binary.open('rb') as stream:
                    if stream.read(4) == b'\x7fELF':
                        binaries.add(str(binary))
    return {'projects': sorted(projects), 'binaries': sorted(binaries)}


def verify_cpp_copies(root, search_roots, expected_source):
    copies = cpp_code_copies(search_roots)
    expected_projects = [str(root / 'cpp')] if (root / 'cpp').is_dir() else []
    # Every allowed executable must be individually SHA-256 checked in the
    # installed manifest. A second standalone executable is never implied.
    allowed_binaries = {str(root / name) for name in expected_source
                        if name == 'cpp/build/smartcar'}
    if copies['projects'] != expected_projects or set(copies['binaries']) != allowed_binaries:
        raise ValueError('Unexpected or unverified C++ code copies require review: ' + repr(copies))
    return copies


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


def delete_verified(root, record, expected_snapshot, expected_source, source_roots=None):
    root, record = checked_record(root, record)
    verify_source(root, expected_source, source_roots)
    if snapshot(root, record) != expected_snapshot:
        raise ValueError('Temporary files changed after the PC backup; nothing removed')
    for name in expected_snapshot['targets']:
        path = record / name
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    verify_source(root, expected_source, source_roots)
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
        delete_verified(root, record, data['snapshot'], data['source'], data.get('source_roots'))
        print(json.dumps({'temporary_code_removed': True}))
    elif action == 'verify':
        verify_source(root, data['source'], data.get('source_roots'))
        search_roots = [root.parent, Path('/tmp'), Path('/opt'), Path('/srv')]
        roots = code_roots(search_roots)
        if roots != [str(root / 'vision/src/carvision')]:
            raise ValueError('Unexpected code copies require review: ' + repr(roots))
        cpp_copies = verify_cpp_copies(root, search_roots, data['source'])
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
                          'source_roots': checked_source_roots(root, data.get('source_roots')),
                          'cpp_code_copies': cpp_copies,
                          'single_release_verified': True}))
    else:
        raise ValueError('Unknown cleanup operation')


if __name__ == '__main__':
    main()
