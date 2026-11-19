"""Archive only this deployment's temporary code on the PC, then remove it on Pi."""
import hashlib
import json
from pathlib import Path
import shlex
import tarfile

REMOTE_SCRIPT = Path(__file__).with_name('release_cleanup_remote.py').read_text(encoding='utf-8')


def _call(client, python, action, root, record, data, output=None):
    stdin, stdout, stderr = client.exec_command(
        shlex.join([python, '-c', REMOTE_SCRIPT, action, root, record]), timeout=240)
    stdin.write(json.dumps(data).encode('utf-8'))
    stdin.flush()
    stdin.channel.shutdown_write()
    if output is None:
        raw = stdout.read()
    else:
        while chunk := stdout.read(1024 * 1024):
            output.write(chunk)
        raw = b''
    error = stderr.read().decode('utf-8', 'replace')
    if stdout.channel.recv_exit_status():
        raise RuntimeError(error or raw.decode('utf-8', 'replace'))
    return json.loads(raw) if output is None else None


def verify_archive(path, expected):
    actual = {}
    with tarfile.open(path, 'r:gz') as archive:
        for member in archive:
            name = member.name.rstrip('/')
            if name in actual or name not in expected['entries']:
                raise ValueError('Unexpected or duplicate archive member: ' + name)
            if member.isdir():
                actual[name] = {'kind': 'dir', 'mode': member.mode}
            elif member.isfile():
                with archive.extractfile(member) as stream:
                    digest = hashlib.file_digest(stream, 'sha256').hexdigest()
                actual[name] = {'kind': 'file', 'mode': member.mode, 'size': member.size, 'sha256': digest}
            else:
                raise ValueError('Backup contains a link or special file')
    if actual != expected['entries']:
        raise ValueError('PC backup is incomplete or changed')


def verify_installed(client, *, python, root, expected_source, version=None, source_roots=None):
    return _call(client, python, 'verify', root, root + '/run',
                 {'source': expected_source, 'version': version, 'source_roots': source_roots})


def finalize(client, *, python, root, record, local_record, expected_source, version=None,
             source_roots=None):
    """No delete before a complete local backup. Failures block success reporting.

    expected_source is the full installed source manifest, never just a patch.
    source_roots extends verification to the complete C++ source and binary set;
    vision/src and vision/web remain mandatory for every deployment.
    Unknown copies elsewhere are reported for review, not blindly deleted.
    """
    local_record = Path(local_record)
    local_record.mkdir(parents=True, exist_ok=True)
    snapshot = _call(client, python, 'snapshot', root, record, {})
    (local_record / 'temporary-code-manifest.json').write_text(
        json.dumps(snapshot, indent=2), encoding='utf-8')
    if snapshot['targets']:
        archive = local_record / 'temporary-code.tar.gz'
        # Keep a previous verified backup intact if a later attempt fails.
        partial = archive.with_suffix('.partial')
        with partial.open('wb') as output:
            _call(client, python, 'archive', root, record, {'snapshot': snapshot}, output)
        verify_archive(partial, snapshot)
        partial.replace(archive)
        _call(client, python, 'delete', root, record,
              {'snapshot': snapshot, 'source': expected_source, 'source_roots': source_roots})
    result = verify_installed(client, python=python, root=root,
                              expected_source=expected_source, version=version,
                              source_roots=source_roots)
    result.update(temporary_code_removed=True, local_backup_directory=str(local_record))
    (local_record / 'single-release-verification.json').write_text(
        json.dumps(result, indent=2), encoding='utf-8')
    return result
