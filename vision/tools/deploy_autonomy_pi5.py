"""Deploy a reviewed local bundle only while Pi control is paused.

Does not start/restart a service, open UART, speak or run a real-motion command.
Password comes from SMARTCAR_SSH_PASSWORD; host key must already be trusted.
All replacements are backed up, verified by SHA256 and atomically renamed.
Existing changed calibration files are refused rather than overwritten.
"""
import argparse
import hashlib
import json
import os
import shlex
import sys
from pathlib import Path, PurePosixPath


def digest(data):
    return hashlib.sha256(data).hexdigest()


def execute(client, arguments, timeout=25):
    _stdin, stdout, stderr = client.exec_command(shlex.join(arguments), timeout=timeout)
    result = {"stdout": stdout.read().decode(), "stderr": stderr.read().decode(),
              "rc": stdout.channel.recv_exit_status()}
    if result["rc"]:
        raise RuntimeError(json.dumps(result))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="extracted project root")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--host", required=True, help="explicit trusted Pi address; no personal address is shipped")
    parser.add_argument("--host-key-alias", help="optional already-verified known_hosts name for this same Pi")
    parser.add_argument("--remote", default="/home/pi/smartcar-race-20261002")
    parser.add_argument("--ssh-deps", type=Path, help="optional existing directory containing paramiko")
    args = parser.parse_args()
    if args.ssh_deps:
        sys.path.insert(0, str(args.ssh_deps))
    import paramiko
    root = args.root.resolve()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8-sig"))
    targets = []
    for name, expected in manifest["files"].items():
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts or not path.parts or path.parts[0] != "vision":
            raise ValueError("manifest path is outside vision")
        payload = (root / name).read_bytes()
        if digest(payload) != expected:
            raise ValueError("source changed since validation: " + name)
        targets.append((name, payload))
    remote = PurePosixPath(args.remote)
    if not remote.is_absolute() or ".." in remote.parts or str(remote) != "/home/pi/smartcar-race-20261002":
        raise ValueError("deployment target must be the existing named Pi workspace")
    client = paramiko.SSHClient()
    client.load_host_keys(str(Path.home() / ".ssh/known_hosts"))
    if args.host_key_alias:
        pinned = client.get_host_keys().lookup(args.host_key_alias)
        if not pinned:
            raise RuntimeError("verified Pi host key unavailable")
        for kind, key in pinned.items():
            client.get_host_keys().add(args.host, kind, key)
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username="pi", password=os.environ["SMARTCAR_SSH_PASSWORD"],
                       look_for_keys=False, allow_agent=False, timeout=8)
        status = json.loads(execute(client, ["curl", "--fail", "--silent", "--max-time", "2", "http://127.0.0.1:8080/api/drive/status"])["stdout"])
        if status.get("controls_paused") is not True or status.get("hardware_output") is not False:
            raise RuntimeError("deployment requires the existing driving service to be paused")
        backup = remote / "run" / ("autonomy-source-backup-" + manifest["bundle_id"])
        execute(client, ["mkdir", str(backup)])  # new backup; never overwrite a previous deployment
        with client.open_sftp() as sftp:
            # First check all configs, before mutating any file.
            for name, payload in targets:
                if not name.startswith("vision/configs/"):
                    continue
                try:
                    with sftp.open(str(remote / name), "rb") as stream:
                        old = stream.read()
                except FileNotFoundError:
                    old = None
                if old is not None and digest(old) != digest(payload):
                    raise RuntimeError("preserve existing Pi configuration; needs explicit merge: " + name)
            for name, payload in targets:
                destination = str(remote / name)
                execute(client, ["mkdir", "-p", str((remote / name).parent)])
                try:
                    with sftp.open(destination, "rb") as stream:
                        old = stream.read()
                except FileNotFoundError:
                    old = None
                if old is not None:
                    with sftp.open(str(backup / name.replace("/", "__")), "wb") as stream:
                        stream.write(old)
                temporary = destination + ".autonomy-upload"
                with sftp.open(temporary, "wb") as stream:
                    stream.write(payload)
                with sftp.open(temporary, "rb") as stream:
                    if digest(stream.read()) != digest(payload):
                        raise RuntimeError("uploaded source hash mismatch: " + name)
                sftp.posix_rename(temporary, destination)
        python = str(remote / ".venv/bin/python")
        check = execute(client, ["nice", "-n", "10", "timeout", "15", python, "-m", "carvision", "autonomy-check", "--profile", str(remote / "vision/configs/autonomy-pi5.json")])
        tests = execute(client, ["nice", "-n", "10", "timeout", "20", python, "-m", "pytest", str(remote / "vision/tests/test_autonomy_stack.py"), "-q", "--disable-warnings"])
        after = json.loads(execute(client, ["curl", "--fail", "--silent", "--max-time", "2", "http://127.0.0.1:8080/api/drive/status"])["stdout"])
        if after.get("controls_paused") is not True or after.get("hardware_output") is not False:
            raise RuntimeError("driving state changed during deployment")
        receipt = {"bundle_id": manifest["bundle_id"], "host": args.host, "source_uploaded": True,
                   "controls_paused": True, "services_restarted": False, "motion_commands_sent": False,
                   "backup": str(backup), "readiness": json.loads(check["stdout"]), "tests": tests}
        with (root / "deployment-receipt.json").open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(receipt, ensure_ascii=False, indent=2))
    finally:
        client.close()


if __name__ == "__main__":
    main()
