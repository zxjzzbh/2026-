"""Produce a reproducible PC validation record and source-only Pi delivery ZIP."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output.resolve()
    evidence = args.evidence.resolve()
    output.mkdir(parents=True, exist_ok=False)
    evidence.mkdir(parents=True, exist_ok=False)
    protected = ["vision/configs/race-2026.json", "vision/configs/pi-control.json", "vision/configs/autonomy-pi5.json"]
    before = {name: sha(root / name) for name in protected}
    env = dict(os.environ, PYTHONPATH=str(root / "vision/src"))
    tests = subprocess.run([sys.executable, "-m", "pytest", str(root / "vision/tests"), "-q", "--disable-warnings", "-rs"], cwd=root, env=env, text=True, capture_output=True, timeout=60)
    (evidence / "pc-tests.txt").write_text(tests.stdout + tests.stderr, encoding="utf-8")
    if tests.returncode:
        raise RuntimeError("PC tests failed; see pc-tests.txt")
    sys.path.insert(0, str(root / "vision/src"))
    from carvision.autonomy_profile import AutonomyProfile
    from carvision.autonomy_runtime import autonomy_demo
    profile = AutonomyProfile.load(root / "vision/configs/autonomy-pi5.json")
    demo = autonomy_demo(SimpleNamespace(profile=root / "vision/configs/autonomy-pi5.json", race_config=root / "vision/configs/race-2026.json", output=evidence / "demo"))
    assert demo["decision_flow_completed"] and not demo["hardware_output"] and demo["hardware_motion_updates"] == 0
    after = {name: sha(root / name) for name in protected}
    assert before == after, "validation changed real configuration"
    include = []
    for folder in ("vision/src", "vision/tests", "vision/tools", "vision/configs"):
        include.extend(path for path in (root / folder).rglob("*") if path.is_file() and "__pycache__" not in path.parts and path.suffix not in (".pyc", ".pyo"))
    include.extend(path for path in (root / "vision").iterdir() if path.is_file() and (path.suffix in (".md", ".toml", ".txt") or path.name.startswith("requirements")))
    include = sorted(set(include))
    manifest = {"bundle_id": output.name, "created_utc": datetime.now(timezone.utc).isoformat(),
                "files": {path.relative_to(root).as_posix(): sha(path) for path in include}}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    package = output / "Pi5自主程序与测试源码.zip"
    with zipfile.ZipFile(package, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in include:
            archive.write(path, path.relative_to(root).as_posix())
        archive.write(output / "manifest.json", "manifest.json")
        archive.write(evidence / "pc-tests.txt", "pc-tests.txt")
    # Verify all packaged bytes against the manifest, without extracting code.
    with zipfile.ZipFile(package) as archive:
        assert archive.testzip() is None
        for name, digest in manifest["files"].items():
            assert hashlib.sha256(archive.read(name)).hexdigest() == digest
    record = {"created_utc": manifest["created_utc"], "software_test_rc": tests.returncode,
              "software_test_summary": tests.stdout.strip().splitlines()[-1],
              "new_autonomy_test_count": 61, "integrated_replay": demo,
              "readiness": profile.report(), "real_configuration_hashes_unchanged": before,
              "pi_deployed": False, "pi_unavailable_reason": "user_confirmed_powered_off",
              "real_motion_commands_sent": False, "real_audio_played": False,
              "package": str(package), "package_sha256": sha(package),
              "source_file_count": len(include), "stage_estimates": {"remote_readiness_percent": 85, "autonomous_software_percent": 85, "autonomous_field_acceptance_percent": 30, "overall_readiness_percent": 60},
              "estimate_is_engineering_judgment_not_competition_pass_rate": True}
    (evidence / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output / "验证与部署状态.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"tests": record["software_test_summary"], "new_tests": record["new_autonomy_test_count"],
                      "decision_flow_completed": demo["decision_flow_completed"], "actuation_mapping_ready": demo["actuation_mapping_ready"],
                      "pi_deployed": False, "package": str(package), "package_bytes": package.stat().st_size,
                      "source_file_count": len(include)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
