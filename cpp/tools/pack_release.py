"""Package one source tree and verify every archived payload hash. No deployment."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = {"CMakeLists.txt", "README.md", "build.sh", "run_race.sh", "run_debug.sh", ".gitignore", ".clang-format", "VALIDATION.json"}
DIRS = {"src", "include", "config", "docs", "tests", "tools"}


def package(destination):
    files = sorted(p for p in ROOT.rglob("*") if p.is_file()
                   and (p.relative_to(ROOT).parts[0] in DIRS or p.name in ROOT_FILES and p.parent == ROOT)
                   and "__pycache__" not in p.parts and p.suffix not in {".pyc", ".obj", ".exe", ".o"})
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    manifest = {"version": "car-2026-rules-20261118", "scope": "independent C++ source package; not deployed",
                "sha256": hashes}
    (ROOT / "MANIFEST.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    destination = Path(destination).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as z:
        for p in [*files, ROOT / "MANIFEST.json"]:
            z.write(p, ROOT.name + "/" + p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(destination) as z:
        expected = {ROOT.name + "/" + name for name in hashes} | {ROOT.name + "/MANIFEST.json"}
        if set(z.namelist()) != expected:
            raise RuntimeError("Archive file set differs from release manifest")
        for name, digest in hashes.items():
            if hashlib.sha256(z.read(ROOT.name + "/" + name)).hexdigest() != digest:
                raise RuntimeError("Archive hash mismatch: " + name)
    print(json.dumps({"archive": str(destination), "files": len(hashes) + 1,
                      "sha256": hashlib.sha256(destination.read_bytes()).hexdigest()}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("destination", type=Path)
    package(parser.parse_args().destination)
