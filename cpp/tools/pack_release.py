"""Build and verify the complete hybrid source archive."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[2]
PREFIX = "smartcar_outdoor_parking1618_cone1115_20261120"
SKIP = {"build", "runs", "run", "__pycache__", ".pytest_cache", ".git", ".venv"}
TREES = {"cpp", "vision", "deployment"}


def package(destination):
    destination = Path(destination).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    files = [ROOT / "README.md"]
    for tree in sorted(TREES):
        files.extend(p for p in (ROOT / tree).rglob("*")
                     if p.is_file() and not SKIP.intersection(p.relative_to(ROOT).parts)
                     and p.suffix.lower() not in {".pyc", ".obj", ".exe", ".o", ".zip"})
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(files)}
    manifest = ROOT / "MANIFEST.json"
    manifest.write_text(json.dumps({"version": PREFIX, "sha256": hashes}, indent=2,
                                   ensure_ascii=False) + "\n", encoding="utf-8")
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in [*sorted(files), manifest]:
            info = zipfile.ZipInfo.from_file(path, PREFIX + "/" + path.relative_to(ROOT).as_posix())
            info.create_system = 3
            info.external_attr = (0o100755 if path.suffix == ".sh" else 0o100644) << 16
            archive.writestr(info, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED)
    with zipfile.ZipFile(destination) as archive:
        names = {PREFIX + "/" + key for key in [*hashes, "MANIFEST.json"]}
        if names != set(archive.namelist()) or archive.testzip() is not None:
            raise RuntimeError("Invalid source archive")
        for name, expected in hashes.items():
            if hashlib.sha256(archive.read(PREFIX + "/" + name)).hexdigest() != expected:
                raise RuntimeError("Hash mismatch: " + name)
    print(f"{len(hashes) + 1} files, SHA-256 {hashlib.sha256(destination.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    package(parser.parse_args().destination)
