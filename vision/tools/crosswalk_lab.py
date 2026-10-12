"""Portable source-tree entry; does not require an editable package install."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from carvision.cli import main


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("replay", "plan"):
        raise SystemExit("Usage: python vision/tools/crosswalk_lab.py {replay|plan} --output NEW_DIR [options]")
    raise SystemExit(main(["crosswalk-" + sys.argv[1], *sys.argv[2:]]))
