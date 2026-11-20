import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent / 'parking_modules'))
from standalone_parking_20261119.parking_only_test import main

if __name__ == '__main__':
    raise SystemExit(main())
