"""Debug entry: manual setup and isolated tests of the production task cores."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from carvision.application import main
if __name__ == '__main__':
    main('debug')
