"""Read-only readiness check; no camera, UART, GPIO or motor commands."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from carvision.feedback_parking import check_file

if __name__=='__main__':
    path=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'configs/parking-feedback.json'
    print(json.dumps(check_file(path),ensure_ascii=False,indent=2))
