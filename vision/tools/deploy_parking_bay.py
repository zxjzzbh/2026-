"""Install parking perception around the existing real car console; never drive."""
import sys
from pathlib import Path
from deploy_parking_payment import main

if __name__=='__main__':
    if len(sys.argv)==1:
        sys.argv+=['--handoff',str(Path(__file__).resolve().parents[3]/'连接准备_20261009/smartcar-connection-handoff')]
    raise SystemExit(main())
