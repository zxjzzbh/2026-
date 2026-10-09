"""F/B parking entry. No commands are sent unless camera --run is explicit."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from carvision.brake_parking import load_settings
from carvision.brake_runtime import camera,replay


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    for name in ('check','replay','camera'):
        d=sub.add_parser(name)
        d.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'configs/brake-parking.json'))
        if name!='check':
            d.add_argument('--output',required=True)
            d.add_argument('--path-mode',choices=['straight','lane'],default='straight')
        if name=='replay':d.add_argument('--input',required=True)
        if name=='camera':
            d.add_argument('--preview',default='http://127.0.0.1:8080')
            d.add_argument('--seconds',type=float,default=60)
            d.add_argument('--show',action='store_true')
            d.add_argument('--feedback',help='optional same-Pi verified speed JSON')
            d.add_argument('--run',action='store_true')
            d.add_argument('--esc-mode',choices=['F/B','F/R','F/B/R'])
            d.add_argument('--port',default='/dev/ttyAMA0')
    a=p.parse_args(argv)
    try:
        if a.action=='check':
            result={'settings':load_settings(a.config),'hardware_output':False,'program_status':'written, field tuning unverified'}
        else:result={'replay':replay,'camera':camera}[a.action](a)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    except KeyboardInterrupt:
        print('Stopped',file=sys.stderr);return 130
    except Exception as exc:
        print(f'{type(exc).__name__}: {exc}',file=sys.stderr);return 1


if __name__=='__main__':
    raise SystemExit(main())
