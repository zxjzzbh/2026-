"""Collect actual traffic-light images through the car's existing preview service."""
import argparse
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
from carvision.traffic_capture_server import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',required=True,help='existing car preview origin; GET only')
    parser.add_argument('--output',type=Path,default=ROOT/'data/raw/traffic_lights')
    parser.add_argument('--port',type=int,default=8091)
    args=parser.parse_args()
    if not 1024<=args.port<=65535:parser.error('port must be 1024..65535')
    run(args.preview,args.output,args.port,ROOT/'vision/web/traffic_capture.html')


if __name__=='__main__':main()
