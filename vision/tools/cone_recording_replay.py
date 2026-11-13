"""Export a completed real-camera recording as one portable HTML file. Author: 13246."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from carvision.cone_replay import export_portable


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    print(export_portable(args.recording,args.out or args.recording/'replay.html'))


if __name__=='__main__':main()
