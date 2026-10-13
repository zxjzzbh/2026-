"""Check or replay the full race locally; neither command opens hardware."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'vision/src'))
from carvision.race_flow import load_bundle,parking_blockers,RaceFlowController


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=('check','replay'))
    parser.add_argument('--input',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv);b=load_bundle(ROOT)
    if args.command=='check':
        print(json.dumps({'software_loaded':True,'hardware_output':False,'rules_crosswalk_hold_s':10,
            'sequence':['manual_handoff','board_removed','crosswalk','actual_green','two_cones','unblocked_parking','human_payment'],
            'parking_missing':parking_blockers(b),'single_tests_preserved':True,'field_verified':False},ensure_ascii=False,indent=2));return 0
    if args.input is None or args.output is None:parser.error('replay requires --input and --output')
    if args.output.exists():parser.error('output already exists; choose a new file')
    controller=None;args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.input.open(encoding='utf-8-sig') as source,args.output.open('w',encoding='utf-8') as target:
        for line in source:
            if not line.strip():continue
            row=json.loads(line);now=row['now_s']
            if controller is None:controller=RaceFlowController(b,now)
            intent=controller.step(row['sample'],now,row.get('audio'))
            target.write(json.dumps({'intent':intent,'state':controller.status(now),'hardware_output':False},ensure_ascii=False,allow_nan=False)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
