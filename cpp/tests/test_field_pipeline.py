"""Recorded field regressions. Never opens cameras, audio, UART or I2C."""
import json
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

CPP=Path(__file__).resolve().parents[1]
FIX=CPP/'tests/fixtures'
sys.path.insert(0,str(CPP/'algorithms'))
from hybrid import Hybrid,Signals,read_json
from legacy.cone_vision import ConeDetector,ImageConeTracker
from legacy.cone_road import red_mask

def image(name):
    row=cv2.imread(str(FIX/('field_20261120_'+name+'.jpg')))
    if row is None:raise RuntimeError('missing actual recorded image: '+name)
    return row

def engine():
    c=read_json(CPP/'config/race.json')
    return Hybrid(CPP/'config/race.json',c['primary_camera'],c['secondary_camera'])

class FieldPipeline(unittest.TestCase):
    def test_red_emission_hysteresis_requires_two_prior_strong_frames(self):
        s=Signals(CPP/'config');results=[]
        for i,index in enumerate((1601,1602,1603)):
            results.append(s.update({'secondary':(image('secondary-'+str(index)),index,i*.2)},i*.2+.15,'traffic_approach'))
        self.assertEqual([r['stop'] for r in results],[0,0,1])
        self.assertEqual(results[-1]['stop_reason'],'confirmed_current_red')
        self.assertEqual(s.stop_boxes['secondary'],s.guard.tracks['secondary'].box)
        self.assertEqual(s.detector.detect(image('secondary-1603'))['state'],'unknown')
        for sequence in ((1603,1603,1603),(1601,1598,1603)):
            s=Signals(CPP/'config')
            for i,index in enumerate(sequence):
                r=s.update({'secondary':(image('secondary-'+str(index)),i,i*.2)},i*.2,'traffic_approach')
                self.assertFalse(r['stop'])

    def test_november_15_navigation_on_recorded_field_frames(self):
        h=engine()
        meta=read_json(FIX/'field_20261120_cones.json')
        secondary=[r for r in meta if r['camera']=='secondary']
        primary=[r for r in meta if r['camera']=='primary']
        results=[]
        for a,b in zip(secondary[:3],primary[:3]):
            at,bt=a['metadata'],b['metadata']
            results.append(json.loads(h.step('two_cones',cv2.imread(str(FIX/a['file'])).tobytes(),at['source_frame'],at['time'],
                cv2.imread(str(FIX/b['file'])).tobytes(),bt['source_frame'],bt['time'],max(at['time'],bt['time'])))['cone'])
        self.assertEqual([r['pwm'] for r in results], [1500,1500,1570])
        self.assertTrue(all(r['passed']==0 and r['confirmed_primary']==0 for r in results))
        stale=json.loads(h.step('two_cones',None,0,-1,None,0,-1,max(at['time'],bt['time'])+1))['cone']
        self.assertEqual(stale['pwm'],1500)

if __name__=='__main__':unittest.main()
