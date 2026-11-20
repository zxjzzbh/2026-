"""All-off field photographs and stop qualification; no camera/actuator access."""
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

CPP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(CPP/'algorithms'))
from hybrid import Signals
from legacy.traffic_signal import TrafficSignalDetector
from legacy.traffic_photometry import RedPhotometry


def lamp(active):
    image=np.full((360,480,3),25,np.uint8)
    for i,(x,color) in enumerate(((100,(0,0,70)),(210,(0,70,70)),(320,(0,70,0)))):
        cv2.circle(image,(x,120),27,tuple(255 if c else 0 for c in color) if i==active else color,-1)
    return image


class TrafficOff(unittest.TestCase):
    def test_sensor_trigger_red_frames_confirm_at_actual_six_hz(self):
        signals=Signals(CPP/'config')
        results=[]
        for i,stamp in enumerate((0.,.2,.334)):
            image=cv2.imread(str(CPP/'tests/fixtures'/('traffic_trigger_red_'+str(1640+i)+'.jpg')))
            self.assertEqual(signals.detector.detect(image)['state'],'red')
            results.append(signals.update({'secondary':(image,i,stamp)},stamp+.18,'traffic_approach'))
        self.assertEqual([r['stop'] for r in results],[0,0,1],results)
        self.assertEqual(results[-1]['stop_reason'],'confirmed_current_red')

    def test_one_metre_stop_independent_of_colour(self):
        for active in (None, 0, 2):
            signals = Signals(CPP/'config', 204)
            im = lamp(active)
            far = np.full_like(im, 25)
            far[90:270,120:360] = cv2.resize(im,(240,180))
            first = signals.update({'primary':(far,1,1.)},1.,'traffic_approach')
            near = signals.update({'primary':(im,2,1.1)},1.1,'traffic_approach')
            self.assertFalse(first['stop'])
            self.assertEqual(near['stop_reason'],'one_metre_fixture')
            self.assertFalse(near['green'])

    def test_unknown_breaks_continuous_green_release_window(self):
        signals=Signals(CPP/'config')
        for i in range(40):
            result=signals.update({'primary':(lamp(None if i%10==9 else 2),i,i/10)},i/10,'traffic_approach')
            self.assertEqual(result['green'],0,(i,result))

    def test_green_160_does_not_need_to_outshine_red_yellow_plastic(self):
        for value,expected in ((159,'unknown'),(160,'green'),(192,'green')):
            image=lamp(None)
            cv2.circle(image,(100,120),27,(0,0,255),-1)
            cv2.circle(image,(210,120),27,(0,248,248),-1)
            cv2.circle(image,(320,120),27,(0,value,0),-1)
            result=TrafficSignalDetector().detect(image)
            self.assertEqual(result['state'],expected,result)

    def test_normalized_red_can_brighten_relative_to_background_while_median_falls(self):
        def signal(core,housing,peaks,state='unknown',clipped=0):
            f=dict(median_v=core,clipped_fraction=clipped,white_fraction=0.,local_contrast=core-housing,
                   led_peak_fraction=peaks,color_fraction=1.,distributed_halo=True)
            return dict(state=state,fixture_detected=True,bbox_xyxy=[80,80,350,160],housing_median_v=housing,
                        lamp_features=[f,{**f,'median_v':70},{**f,'median_v':70}])
        model=RedPhotometry()
        for i in range(3):model.update(signal(140,100,.01,'off'),i,i/10,i/10)
        self.assertIsNotNone(model.reference)
        globally_dim=model.update(signal(70,50,.01),3,.3,.3)
        self.assertNotEqual(globally_dim['state'],'red')
        powered=model.update(signal(120,50,.12),4,.4,.4)
        self.assertEqual(powered['state'],'red',powered)
        self.assertGreater(powered['red_photometry']['compensated_red_rise'],30)

    def test_unknown_or_clipped_frames_never_become_off_baseline(self):
        model=RedPhotometry()
        f=dict(median_v=255,clipped_fraction=.9,white_fraction=0.,local_contrast=100.,
               led_peak_fraction=.1,color_fraction=1.,distributed_halo=True)
        s=dict(state='unknown',fixture_detected=True,bbox_xyxy=[80,80,350,160],housing_median_v=140,
               lamp_features=[f,f,f])
        for i in range(8):model.update(s,i,i/10,i/10)
        self.assertIsNone(model.reference)

    def test_current_sunlit_all_off_photographs_keep_real_fixture(self):
        for camera in ('primary','secondary'):
            image=cv2.imread(str(CPP/'tests/fixtures'/('traffic_off_sun_'+camera+'.jpg')))
            result=TrafficSignalDetector().detect(image)
            self.assertTrue(result['fixture_detected'],result)
            self.assertIn(result['state'],('off','unknown'),result)
            self.assertEqual(result['reason'],'ambiguous_emission')

    def test_fence_and_sky_cannot_form_a_green_fixture(self):
        image=cv2.imread(str(CPP/'tests/fixtures/traffic_off_false_fixture.jpg'))
        result=TrafficSignalDetector().detect(image)
        self.assertFalse(result['fixture_detected'],result)
        self.assertNotEqual(result['state'],'green')

    def test_true_synthetic_lights_survive_geometry_filter(self):
        for active,color in enumerate(('red','yellow','green')):
            result=TrafficSignalDetector().detect(lamp(active))
            self.assertEqual(result['state'],color,result)

    def test_transient_red_or_yellow_cannot_latch_stop(self):
        for active in (0,1):
            signals=Signals(CPP/'config')
            for i in range(20):
                image=lamp(active if i==0 else None)
                result=signals.update({'primary':(image,i,i/10)},i/10,'traffic_approach')
                self.assertEqual((result['red'],result['stop'],result['green']),(0,0,0),(i,result))
            self.assertEqual(signals.stop_boxes,{})
            self.assertIsNone(signals.guard.stop_reason)

    def test_three_distinct_frames_stop_and_unknown_does_not_release(self):
        for active in (0,1):
            signals=Signals(CPP/'config');image=lamp(active)
            results=[signals.update({'primary':(image,i,i/10)},i/10,'traffic_approach') for i in range(3)]
            self.assertEqual([r['stop'] for r in results],[0,0,1],results)
            self.assertIn('primary',signals.stop_boxes)
            result=signals.update({'primary':(lamp(None),3,.3)},.3,'traffic_approach')
            self.assertEqual(result['stop'],1)
            for i in range(4,15):
                result=signals.update({'primary':(lamp(None),i,i/10)},i/10,'traffic_wait_green')
                self.assertEqual(result['green'],0)

    def test_duplicate_stale_and_two_cameras_do_not_combine_streaks(self):
        signals=Signals(CPP/'config');red=lamp(0)
        for now in (0,.05,.1):
            self.assertEqual(signals.update({'primary':(red,1,0)},now,'traffic_approach')['stop'],0)
        self.assertEqual(signals.update({'primary':(red,2,.2)},.5,'traffic_approach')['stop'],0)
        signals=Signals(CPP/'config')
        for i in range(8):
            cam=('primary','secondary')[i%2]
            self.assertEqual(signals.update({cam:(red,i,i/10)},i/10,'traffic_approach')['stop'],0)

    def test_same_fixture_green_requires_full_fresh_wait(self):
        signals=Signals(CPP/'config')
        for i in range(3):signals.update({'primary':(lamp(0),i,i/10)},i/10,'traffic_approach')
        results=[signals.update({'primary':(lamp(2),i,i/10)},i/10,'traffic_wait_green') for i in range(3,28)]
        self.assertFalse(any(r['green'] for r in results[:20]),results)
        self.assertTrue(results[-1]['green'],results[-1])


if __name__=='__main__':unittest.main(verbosity=2)
