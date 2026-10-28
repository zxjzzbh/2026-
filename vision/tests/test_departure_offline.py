"""Offline departure analysis regressions; no GPIO/serial/camera APIs used."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.departure import DepartureSequence, detect_board
from carvision.race_flow import RaceFlowController, load_bundle
from flow_calibration_fixture import current_crosswalk_sample

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('departure_offline', ROOT / 'vision/tools/departure_offline.py')
offline = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(offline)


def fixture_image(name):
    # OpenCV's filename API does not reliably handle this workspace's Chinese path.
    source=ROOT/'vision/tests/fixtures'/name
    image=cv2.imdecode(np.frombuffer(source.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    assert image is not None, source
    return image


def test_historical_fixtures_demo_one_event_only_even_if_parking_board_appears_later():
    fast = offline.run_replay(offline._demo_frames(ROOT, 10), 10, blue_percent=3., clear_s=.5)
    regular = offline.run_replay(offline._demo_frames(ROOT, 10), 10, blue_percent=3., clear_s=1.)
    assert fast['departure_event_count'] == regular['departure_event_count'] == 1
    assert fast['final_phase'] == regular['final_phase'] == 'complete'
    assert fast['board_confirmed_at_s'] == .5
    assert fast['observed_clear_to_trigger_s'] == .5
    assert regular['observed_clear_to_trigger_s'] == 1.
    assert regular['departure_trigger_at_s'] > fast['departure_trigger_at_s']


def test_no_blue_board_never_arms_or_dispatches():
    normal = np.full((360,480,3), (120,140,150), np.uint8)
    # Non-uniform neutral gray for a healthy image.
    for row in range(360):
        normal[row, :, :] = 50 + (row % 140)
    samples = ({'primary': normal, 'secondary': normal} for _ in range(40))
    report = offline.run_replay(samples, 10)
    assert report['departure_event_count'] == 0
    assert report['final_phase'] == 'wait_board'


def test_unknown_camera_during_clear_never_dispatches():
    present = np.full((360,480,3), (255,0,0), dtype=np.uint8)
    empty = np.tile(np.arange(360,dtype=np.uint8)[:,None,None], (1,480,3))
    blackout = np.zeros_like(present)
    pairs = ([{'primary':present,'secondary':present} for _ in range(8)]
             + [{'primary':empty,'secondary':blackout} for _ in range(30)])
    result = offline.run_replay(pairs, 10, blue_percent=50., clear_s=.5)
    assert result['departure_event_count'] == 0
    assert result['final_phase'] == 'wait_board'  # invalid second camera resets state


def test_two_inputs_required_and_no_hardware_side_effects(tmp_path):
    fixture=ROOT/'vision/tests/fixtures/blue-board-close.jpg'
    with pytest.raises(SystemExit) as exc:
        offline.main(['--primary',str(fixture)])
    assert exc.value.code == 2
    result_file=tmp_path/'result.json'
    assert offline.main(['--primary',str(fixture), '--secondary',str(fixture),
                         '--fps','10','--report',str(result_file)]) == 0
    assert result_file.is_file()
    assert offline.main(['--demo','--blue-percent','3','--clear-s','0.1']) != 0


def test_replayed_image_directories_synchronize_and_detect(tmp_path):
    src = fixture_image('blue-board-close.jpg')
    gone = fixture_image('crosswalk-background-primary.jpg')
    for name in ('primary','secondary'):
        dst = tmp_path/name;dst.mkdir()
        for i in range(8):cv2.imwrite(str(dst/f'{i:02}.png'),src)
        for i in range(8,24):cv2.imwrite(str(dst/f'{i:02}.png'),gone)
    result = offline.run_sources(tmp_path/'primary',tmp_path/'secondary',10.,3.,.5,30,None)
    assert result['departure_event_count']==1 and result['final_phase']=='complete'


def test_parking_blue_fixture_is_not_sufficient_to_retrigger_departure_after_stage_handoff():
    # This historical blue parking prop really does resemble the blue start board.
    # The actual protection is the stage gate, not an unsupported claim that HSV
    # alone distinguishes two similar blue boards.
    parking=fixture_image('parking-border-clipped-left-board-20261013.jpg')
    assert detect_board(parking,min_blue_percent=3)['state']=='present'
    controller=RaceFlowController(load_bundle(ROOT),0.)
    controller._go('wait_board',0.,'test')
    def sample(t,i,board_state):
        value={'sequence':i,'captured_s':t,
                'board': {cam:{'state':board_state,'frame_id':i,'captured_s':t,'scene_valid':True}
                          for cam in ('primary','secondary')},
                'crosswalk': {'fresh':True,'frame_id':i,'captured_s':t,'image_size':[480,360],
                              'candidate':False,'motion':{'valid':True,'moving':False,'captured_s':t}},
                'navigation': {'experimental':True,'ready':True,'captured_s':t,'frame_id':i,
                               'ego_fraction':.5,'half_envelope':.2,
                               'lane_bounds_px':[[100,380],[50,430]],'cones':[], 'uncertain':[],
                               'source_camera':'secondary'}}
        value['crosswalk']=current_crosswalk_sample(controller,value['crosswalk'])
        return value
    messages=[]
    for i in range(1,9):
        t=i*.1
        out=controller.step(sample(t,i,'present'),t,{})
        messages += [e for e in out['events'] if e.get('id')=='departure']
    assert controller.stage=='wait_board'
    for i in range(9,25):
        t=i*.1
        out=controller.step(sample(t,i,'absent'),t,{})
        messages += [e for e in out['events'] if e.get('id')=='departure']
        if controller.stage=='crosswalk':
            break
    # This car intentionally keeps full-flow departure silent; the independent
    # departure panel still announces. Preserve that policy during this update.
    assert controller.stage=='crosswalk' and len(messages)==0
    for i in range(i+1,i+6):
        t=i*.1
        out=controller.step(sample(t,i,'present'),t,{})
        messages += [e for e in out['events'] if e.get('id')=='departure']
    assert len(messages)==0 and controller.stage=='crosswalk'
