"""Current competition rules, with synthetic measurements and no vehicle I/O."""
import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carvision.parking_entry_run import ParkingEntryRun
from carvision.race_flow import load_bundle
from carvision.race_flow_console import RaceFlowDrive
from test_parking_entry_run import configurations
from test_parking_stages import ground
from test_race_flow import measured_bundle

ROOT=Path(__file__).resolve().parents[2]


def test_visual_controller_uses_inner_bay_dimensions_but_keeps_actual_lane_width():
    parking,brake,cone=configurations()
    parking.setdefault('visual_entry',{})['ground_pose_confirmed']=True
    cone['ground']=ground()
    controller=ParkingEntryRun._controller({'parking':parking,'brake':brake,'cone':cone,
        'controller_mode':'visual_trial','selected_slot':'left','forward_us':1570,'steering_gain':1.})
    width=parking['lane_width_m']/2-.05
    assert controller.route.bay_length==pytest.approx(1.3)
    assert controller.route.bay_width==pytest.approx(width)
    assert controller.track_width==parking['lane_width_m']
    # Projected coordinates isolate the length passed by the real call site.
    controller.route.bay_pose.view=SimpleNamespace(error=.01,segment=lambda a,b:np.linspace(a,b,60))
    landmarks={'source':'current_bay_landmarks','frame_id':1,'captured_s':1.,'far_observed':True,
        'far_edge_px':[[-width/2,2.4],[width/2,2.4]],
        'side_edges_px':{'left':[[-width/2,1.1],[-width/2,2.4]],'right':[[width/2,1.1],[width/2,2.4]]},
        'side_support':{'left':1.,'right':1.}}
    pose=controller.route.bay_pose.measure(landmarks,1,1.)
    assert pose['distance_m']==pytest.approx(1.1)
    assert pose['center_m'][0]==pytest.approx(0.)


def formal_drive(tmp_path,monkeypatch,bundle):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *args:copy.deepcopy(bundle))
    base=SimpleNamespace(status=lambda:{'mode':'enabled','owner_client':'page','setup_token':'setup'},
        request=lambda *args:pytest.fail('readiness must not change manual output'),
        payment=SimpleNamespace(freeze_race_code=lambda *args:pytest.fail('readiness must precede payment freeze')))
    return RaceFlowDrive(base,{},tmp_path,live=True,offline_competition=True,require_ready=False,
        actuator_factory=lambda _:pytest.fail('readiness must precede output acquisition'))


def enter():
    return {'client':'page','setup_token':'setup','stationary_confirmed':True,
            'in_switch_zone':True,'fb_mode_confirmed':True}


def test_formal_entry_rejects_missing_parking_before_releasing_manual_owner(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT);original=copy.deepcopy(bundle)
    flow=formal_drive(tmp_path,monkeypatch,bundle)
    report=flow.status()['race_flow']
    assert not report['can_enter'] and not report['start_readiness']['ready']
    assert len(report['start_readiness']['missing'])==13
    with pytest.raises(ValueError,match='启动条件'):flow.request('race_enter',enter())
    assert not flow.active and flow.thread is None and flow.owner is None
    assert bundle==original and not (tmp_path/'run').exists()


@pytest.mark.parametrize('change',[{'zone_length_m':1.},{'boundary_tape_width_m':0.},
                                  {'boundary_tape_width_m':None}])
def test_formal_entry_rejects_legacy_dimensions_without_rewriting_calibration(tmp_path,monkeypatch,change):
    bundle=measured_bundle(load_bundle(ROOT));bundle['parking'].update(change)
    if change.get('boundary_tape_width_m',1) is None:bundle['parking'].pop('boundary_tape_width_m')
    original=copy.deepcopy(bundle);flow=formal_drive(tmp_path,monkeypatch,bundle)
    assert not flow.status()['race_flow']['can_enter']
    with pytest.raises(ValueError,match='1.5.*0.1'):flow.request('race_enter',enter())
    assert bundle==original and flow.thread is None


def test_formal_entry_rechecks_current_configuration_after_page_load(tmp_path,monkeypatch):
    bundle=measured_bundle(load_bundle(ROOT));flow=formal_drive(tmp_path,monkeypatch,bundle)
    assert flow.status()['race_flow']['can_enter']
    bundle['parking']['zone_length_m']=1.
    with pytest.raises(ValueError,match='1.5.*0.1'):flow.request('race_enter',enter())
    assert not flow.active and flow.thread is None
