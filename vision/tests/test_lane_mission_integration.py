"""Real flow/output guards with fake writes and explicit observation evidence."""
import copy

import pytest

from carvision.cone_trial_output import ConePulseGuard
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.race_flow_console import RaceFlowDrive
from cone_mission_fixture import ground_nav,exit_nav
from test_cone_passage import Scene
from test_cone_turn_controller import ROOT,lane


def frozen_settings(bundle):
    captured={}
    class Borrower:
        def borrow_actuator(self,settings,client):
            captured.update(copy.deepcopy(settings));return object()
    shell=RaceFlowDrive.__new__(RaceFlowDrive)
    shell.drive=Borrower();shell.owner='offline-integration'
    shell._make_actuator(bundle)
    return captured


class Rig:
    def __init__(self,stage):
        self.flow=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
        self.flow._go(stage,0.,'offline integration')
        self.settings=frozen_settings(self.flow.b);self.writes=[]
        self.guard=ConePulseGuard(self.settings,lambda *args:self.writes.append(args),mode='F/B')
        self.t=0.;self.sequence=0;self.send_sequence=0;self.sent=[]

    def step(self,nav=None,*,dt=.1,color='green',layout=False):
        self.t=round(self.t+dt,8);self.sequence+=1;t=self.t;i=self.sequence
        if callable(nav):nav=nav(t,i)
        nav=nav or lane(t,fid=i)
        motion={'valid':True,'moving':True,'captured_s':t}
        sample={'sequence':i,'captured_s':t,'navigation':nav,'motion':motion,
            'parking_view':{'layout_found':layout},
            'traffic':{'fresh':True,'frame_id':i,'captured_s':t,'image_size':[480,360],
                'camera_id':'dual','motion':motion,
                'camera_observations':{'secondary':{'camera_id':'secondary','frame_id':i,
                    'captured_s':t,'image_size':[480,360],
                    'signal':{'state':color,'fixture_detected':True,'bbox_xyxy':[100,70,250,125]}}}}}
        result=self.flow.step(sample,t)
        assert result is not None
        # Exact forwarding rule of the real console: duplicate forward intent
        # cannot renew motion, while a new brake/neutral always reaches output.
        if result.get('fresh_control') is not False or result['esc_us']<=1500:
            self.guard.accept({**result,'sequence':self.send_sequence,'sent_s':t},t)
            self.send_sequence+=1;self.sent.append(result)
        assert not self.guard.fault
        return result


def test_real_actuator_settings_freeze_prepared_traffic_profile_without_changing_local_values():
    rig=Rig('parking_search')
    assert rig.settings['traffic_forward_us']==rig.flow.b['traffic']['pwm']['search_us']==1570
    assert rig.settings['cone_forward_us']==[1570,1580]
    result=rig.step(lambda t,i:lane(t,fid=i,near=.54,far=.54))
    assert result['effective_steering_gain']==1. and 1610<result['steering_us']<1870
    assert rig.guard.actual_steering==result['steering_us'] and rig.guard.esc==result['esc_us']


@pytest.mark.parametrize('fresh_pair_at_deadline',[False,True])
def test_duplicate_or_single_new_pair_cannot_wash_away_normal_lane_recovery_deadline(fresh_pair_at_deadline):
    rig=Rig('parking_search');rig.step()
    for _ in range(3):rig.step(lambda t,i:lane(t,state='missing',fid=i))
    nav=(lane(.5,fid=5) if fresh_pair_at_deadline else lane(.4,state='missing',fid=4))
    result=rig.step(nav)
    assert rig.flow.stage=='fault' and '0.4 秒' in result['reason']
    assert result['action']=='neutral' and result['esc_us']==1500
    assert rig.guard.esc==1500 and rig.guard.actual_steering==1610


def test_new_red_reaches_guard_while_the_lane_camera_frame_is_repeated():
    rig=Rig('traffic');nav=lane(.1,fid=1)
    assert rig.step(nav)['esc_us']>1500
    duplicate=rig.step(nav)
    assert duplicate['fresh_control'] is False and len(rig.sent)==1
    result=rig.step(nav,dt=.08,color='red')
    assert result['action']=='brake' and result['esc_us']==1400
    assert result.get('fresh_control') is not False
    assert rig.guard.esc==1400 and rig.sent[-1]['action']=='brake'


@pytest.mark.parametrize('stage',['parking_search','cones'])
def test_urgent_outward_steering_withdrawal_is_centered_neutral_through_flow_and_guard(stage):
    rig=Rig(stage)
    for _ in range(3):rig.step()
    for _ in range(5):rig.step(lambda t,i:lane(t,fid=i,near=.9,far=.9))
    assert rig.guard.actual_steering>1610
    result=rig.step(lambda t,i:lane(t,fid=i,near=.1,far=.1,
        boundary_correction='right',boundary_margin_warning=True))
    assert result['boundary_output_withdrawal']
    assert result['action']=='neutral' and result['esc_us']==1500 and result['steering_us']==1610
    assert rig.guard.esc==1500 and rig.guard.actual_steering==1610


def test_two_registered_true_visual_exit_estimates_and_continuous_return_reach_parking_search():
    rig=Rig('cones')
    for _ in range(3):rig.step()
    assert rig.t==.3 and rig.guard.esc>1500  # No repeated three-second ESC arming.
    for identifier,side in (('secondary-A','left'),('secondary-B','right')):
        for y in (.55,.63,.71,.73,.76):
            result=rig.step(lambda t,i:ground_nav(t,identifier,side=side,y=y,fid=i))
        assert rig.flow.controller.pass_gate_hits==3
        assert not rig.flow.controller.pass_evidence['visual_estimate_ready']
        for index in range(3):
            result=rig.step(lambda t,i:exit_nav(t,identifier,side=side,index=index,fid=i))
    assert rig.flow.controller.count==2 and rig.flow.controller.mission.completion_ready()
    assert rig.flow.stage=='cones' and not result['physical_tail_verified']
    for _ in range(3):
        result=rig.step()
        assert rig.flow.stage=='cones'
    result=rig.step()
    assert rig.flow.stage=='parking_search' and result['esc_us']>1500
    assert rig.flow.passed_cone_ids=={'secondary-A','secondary-B'}
    assert rig.guard.esc==result['esc_us'] and not rig.guard.fault


def test_actual_navigation_mean_reference_counterexample_stays_unconfirmed_in_flow():
    rig=Rig('cones');scene=Scene((.875,.875,.5),.4)
    def observed(t,i,identifier=None,y=None):
        return scene.row(t,y,id=identifier or 'secondary-unused')[1]
    for _ in range(3):rig.step(lambda t,i:observed(t,i))
    for identifier in ('secondary-A','secondary-B'):
        for y in (.55,.63,.73,.75,.76):
            rig.step(lambda t,i:observed(t,i,identifier,y))
        assert rig.flow.controller.pass_gate_hits==0
        for _ in range(14):rig.step(lambda t,i:observed(t,i))
    centered=Scene((.5,.5,.5))
    for _ in range(6):rig.step(lambda t,i:centered.row(t)[1],layout=True)
    assert rig.flow.controller.count==0 and rig.flow.stage=='cones'
    assert not rig.flow.controller.mission.completion_ready()
    assert rig.flow.payment_started is None
    assert rig.flow.controller.mission.diagnostics()['mission_unresolved']
