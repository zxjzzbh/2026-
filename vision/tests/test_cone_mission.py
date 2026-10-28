"""Persistent task accounting and real navigation passage contracts; no I/O."""
import copy

import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_mission import ConeMissionLedger
from test_cone_passage import Scene


class Trial:
    def __init__(self,reference=(.3,.3,.3)):
        self.scene=Scene(reference)
        self.controller=ContinuousConeController(self.scene.s,0.,initialized=True,handoff_after_two=True)
        self.t=-.1
        for _ in range(3):self.step()

    def step(self,identifier=None,y=None,x=None,*,camera='secondary',pixels=True):
        self.t=round(self.t+.1,8)
        local=identifier.replace('primary-','secondary-',1) if identifier else 'secondary-unused'
        _,nav=self.scene.row(self.t,y,x,id=local)
        if camera=='primary':
            # A synthetic consistent primary nav contract. No primary accident
            # pixels/calibration are claimed; auxiliary counting must refuse it.
            nav=copy.deepcopy(nav);nav['source_camera']='primary'
            for key in ('cones','uncertain_cones','visual_cones','obstacle_diagnostics'):
                for entry in nav.get(key,[]):
                    if 'id' in entry:entry['id']=entry['id'].replace('secondary-','primary-',1)
                    if 'source_camera' in entry:entry['source_camera']='primary'
                    proof=entry.get('pass_evidence')
                    if proof:
                        proof['camera']='primary';proof['target_id']=entry['id']
        if not pixels:nav['visual_cones']=[]
        self.last_nav=nav
        self.last=self.controller.step(nav,self.t)
        return self.last

    def estimated_pass(self,identifier,*,camera='secondary',ground_pixels=True):
        for y in (.55,.63,.73,.75,.76):self.step(identifier,y,camera=camera,pixels=ground_pixels)
        if camera=='secondary':assert self.controller.pass_gate_hits==3
        for x,y in ((.80,.86),(.84,.90),(.88,.94)):
            self.step(identifier,y,x,camera=camera)
        return self.last

    def centered(self):
        self.scene=Scene((.5,.5,.5))
        for _ in range(6):self.step()
        return self.last


def register(ledger,identifier,start=0.):
    assert not ledger.observe('secondary',identifier,round(start*1000),start,start,credible=True)
    return ledger.observe('secondary',identifier,round((start+.1)*1000),start+.1,start+.1,credible=True)


def estimate(identifier,t=1.):
    return {'target_id':identifier,'source_camera':'secondary','frame_id':round(t*1000),
            'captured_s':t,'visual_estimate_ready':True,'gate_hits':3,'exit_frames':3}


def test_task_registration_requires_repeated_current_trusted_frames_not_one_or_duplicates():
    ledger=ConeMissionLedger()
    assert not ledger.observe('secondary','secondary-A',1,0.,0.,credible=True)
    assert not ledger.observe('secondary','secondary-A',1,0.,.1,credible=True)
    assert not ledger.registered('secondary','secondary-A')
    assert ledger.observe('secondary','secondary-A',2,.1,.1,credible=True)
    assert ledger.registered('secondary','secondary-A')
    ledger.observe('secondary',None,3,.2,.2,credible=False)
    ledger.observe('primary','primary-B',4,.3,.3,credible=True)
    assert [entry['id'] for entry in ledger.diagnostics()['mission_unresolved']]==['secondary-A']


def test_noncredible_gap_breaks_candidate_confirmation_but_never_resolves_registered_task():
    ledger=ConeMissionLedger()
    ledger.observe('secondary','secondary-A',1,0.,0.,credible=True)
    ledger.observe('secondary','secondary-A',2,.1,.1,credible=False)
    assert not ledger.observe('secondary','secondary-A',3,.2,.2,credible=True)
    assert ledger.observe('secondary','secondary-A',4,.3,.3,credible=True)
    for i in range(5,30):ledger.observe('secondary',None,i,i/10,i/10,credible=False)
    assert len(ledger.diagnostics()['mission_unresolved'])==1
    assert not ledger.completion_ready()


def test_capacity_overflow_preserves_records_and_latches_completion_without_raising_stop():
    ledger=ConeMissionLedger(capacity=2)
    assert register(ledger,'secondary-A') and register(ledger,'secondary-B',.2)
    assert not register(ledger,'secondary-C',.4)
    assert ledger.overflow and len(ledger.records)==2
    assert {entry['id'] for entry in ledger.diagnostics()['mission_unresolved']}=={'secondary-A','secondary-B'}
    assert ledger.commit_estimate('secondary','secondary-A',estimate('secondary-A'),1.)
    assert ledger.commit_estimate('secondary','secondary-B',estimate('secondary-B',1.1),1.1)
    assert not ledger.completion_ready() and ledger.diagnostics()['mission_overflow']
    assert ledger.diagnostics()['mission_first_overflow']['id']=='secondary-C'


@pytest.mark.parametrize('change',[{'source_camera':'primary'},{'target_id':'secondary-B'},
                                   {'visual_estimate_ready':False},{'captured_s':0.},
                                   {'frame_id':None}])
def test_registered_task_cannot_be_resolved_by_wrong_or_stale_estimate(change):
    ledger=ConeMissionLedger();register(ledger,'secondary-A')
    evidence={**estimate('secondary-A'),**change}
    assert not ledger.commit_estimate('secondary','secondary-A',evidence,1.)
    assert ledger.diagnostics()['mission_unresolved'][0]['id']=='secondary-A'


def test_registered_unresolved_A_survives_handoff_and_two_later_real_estimates():
    run=Trial()
    run.step('secondary-A',.55);run.step('secondary-A',.60)
    assert run.controller.mission.registered('secondary','secondary-A')
    run.estimated_pass('secondary-B');run.estimated_pass('secondary-C')
    assert run.controller.count==2 and run.controller.passed_ids=={'secondary-B','secondary-C'}
    result=run.centered()
    assert result['phase']!='complete' and not result['mission_completion_ready']
    assert [entry['id'] for entry in result['mission_unresolved']]==['secondary-A']
    assert result['esc_us']>1500  # Contact continuation is independent of completion.


def test_one_frame_candidate_does_not_permanently_lock_two_confirmed_task_estimates():
    run=Trial();run.step('secondary-candidate',.55)
    assert not run.controller.mission.registered('secondary','secondary-candidate')
    run.estimated_pass('secondary-B');run.estimated_pass('secondary-C')
    result=run.centered()
    assert result['phase']=='complete' and result['mission_completion_ready']
    assert result['mission_registered_count']==2 and not result['mission_unresolved']
    assert not result['physical_tail_verified'] and not result['tail_clearance_verified']


def test_ground_candidate_and_one_second_without_current_exit_do_not_count_or_resolve():
    run=Trial()
    for y in (.55,.63,.73,.75,.76):run.step('secondary-A',y)
    assert run.controller.pass_evidence['visual_pass_candidate']
    for _ in range(15):result=run.step()
    assert run.controller.count==0 and not run.controller.passed_ids
    assert [entry['id'] for entry in result['mission_unresolved']]==['secondary-A']
    assert not result['physical_tail_verified'] and result['phase']!='complete'


def test_missing_visual_provenance_cannot_register_a_legacy_fixture_then_count_from_late_pixels():
    run=Trial();run.estimated_pass('secondary-A',ground_pixels=False)
    assert run.controller.count==0 and not run.controller.passed_ids
    assert run.last['mission_registered_count']==0
    assert run.last['last_pass_rejection']=='no_registered_current_secondary_visual_estimate'
    assert not run.last['mission_completion_ready']


def test_same_slice_contract_rejects_the_old_mean_reference_clearance_counterexample():
    run=Trial((.875,.875,.5));run.scene.fraction=.4
    for y in (.55,.63,.73,.75,.76):run.step('secondary-A',y)
    assert run.controller.side=='right' and run.controller.pass_gate_hits==0
    for x,y in ((.20,.86),(.15,.90),(.10,.94)):run.step('secondary-A',y,x)
    assert run.controller.count==0 and not run.controller.passed_ids


def test_cross_camera_same_fraction_never_merges_aliases_or_counts_auxiliary_as_another_cone():
    run=Trial();run.step('secondary-A',.55);run.step('secondary-A',.60)
    assert run.controller.mission.registered('secondary','secondary-A')
    run.estimated_pass('primary-B',camera='primary')
    assert run.controller.count==0 and not run.controller.passed_ids
    assert run.controller.target_aliases in ({'primary-B'},set())
    assert run.last['mission_camera']=='secondary'
    assert run.last['passage_evidence'].get('reason')=='primary_camera_does_not_count_task_cones'
    assert [entry['id'] for entry in run.last['mission_unresolved']]==['secondary-A']
    run.step('secondary-A',.55)
    assert run.controller.target=='secondary-A' and run.controller.pass_gate_hits==0
    assert run.controller.target_aliases=={'secondary-A'}


def test_auxiliary_view_cannot_double_count_a_secondary_passed_identity():
    run=Trial();run.estimated_pass('secondary-A')
    assert run.controller.count==1 and run.controller.passed_ids=={'secondary-A'}
    run.estimated_pass('primary-B',camera='primary')
    assert run.controller.count==1 and 'primary-B' not in run.controller.passed_ids
    assert run.last['mission_registered_count']==1 and not run.last['mission_completion_ready']


def test_continuous_manual_confirm_endpoint_cannot_increase_task_count():
    run=Trial()
    with pytest.raises(ValueError):run.controller.confirm_pass()
    assert run.controller.count==run.controller.manual_count==0
