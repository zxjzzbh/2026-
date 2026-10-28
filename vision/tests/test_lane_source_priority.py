"""Actual dual-view selection with explicit per-camera navigator snapshots."""
import copy
from pathlib import Path

import pytest

from carvision.cone_low_speed import DualLowSpeedNavigator,trial_settings
from carvision.cone_planning import load_settings


ROOT=Path(__file__).parents[1]


def view(camera,t,*,state='paired',fid=None,correction=None,**changes):
    near=.31 if camera=='secondary' else .72
    bounds=[[100,320],[70,390]] if camera=='secondary' else [[150,430],[30,470]]
    nav={'captured_s':t,'frame_id':round(t*1000) if fid is None else fid,
         'experimental':True,'ready':True,'action':'visual_trial_ready',
         'line_state':state,'line_missing':state=='missing','line_held':state=='single',
         'boundary_observed':state=='paired','ego_fraction':near,'ego_near_fraction':near,
         'ego_far_fraction':near+.02,'heading_error':-.02,'half_envelope':.2,
         'lane_bounds_px':bounds,'lane_rows':[190,270],
         'boundary_correction':correction,'cones':[{'id':camera+'-only','fraction':near+.1}],
         'view_marker':camera,'uncertain':[]}
    if state=='missing':
        nav.pop('lane_bounds_px');nav.pop('ego_fraction')
    if state=='single':nav.update(prediction_age_s=.1,line_age_s=.1)
    nav.update(changes)
    return nav


class FakeView:
    def update(self,image,row,now):return copy.deepcopy(row['snapshot'])


class Selection:
    def __init__(self):
        settings=trial_settings(load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json'))
        self.nav=DualLowSpeedNavigator(settings)
        self.nav.views={'primary':FakeView(),'secondary':FakeView()}

    def update(self,now,secondary=None,primary=None):
        rows={};images={}
        for camera,snapshot in (('secondary',secondary),('primary',primary)):
            if snapshot is None:continue
            images[camera]=object()
            rows[camera]={'captured_s':snapshot['captured_s'],'frame_id':snapshot['frame_id'],
                'road_context':{'valid':snapshot['line_state']=='paired','turn_aware':True},
                'snapshot':snapshot}
        return self.nav.update_views(images,rows,now)

    def on_primary(self):
        result=self.update(0.,view('secondary',0.,state='missing'),view('primary',0.))
        assert result['source_camera']=='primary'


def test_secondary_is_initial_main_view_when_both_current_pairs_are_available():
    run=Selection()
    result=run.update(.1,view('secondary',.1),view('primary',.1))
    assert result['source_camera']=='secondary' and result['view_marker']=='secondary'


def test_primary_is_temporary_until_three_fresh_secondary_pairs_span_150ms():
    run=Selection();run.on_primary()
    for t in (.1,.2):
        assert run.update(t,view('secondary',t),view('primary',t))['source_camera']=='primary'
    result=run.update(.3,view('secondary',.3),view('primary',.3))
    assert result['source_camera']=='secondary'


def test_three_frames_with_too_short_a_span_do_not_switch_early():
    run=Selection();run.on_primary()
    for t in (.1,.17,.24):
        assert run.update(t,view('secondary',t),view('primary',t))['source_camera']=='primary'
    assert run.update(.25,view('secondary',.25),view('primary',.25))['source_camera']=='secondary'


def test_duplicate_secondary_frames_never_count_as_independent_recovery():
    run=Selection();run.on_primary();cached=view('secondary',.1,fid=10)
    for t in (.1,.16,.22,.28):
        assert run.update(t,cached,view('primary',t))['source_camera']=='primary'
    assert run.nav.secondary_recovery.count==1
    assert run.update(.3,view('secondary',.3,fid=11),view('primary',.3))['source_camera']=='primary'
    assert run.update(.4,view('secondary',.4,fid=12),view('primary',.4))['source_camera']=='secondary'


@pytest.mark.parametrize('interruption',['absent','missing','single','stale'])
def test_missing_single_or_stale_secondary_breaks_the_recovery_sequence(interruption):
    run=Selection();run.on_primary()
    for t in (.1,.2):run.update(t,view('secondary',t),view('primary',t))
    when=.5 if interruption=='stale' else .3
    secondary=(None if interruption=='absent' else view('secondary',.2) if interruption=='stale'
               else view('secondary',when,state=interruption))
    assert run.update(when,secondary,view('primary',when))['source_camera']=='primary'
    assert run.nav.secondary_recovery.count==0
    for offset in (.1,.2):
        t=round(when+offset,8)
        assert run.update(t,view('secondary',t),view('primary',t))['source_camera']=='primary'
    t=round(when+.3,8)
    assert run.update(t,view('secondary',t),view('primary',t))['source_camera']=='secondary'


@pytest.mark.parametrize('primary_state',['absent','missing','single','stale','danger'])
def test_sole_reliable_secondary_pair_is_used_immediately_when_primary_is_unreliable(primary_state):
    run=Selection();run.on_primary();t=.4 if primary_state=='stale' else .1
    primary=(None if primary_state=='absent' else view('primary',0.) if primary_state=='stale'
             else view('primary',t,boundary_danger=True) if primary_state=='danger'
             else view('primary',t,state=primary_state))
    result=run.update(t,view('secondary',t),primary)
    assert result['source_camera']=='secondary' and result['line_state']=='paired'
    assert run.nav.secondary_recovery.count==1  # This safety choice does not wait for recovery voting.


def test_selected_view_geometry_and_correction_are_never_merged_with_other_camera():
    run=Selection();run.on_primary()
    secondary=view('secondary',.1,correction='left')
    primary=view('primary',.1,correction=None)
    result=run.update(.1,secondary,primary)
    assert result['source_camera']=='primary'
    for key in ('ego_fraction','ego_near_fraction','ego_far_fraction','heading_error',
                'lane_bounds_px','lane_rows','cones','view_marker','boundary_correction'):
        assert result[key]==primary[key]
    assert 'boundary_disagreement' in result
    assert [c['id'] for c in result['cones']]==['primary-only']
    assert run.nav.results['secondary']['ego_fraction']==secondary['ego_fraction']
    for t in (.2,.3):
        result=run.update(t,view('secondary',t,correction='left'),view('primary',t))
    assert result['source_camera']=='secondary' and result['boundary_correction']=='left'
    assert result['cones']==view('secondary',.3)['cones']


def test_initial_secondary_does_not_inherit_a_primary_only_direction_warning():
    run=Selection()
    result=run.update(.1,view('secondary',.1),view('primary',.1,correction='right'))
    assert result['source_camera']=='secondary' and result['boundary_correction'] is None
    assert 'boundary_disagreement' in result
