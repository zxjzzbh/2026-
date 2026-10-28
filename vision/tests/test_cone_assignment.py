"""Full-matrix cone association must not inherit identity through ambiguity."""
import itertools
import json

import pytest

from carvision.cone_vision import ImageConeTracker


@pytest.fixture
def settings():
    return {'image_size':[480,360],'confirm_frames':3,'confirm_s':.15,
        'track_timeout_s':.6,'max_frame_age_s':.25,
        'scene':{'surface':'indoor_white_line_track'}}


def detection(box,foot,*,color='blue',kind='cone_candidate'):
    return {'bbox_xyxy':list(box),'foot_pixel':list(foot),'kind':kind,
        'obstacle_color':color,'clipped':False,'ground_contact_usable':kind=='cone_candidate'}


def visible(rows):return [row for row in rows if row['visible']]


def warm_pair(settings,*,enabled=True):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=enabled)
    for frame in range(5):
        tracker.update([
            detection([100,200+3*frame,140,260+3*frame],[120,255+3*frame]),
            detection([125,210+3*frame,165,270+3*frame],[145,265+3*frame])],frame,frame*.1)
    return tracker


X=detection([108,222,148,282],[128,277])
Y=detection([100,212,140,272],[120,267])
BACKGROUND=detection([400,120,430,150],[415,145])


@pytest.mark.parametrize('order',list(itertools.permutations((X,Y,BACKGROUND))))
def test_full_matrix_resolves_exact_match_before_remaining_competitor(settings,order):
    tracker=warm_pair(settings)
    rows=visible(tracker.update(order,5,.5));by_box={tuple(row['bbox_xyxy']):row for row in rows}
    assert by_box[tuple(Y['bbox_xyxy'])]['id']=='secondary-1'
    assert by_box[tuple(X['bbox_xyxy'])]['id']=='secondary-2'
    for current in (X,Y):
        row=by_box[tuple(current['bbox_xyxy'])]
        assert row['confirmed'] and row['identity_revision']==0 and not row['identity_recovered']
    assert len({row['id'] for row in rows})==len(rows)
    assert tracker.latest_diagnostics['normal_matching']['matched']==2


def test_default_legacy_ordered_policy_is_preserved(settings):
    tracker=warm_pair(settings,enabled=False)
    rows=visible(tracker.update([X,Y],5,.5))
    assert rows[0]['id']=='secondary-1' and rows[1]['id']=='secondary-2'
    assert all('identity_revision' not in row for row in rows)


def test_tied_competitors_do_not_inherit_either_confirmed_identity(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True)
    for frame in range(5):
        y=240+6*frame
        tracker.update([detection([100,y-28,140,y+8],[120,y]),
            detection([110,y-28,150,y+8],[130,y])],frame,frame*.1)
    current=detection([105,236,145,272],[125,264])
    for frame,t in ((5,.5),(6,.6)):
        rows=visible(tracker.update([current],frame,t))
        assert len(rows)==1 and rows[0]['id'] not in ('secondary-1','secondary-2')
        assert not rows[0]['confirmed']
    assert not tracker.latest_diagnostics['recovered']


def test_stable_unambiguous_identity_does_not_increment_revision_or_restart_confirmation(settings):
    tracker=warm_pair(settings)
    for frame in range(5,12):
        rows=visible(tracker.update([Y,detection([125,222,165,282],[145,277])],frame,frame*.1))
        assert {row['id'] for row in rows}=={'secondary-1','secondary-2'}
        assert all(row['confirmed'] and row['identity_revision']==0 for row in rows)
        assert not tracker.latest_diagnostics['recovered']


def warm_positions(settings,xs):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True)
    for frame,y in enumerate((240,246,252,260,270)):
        tracker.update([detection([x-14,y-28,x+14,y+8],[x,y]) for x in xs],frame,frame*.1)
    return tracker


NEAR=detection([230,275,300,343],[258,315],kind='unresolved_blue')


def test_ninth_compatible_old_track_vetoes_apparently_unique_recovery(settings):
    tracker=warm_positions(settings,[216,20,50,80,110,340,380,430,260])
    sample=tracker._sample(NEAR,5,.5)
    assert all(tracker._compatible(key,sample,{'sample':None}) for key in ('secondary-1','secondary-9'))
    tracker.update([NEAR],5,.5)
    rows=visible(tracker.update([NEAR],6,.6))
    assert rows[0]['id'] not in ('secondary-1','secondary-9') and not rows[0]['confirmed']
    assert not tracker.latest_diagnostics['recovered']
    assert tracker.latest_diagnostics['rejected']['ambiguous']>=2
    assert len(tracker.latest_diagnostics['pending'])<=8
    assert tracker.latest_diagnostics['pending_omitted']>=1


def test_ninth_far_track_does_not_block_a_unique_local_recovery(settings):
    tracker=warm_positions(settings,[216,20,50,80,110,340,380,430,460])
    tracker.update([NEAR],5,.5)
    rows=visible(tracker.update([NEAR],6,.6))
    assert rows[0]['id']=='secondary-1' and rows[0]['identity_revision']==1
    assert rows[0]['identity_recovered'] and not rows[0]['ground_contact_usable']


def test_overcrowded_local_association_keeps_independent_normal_track_running(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True)
    xs=list(range(170,331,20));far=detection([20,120,45,155],[32,150])
    for frame,y in enumerate((240,246,252,260,270)):
        rows=[detection([x-30,y-34,x+30,y+16],[x,y]) for x in xs]+[far]
        tracker.update(rows,frame,frame*.1)
    crowded=detection([180,290,348,340],[264,315],kind='unresolved_blue')
    tracker.update([crowded,far],5,.5)
    rows=visible(tracker.update([crowded,far],6,.6))
    assert len(rows)==2 and len({row['id'] for row in rows})==2
    assert next(row for row in rows if row['bbox_xyxy']==far['bbox_xyxy'])['confirmed']
    assert not tracker.latest_diagnostics['recovered']
    assert tracker.latest_diagnostics['rejected']['association_capacity']==9
    json.dumps(tracker.latest_diagnostics,allow_nan=False)


def test_wrong_color_cannot_inherit_normal_confirmed_identity(settings):
    tracker=warm_pair(settings)
    red=detection(Y['bbox_xyxy'],Y['foot_pixel'],color='red')
    row=visible(tracker.update([red],5,.5))[0]
    assert row['id'] not in ('secondary-1','secondary-2') and not row['confirmed']


@pytest.mark.parametrize('gap,retained',[(.25,True),(.25000001,False)])
def test_capture_gap_boundary_preserves_or_expires_identity(settings,gap,retained):
    tracker=warm_pair(settings)
    row=visible(tracker.update([Y],5,.4+gap))[0]
    assert (row['id']=='secondary-1') is retained
    assert row['confirmed'] is retained


def test_repeated_frame_cannot_advance_normal_or_recovery_assignment(settings):
    tracker=warm_pair(settings);last=tracker.last_id
    with pytest.raises(ValueError):tracker.update([X,Y],last,.5)
    rows=visible(tracker.update([X,Y],5,.5))
    assert {tuple(row['bbox_xyxy']):row['id'] for row in rows}=={
        tuple(Y['bbox_xyxy']):'secondary-1',tuple(X['bbox_xyxy']):'secondary-2'}
