"""Bounded same-camera recovery uses two real observations, never predictions."""
import copy
import json

import pytest

from carvision.cone_vision import ImageConeTracker,iou


@pytest.fixture
def settings():
    return {'image_size':[480,360],'confirm_frames':3,'confirm_s':.15,
        'track_timeout_s':.6,'max_frame_age_s':.25,
        'scene':{'surface':'indoor_white_line_track'}}


def detection(x=216.,y=270.,*,box=None,**changes):
    return {'kind':'cone_candidate','obstacle_color':'blue',
        'bbox_xyxy':box or [int(x)-14,int(y)-28,int(x)+14,int(y)+8],
        'foot_pixel':[x,y],'ground_contact_usable':True,'clipped':False,**changes}


def warm(tracker,*,still=False,second=False,primary=False):
    ys=[300.,306.,312.,320.,330.] if primary else [240.,246.,252.,260.,270.]
    for i,y in enumerate(ys):
        rows=[detection(216. if still else 200.+4*i,270. if still else y)]
        if second:rows.append(detection(70.+4*i,y))
        output=tracker.update(rows,i,i*.1)
    return next(row for row in output if row['id']==tracker.camera+'-1')


def first_clip():
    return detection(246.,315.,box=[224,275,274,345],kind='unresolved_blue',
        clipped=True,ground_contact_usable=False)


def second_clip():
    return detection(263.,340.,box=[236,291,300,360],kind='unresolved_blue',
        clipped=True,ground_contact_usable=False)


def visible(rows):return [row for row in rows if row['visible']]


def test_two_real_fast_nearfield_observations_restore_old_id_without_ground_evidence(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True)
    old=warm(tracker)
    assert old['confirmed'] and old['identity_revision']==0
    assert iou(old['bbox_xyxy'],first_clip()['bbox_xyxy'])<.15
    first=tracker.update([first_clip()],5,.5)
    assert not next(row for row in first if row['id']==old['id'])['visible']
    assert len(visible(first))==1 and not visible(first)[0]['confirmed']
    restored=visible(tracker.update([second_clip()],6,.6))
    assert len(restored)==1
    current=restored[0]
    assert current['id']==old['id'] and current['confirmed'] and current['visible']
    assert current['bbox_xyxy']==second_clip()['bbox_xyxy']
    assert current['foot_pixel']==second_clip()['foot_pixel'] and current['seen_s']==.6
    assert current['clipped'] and not current['ground_contact_usable']
    assert current['identity_revision']==1 and current['identity_recovered']
    event=current['identity_recovery']
    assert event['evidence_frame_ids']==[5,6] and event['anchor_frame_id']==4
    assert event['absorbed_ids']==['secondary-2'] and event['ground_evidence'] is False
    assert 'secondary-2' not in tracker.tracks
    assert not any('passed' in key or 'verified' in key for key in current)
    json.dumps(tracker.latest_diagnostics,allow_nan=False)


def test_revision_persists_but_recovery_event_does_not_repeat(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5);tracker.update([second_clip()],6,.6)
    actual=detection(263.,340.,box=[236,290,300,355])
    current=visible(tracker.update([actual],7,.7))[0]
    assert current['id']=='secondary-1' and current['identity_revision']==1
    assert not current['identity_recovered'] and 'identity_recovery' not in current
    assert current['ground_contact_usable']  # Only this new actual detector row supplies it.


def test_recovered_tapered_candidate_still_has_no_ground_evidence_on_recovery_frame(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    one={**first_clip(),'kind':'cone_candidate','clipped':False,'ground_contact_usable':True}
    two={**second_clip(),'kind':'cone_candidate','clipped':False,'ground_contact_usable':True}
    tracker.update([one],5,.5);row=visible(tracker.update([two],6,.6))[0]
    assert row['id']=='secondary-1' and row['identity_recovered']
    assert not row['ground_contact_usable']


def test_legacy_opt_out_keeps_original_iou_identity_and_output(settings):
    tracker=ImageConeTracker('secondary',settings);old=warm(tracker)
    assert 'identity_revision' not in old
    tracker.update([first_clip()],5,.5)
    current=visible(tracker.update([second_clip()],6,.6))[0]
    assert current['id']!='secondary-1' and not current['confirmed']
    assert 'identity_recovered' not in current


def test_normal_iou_tracking_still_matches_immediately(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);old=warm(tracker)
    same={**detection(218.,274.),'clipped':True,'kind':'unresolved_blue','ground_contact_usable':False}
    row=visible(tracker.update([same],5,.5))[0]
    assert row['id']==old['id'] and row['confirmed'] and not row['identity_recovered']
    assert not row['ground_contact_usable']


def test_stationary_confirmed_blue_object_has_no_motion_recovery_permission(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker,still=True)
    tracker.update([first_clip()],5,.5);rows=visible(tracker.update([second_clip()],6,.6))
    assert not any(row['id']=='secondary-1' for row in rows)
    assert not tracker.latest_diagnostics['recovered']


@pytest.mark.parametrize('bad',[
    detection(50.,150.,box=[40,130,60,160]),
    {**first_clip(),'obstacle_color':'red','kind':'unresolved_red'},
    detection(450.,315.,box=[400,275,480,350],kind='unresolved_blue',clipped=True),
    detection(246.,350.,box=[170,190,330,360],kind='unresolved_blue',clipped=True),
])
def test_far_wrong_color_lateral_jump_or_huge_patch_cannot_steal_old_id(settings,bad):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([bad],5,.5)
    rows=visible(tracker.update([bad],6,.6))
    assert all(row['id']!='secondary-1' for row in rows)
    assert not tracker.latest_diagnostics['recovered']


def test_two_nearby_current_candidates_are_ambiguous_not_best_guess(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    other=detection(266.,315.,box=[250,280,300,350],kind='unresolved_blue',clipped=True,ground_contact_usable=False)
    for fid,t in ((5,.5),(6,.6)):
        rows=visible(tracker.update([first_clip(),other],fid,t))
        assert all(row['id']!='secondary-1' for row in rows)
        assert len(rows)==2 and len({row['id'] for row in rows})==2
    assert tracker.latest_diagnostics['rejected']['ambiguous']==1


def test_one_candidate_cannot_restore_two_old_tracks(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True)
    for i,y in enumerate((240.,246.,252.,260.,270.)):
        tracker.update([detection(200.+4*i,y),detection(244.+4*i,y)],i,i*.1)
    merged=detection(258.,315.,box=[224,275,300,345],kind='unresolved_blue',clipped=True,ground_contact_usable=False)
    for fid,t in ((5,.5),(6,.6)):
        rows=visible(tracker.update([merged],fid,t))
        assert len(rows)==1 and rows[0]['id'] not in ('secondary-1','secondary-2')
        assert not rows[0]['confirmed']
    assert not tracker.latest_diagnostics['recovered']


def test_recovery_and_other_normal_object_remain_one_to_one(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker,second=True)
    tracker.update([first_clip(),detection(90.,280.)],5,.5)
    rows=visible(tracker.update([second_clip(),detection(94.,290.)],6,.6))
    assert {row['id'] for row in rows}=={'secondary-1','secondary-2'}
    assert len(rows)==2 and sum(row['identity_recovered'] for row in rows)==1


def test_missing_update_breaks_consecutive_support_without_refreshing_deadline(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5)
    tracker.update([],6,.55)
    rows=visible(tracker.update([first_clip()],7,.6))
    assert all(row['id']!='secondary-1' for row in rows)
    pending=tracker.latest_diagnostics['pending'][0]
    assert pending['first_s']==.5 and pending['support_frames']==1


def test_object_observation_gap_does_not_qualify_as_continuous_nearfield_motion(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([],5,.5);tracker.update([],6,.6)
    current=visible(tracker.update([detection()],7,.7))[0]
    assert current['id']=='secondary-1'  # Ordinary IoU remains unchanged.
    tracker.update([first_clip()],8,.8)
    rows=visible(tracker.update([second_clip()],9,.9))
    assert all(row['id']!='secondary-1' for row in rows)
    assert not tracker.latest_diagnostics['recovered']


def test_timeout_cannot_restart_the_same_old_anchor(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5)
    tracker.update([],6,.6);tracker.update([],7,.7)
    for fid,t in ((8,.76),(9,.85)):
        rows=visible(tracker.update([second_clip()],fid,t))
        assert all(row['id']!='secondary-1' for row in rows)
        assert not tracker.latest_diagnostics['recovered']


def test_duplicate_or_reversed_frames_do_not_advance_pending_evidence(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5);before=copy.deepcopy(tracker.latest_diagnostics)
    for fid,t in ((5,.6),(4,.6),(6,.5)):
        with pytest.raises(ValueError):tracker.update([second_clip()],fid,t)
    assert tracker.latest_diagnostics==before
    current=visible(tracker.update([second_clip()],6,.6))[0]
    assert current['identity_recovery']['evidence_frame_ids']==[5,6]


def test_camera_gap_clears_recovery_and_does_not_create_visible_predictions(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5)
    rows=tracker.update([],6,.8)
    assert rows==[] and tracker.tracks=={}
    current=visible(tracker.update([second_clip()],7,.9))[0]
    assert current['id']!='secondary-1' and not current['confirmed']


def test_other_camera_cannot_reuse_secondary_recovery_state(settings):
    secondary=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(secondary)
    secondary.update([first_clip()],5,.5)
    primary=ImageConeTracker('primary',settings,near_field_recovery=True)
    rows=visible(primary.update([second_clip()],6,.6))
    assert rows[0]['id']=='primary-1' and not rows[0]['confirmed']
    assert not primary.latest_diagnostics['recovered']


def test_primary_recovery_uses_its_own_nearfield_history(settings):
    tracker=ImageConeTracker('primary',settings,near_field_recovery=True);warm(tracker,primary=True)
    one=detection(240.,347.,box=[226,329,270,360],kind='unresolved_blue',clipped=True,ground_contact_usable=False)
    two=detection(248.,354.,box=[236,335,284,360],kind='unresolved_blue',clipped=True,ground_contact_usable=False)
    assert iou(tracker.tracks['primary-1']['bbox_xyxy'],one['bbox_xyxy'])<.15
    tracker.update([one],5,.5);row=visible(tracker.update([two],6,.6))[0]
    assert row['id']=='primary-1' and row['identity_recovered']
    assert row['identity_recovery']['source_camera']=='primary'
    assert not row['ground_contact_usable']


def test_disabling_recovery_discards_pending_without_altering_real_track_identity(settings):
    tracker=ImageConeTracker('secondary',settings,near_field_recovery=True);warm(tracker)
    tracker.update([first_clip()],5,.5);tracker.set_near_field_recovery(False)
    assert not tracker._pending and tracker.tracks['secondary-1']['confirmed']
    assert 'identity_revision' not in tracker.tracks['secondary-1']
    assert visible(tracker.update([second_clip()],6,.6))[0]['id']!='secondary-1'
    with pytest.raises(ValueError):tracker.set_near_field_recovery('yes')
