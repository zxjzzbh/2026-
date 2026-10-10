import pytest
from carvision.traffic_voting import DualSignalVotes, SignalFrameVote


def frame(camera, color, fid, t, box=None):
    return {'camera_id':camera,'frame_id':fid,'captured_s':t,'image_size':[480,360],
            'signal':{'state':color,'fixture_detected':color!='unknown',
                      'bbox_xyxy':box or [100,60,250,130]}}


@pytest.mark.parametrize('camera',['primary','secondary'])
@pytest.mark.parametrize('color',['red','green'])
def test_either_camera_independently_confirms_either_color(camera,color):
    v=DualSignalVotes(2,60)
    other='secondary' if camera=='primary' else 'primary'
    for i in range(21):
        result=v.update({camera:frame(camera,color,i,i/10),other:frame(other,'unknown',i,i/10)},i/10)
    assert result[color+'_confirmed']
    assert result[color+'_sources']==[camera]
    assert result['cameras'][other]['total_frames']==20


def test_no_cross_camera_pooling_and_shared_parameter_applies_to_both_colors():
    for color in ('red','green'):
        v=DualSignalVotes(2,50)
        for i in range(21):
            rows={c:frame(c,color if (i%10)<4 else 'off',i,i/10) for c in ('primary','secondary')}
            result=v.update(rows,i/10)
        assert not result[color+'_confirmed']
        assert all(r[color+'_percent']==40 for r in result['cameras'].values())
        v=DualSignalVotes(2,35)
        for i in range(21):result=v.update(rows:={c:frame(c,color if (i%10)<4 else 'off',i,i/10) for c in ('primary','secondary')},i/10)
        assert result[color+'_confirmed']


def test_red_wins_cross_camera_and_same_camera_temporal_conflicts():
    v=DualSignalVotes(2,35)
    for i in range(21):
        result=v.update({'primary':frame('primary','red',i,i/10),'secondary':frame('secondary','green',i,i/10)},i/10)
    assert result['conflict'] and result['red_confirmed'] and not result['green_confirmed']
    for i in range(21,42):
        result=v.update({'primary':frame('primary','red' if i%2 else 'green',i,i/10)},i/10)
    assert result['conflict'] and not result['green_confirmed']


def test_cached_frames_are_not_recounted_when_other_camera_is_faster():
    v=DualSignalVotes(2,50)
    for i in range(41):
        j=i//4
        result=v.update({'primary':frame('primary','red',i,i*.05),
                         'secondary':frame('secondary','green',j,j*.2)},i*.05)
    assert result['cameras']['primary']['total_frames']==40
    assert result['cameras']['secondary']['total_frames']==10
    assert result['conflict']


def test_missing_camera_drops_stale_votes_and_cannot_add_stale_green():
    v=DualSignalVotes(2,50)
    for i in range(21):
        result=v.update({'primary':frame('primary','green',i,i/10)},i/10)
    assert result['green_confirmed']
    result=v.update({'primary':frame('primary','green',20,2.0),
                     'secondary':frame('secondary','off',1,2.3)},2.3)
    assert not result['green_confirmed'] and not result['cameras']['primary']['valid']
    assert result['cameras']['primary']['total_frames']==0


def test_new_fixture_resets_only_that_camera_and_coordinates_are_not_compared_between_cameras():
    v=DualSignalVotes(2,50)
    for i in range(21):
        result=v.update({'primary':frame('primary','off',i,i/10,[10,10,70,40]),
                         'secondary':frame('secondary','red',i,i/10,[250,210,470,280])},i/10)
    assert result['red_sources']==['secondary']
    result=v.update({'primary':frame('primary','green',21,2.1,[200,100,320,190]),
                     'secondary':frame('secondary','red',21,2.1,[250,210,470,280])},2.1)
    assert result['red_sources']==['secondary'] and not result['green_confirmed']
    assert result['cameras']['primary']['total_frames']==1


def test_reset_requires_new_full_windows_and_yellow_blocks_green_completion():
    v=DualSignalVotes(2,50)
    for i in range(21):result=v.update({'primary':frame('primary','green',i,i/10)},i/10)
    assert result['green_confirmed']
    v.reset()
    result=v.update({'primary':frame('primary','green',21,2.1)},2.1)
    assert not result['green_confirmed']
    for i in range(22,43):
        result=v.update({'primary':frame('primary','green',i,i/10),
                         'secondary':frame('secondary','yellow',i,i/10)},i/10)
    assert result['green_sources']==['primary'] and not result['green_confirmed']


def test_one_color_threshold_does_not_accept_fake_geometry_or_wrong_source():
    v=DualSignalVotes(2,35)
    for i in range(25):
        bad=frame('secondary','red',i,i/10)
        result=v.update({'primary':bad,'secondary':frame('secondary','green',i,i/10,[0,0,0,0])},i/10)
    assert not result['red_confirmed'] and not result['green_confirmed']
