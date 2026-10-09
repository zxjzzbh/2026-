from carvision.feedback_parking import FeedbackParkingController, ParkingLimits, readiness, check_file
from pathlib import Path


def calibration():
    # Explicit software fixture; not written into the real calibration profile.
    return {'esc_model':'test-only','esc_running_mode':'verified-forward-brake',
            'speed_feedback_source':'test-only','brake_evidence':'synthetic unit fixture',
            'speed_feedback_verified':True,'low_speed_control_verified':True,
            'distance_feedback_verified':True,'active_brake_verified':True,
            'brake_does_not_reverse_verified':True,'braking_deceleration_lower_bound_mps2':.2,
            'brake_response_upper_bound_s':.1,'minimum_stable_speed_mps':.02,
            'distance_verified_range_m':[.03,2]}


def obs(t,d=.6,v=.06,**kw):
    return {'camera_ok':True,'speed_valid':True,'speed_mps':v,'speed_captured_s':t,
            'distance_valid':d is not None,'distance_m':d,'distance_captured_s':t,
            'crosswalk_visible':d is not None,'search_region_valid':True,**kw}


def test_real_profile_is_blocked_not_an_armed_vehicle_configuration():
    root=Path(__file__).parents[1]
    result=check_file(root/'configs/parking-feedback.json')
    assert not result['ready'] and not result['hardware_output']
    assert 'esc_running_mode' in result['missing'] and 'active_brake_verified' in result['missing']


def test_unverified_brake_never_becomes_reverse_pwm():
    c=calibration();c['active_brake_verified']=False
    result=FeedbackParkingController(c).step(obs(0,d=.15,v=.1),0)
    assert result['action']=='neutral' and result['target_speed_mps']==0
    assert result['reverse_requested'] is False and 'esc_us' not in result


def test_search_and_approach_limit_actual_speed():
    c=FeedbackParkingController(calibration())
    assert c.step(obs(0,d=None,v=0),0)['target_speed_mps']==.08
    d=c.step(obs(.1,d=None,v=.2),.1)
    assert d['action']=='brake' and d['target_speed_mps']==0


def test_creep_goal_is_lower_near_the_target():
    c=FeedbackParkingController(calibration())
    assert c.step(obs(0,d=.35,v=.02),0)['target_speed_mps']==.03
    assert c.step(obs(.1,d=.30,v=.08),.1)['action']=='brake'


def test_lower_deceleration_causes_earlier_braking():
    weak=calibration();weak['braking_deceleration_lower_bound_mps2']=.025
    strong=FeedbackParkingController(calibration()).step(obs(0,d=.30,v=.08),0)
    weak_result=FeedbackParkingController(weak).step(obs(0,d=.30,v=.08),0)
    assert weak_result['action']=='brake'
    assert weak_result['estimated_stopping_distance_m']>strong['estimated_stopping_distance_m']


def test_stopping_requires_measured_stationary_interval_then_three_seconds():
    c=FeedbackParkingController(calibration());events=[]
    assert c.step(obs(0,d=.17,v=.05),0)['action']=='brake'
    for i in range(1,35):
        result=c.step(obs(i/10,d=.16,v=0),i/10)
        events.extend(result.get('events',[]))
        assert result['phase']!='done'
    assert c.step(obs(3.5,d=.16,v=0),3.5)['phase']=='done'
    assert events==[{'event':'speak','text':'我停车了啊'}]
    assert c.step(obs(3.6,d=.16,v=0),3.6)['target_speed_mps']==0


def test_rolling_during_hold_resets_timer_and_requests_brake():
    c=FeedbackParkingController(calibration())
    for i in range(11):c.step(obs(i/10,d=.16,v=0),i/10)
    r=c.step(obs(1.1,d=.16,v=.03),1.1)
    assert r['action']=='brake' and c.hold_since is None


def test_stale_feedback_latches_fault_and_never_accelerates():
    c=FeedbackParkingController(calibration())
    result=c.step(obs(1,d=.2,v=.08,speed_captured_s=0),1)
    assert result['phase']=='fault' and result['action']=='brake'
    assert c.step(obs(1.1,d=.5,v=0),1.1)['action']=='neutral'


def test_target_lost_after_seen_does_not_restore_search_speed():
    c=FeedbackParkingController(calibration());c.step(obs(0),0)
    result=c.step(obs(.1,d=None,v=.06),.1)
    assert result['phase']=='fault' and result['action']=='brake'


def test_early_stop_reapproaches_at_creep_not_cruise():
    c=FeedbackParkingController(calibration());c.phase='braking'
    result=c.step(obs(0,d=.45,v=0),0)
    assert result['action']=='set_speed' and result['target_speed_mps']==.03
    assert c.step(obs(.1,d=.44,v=.02),.1)['target_speed_mps']==.03


def test_overrun_never_commands_forward_correction():
    c=FeedbackParkingController(calibration())
    result=c.step(obs(0,d=-.05,v=.05),0)
    assert result['phase']=='fault' and result['action']=='brake'
    assert result['target_speed_mps']==0 and not result['reverse_requested']


def test_final_zone_must_be_inside_measured_distance_coverage():
    c=calibration();c['distance_verified_range_m']=[.25,.65]
    assert 'distance_coverage_including_final_stop_zone' in readiness(c)['missing']
