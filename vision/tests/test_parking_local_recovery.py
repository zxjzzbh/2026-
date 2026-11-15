"""Synthetic regressions for near-entry correction and stopped-bay recovery."""
import math

import pytest

from carvision.parking_mission import contained, layout, plan_entry
from carvision.parking_control import curvature_for_pwm
from carvision.parking_task import ParkingTask
from test_parking_bay import sample, slots
from test_parking_path import measured_settings, measured_cones, selected_mission


@pytest.mark.parametrize('side,blocked,sign', [('right','left',1),('left','right',-1)])
@pytest.mark.parametrize('dx,yaw', [(0.,0.),(.002,0.),(0.,.002),(-.02,-.02)])
def test_near_entry_small_correction_parks_using_final_pwm(side,blocked,sign,dx,yaw):
    settings=measured_settings()
    _,boxes=layout(slots(blocked),settings)
    box=boxes[side]
    pose={'x_m':(box[0]+box[2])/2+sign*dx,'y_m':box[1]-.1,'yaw_rad':sign*yaw}
    path=plan_entry(slots(blocked),side,pose,settings,max_control_gap_s=.75)
    assert contained(path[-1],box,settings,.03)
    task=ParkingTask(settings,measured_cones())
    speed=0.
    for i in range(180):
        now=i*.1
        output=task.step(sample(i,now,dict(pose),blocked,speed),now)
        assert output['phase']!='fault',output
        if output['payment_ready']:
            break
        target=output.get('command_speed_mps',0.)
        speed+=max(-.04,min(.03,target-speed))
        curvature=curvature_for_pwm(output['steering_us'],settings)
        assert output['command_curvature_1pm']==pytest.approx(curvature)
        assert output['esc_us']<=1500 or output['esc_us']>=1570
        middle=pose['yaw_rad']+.05*speed*curvature
        pose['x_m']+=.1*speed*math.sin(middle)
        pose['y_m']+=.1*speed*math.cos(middle)
        pose['yaw_rad']+=.1*speed*curvature
    assert output['payment_ready'] and abs(speed)<=settings['stopped_speed_mps']
    assert contained(pose,box,settings,.03)


@pytest.mark.parametrize('pose',[
    {'x_m':0.,'y_m':1.9,'yaw_rad':0.},
    {'x_m':.52,'y_m':1.9,'yaw_rad':0.},
    {'x_m':.3075,'y_m':1.9,'yaw_rad':.34},
    {'x_m':.3095,'y_m':2.3,'yaw_rad':0.},
])
def test_late_entry_still_rejects_unreachable_or_sweeping_boundary(pose):
    with pytest.raises(ValueError):
        plan_entry(slots('left'),'right',pose,measured_settings(),max_control_gap_s=.75)


def inside_recovery():
    """Reach the bay with physically bounded observations; no state bypass."""
    mission=selected_mission()
    now=.6;fid=6
    for pose in mission.path[1:]:
        now+=.15;fid+=1
        output=mission.step(sample(fid,now,pose,speed=.1),now)
        assert output['phase']!='fault',output
    pose={**mission.path[-1],'x_m':mission.path[-1]['x_m']+.08}
    _,boxes=layout(slots('left'),mission.s)
    assert contained(pose,boxes['right'],mission.s,.03)
    mission.recover_control()
    return mission,pose,now,fid


def test_inside_recovery_uses_fresh_stopped_pose_instead_of_old_path_error():
    mission,pose,now,fid=inside_recovery()
    for index,delta in enumerate((.9,1.2,1.5,1.8,2.1),1):
        output=mission.step(sample(fid+index,now+delta,pose),now+delta)
        assert output['phase']!='fault',output
        assert output['action'] in ('hold','brake')
        if index<5:assert not output['payment_ready']
    assert output['payment_ready'] and output['four_wheels_inside']


@pytest.mark.parametrize('kind',['moving','outside','bad_yaw','jump','stale','blocked'])
def test_inside_recovery_does_not_confirm_bad_or_moving_feedback(kind):
    mission,pose,now,fid=inside_recovery()
    # Keep this displacement inside the continuity allowance, so the actual
    # body check (not the jump check) must reject the outer-line crossing.
    if kind=='outside':pose['x_m']=.46
    if kind=='bad_yaw':
        pose.update(x_m=mission.path[-1]['x_m'],yaw_rad=.15)
        _,boxes=layout(slots('left'),mission.s)
        assert contained(pose,boxes['right'],mission.s,.03)
    if kind=='jump':pose['x_m']=.1
    for index,delta in enumerate((.9,1.2,1.5,1.8,2.1),1):
        world=sample(fid+index,now+delta,pose,speed=.03 if kind=='moving' else 0.)
        if kind=='stale':world['pose']['captured_s']=now
        if kind=='blocked':
            world['slots'][0]['availability']='clear'
            world['slots'][1]['availability']='blocked'
        output=mission.step(world,now+delta)
        assert not output['payment_ready']
        if kind=='outside' and index==1:assert '边界' in output['reason']
    if kind in ('outside','jump','stale','blocked'):assert output['phase']=='fault'
