from pathlib import Path
import struct

import pytest

from carvision.cone_low_speed import LowSpeedNavigator, trial_settings
from carvision.cone_observer import practice_reference
from carvision.cone_planning import load_settings
from carvision.brake_output import write_parking_pulse
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.rasadapter5 import RasAdapter
from test_cone_continuous import observation, warmed
from test_cone_observed_corridor_integration import IMAGE, row


CONFIG = Path(__file__).resolve().parents[1] / 'configs'


def practice():
    base = load_settings(CONFIG / 'cone-trial.json')
    return trial_settings(practice_reference(base, CONFIG), {'lane_feedback': 'weighted_rows'})


def test_reference_from_clean_install_keeps_local_geometry_and_projection():
    base = load_settings(CONFIG / 'cone-trial.json')
    reference = practice_reference(base, CONFIG)
    assert base['ground']['homography'] is None and base['geometry']['lane_width_m'] == 1.23
    assert reference['ground']['historical_verified'] and reference['geometry']['lane_width_m'] == 1.17
    base['scene']['surface'] = 'indoor_white_line_track'
    assert practice_reference(base, CONFIG)['geometry']['lane_width_m'] == 1.25
    reference['ground'].update(verified=True, historical_verified=False)
    reference['geometry'].update(measured=True, lane_width_m=1.23)
    current = practice_reference(reference, CONFIG)
    assert current == reference and current is not reference
    assert trial_settings(current, {'lane_feedback':'weighted_rows'})['ground'] == reference['ground']


@pytest.mark.parametrize('direction', [-1, 1])
def test_mirrored_bends_steer_inward_on_current_frame_only(direction):
    settings = practice()
    nav = LowSpeedNavigator(settings)
    data = row(settings, 1., 1, wide=True)
    road = data['road_context']
    # Both ends are centred. Only the intermediate visible rows curve sideways.
    for edge in ('left_inner', 'right_inner'):
        for point in road[edge]:
            y = (point[1] - 180) / 80
            point[0] += direction * 20 * 4*y*(1-y)
    measured = nav.update(IMAGE, data, 1.)
    assert measured['line_state'] == 'paired'
    assert measured['ego_near_fraction'] == pytest.approx(.5)
    assert direction*(.5-measured['weighted_ego_fraction']) > .04
    ctl = warmed(settings)
    observed = observation(3.1, cones=[], weighted_ego_fraction=measured['weighted_ego_fraction'],
                           boundary_observed=True, ego_near_fraction=.5, ego_far_fraction=.5)
    command = ctl.step(observed, 3.1)
    assert command['steering_policy'] == 'weighted_nine_rows'
    assert direction*(1610-command['steering_us']) > 0 and command['esc_us'] > 1500
    lost = {**data, 'captured_s':1.1, 'frame_id':2, 'road_context':{**road, 'valid':False}}
    assert 'weighted_ego_fraction' not in nav.update(IMAGE, lost, 1.1)


def test_practice_motor_packets_and_release_without_physical_uart():
    settings = practice()
    ctl = warmed(settings)
    packets = []
    adapter = RasAdapter()
    def capture(function, data):
        assert function == 4
        _, _, _, channel, pulse = struct.unpack('<BHBBH', data)
        packets.append((channel, pulse))

    adapter.send = capture
    guard = ConePulseGuard(actuator_settings(settings),
        lambda channel, pulse: write_parking_pulse(adapter, channel, pulse, forward_limit_us=1625),
        mode='F/B')
    for i, x in enumerate((.5,.51,.52,.495,.51,.70)):
        t = 3.1 + i*.1
        data = observation(t)
        data['cones'][0].update(fraction=x,near=False,y_fraction=.55)
        intent = ctl.step(data,t)
        guard.accept({**intent,'sequence':i,'sent_s':t,'control_profile':'cone'},t)
    assert any(channel == 4 and pulse > 1500 for channel,pulse in packets)
    assert intent['steering_phase']=='route_recheck' and guard.esc==1500
    assert (3,1610) in packets[-2:]
    guard.abort('operator stop',3.75)
    guard.tick(4.0)
    assert guard.esc==1500
