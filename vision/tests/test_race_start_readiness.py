"""Stage-scoped competition entry; real configs, fake threads/output only."""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision import race_flow_console
from carvision.race_flow import RaceFlowController, load_bundle
from carvision.parking_control import parking_readiness
from test_race_flow import Frames, measured_bundle, parking_sample


ROOT = Path(__file__).resolve().parents[2]


class Base:
    def __init__(self, **changes):
        self.state = {'mode':'disabled', 'hardware_output':False, 'worker_started':False,
                      'pending_hardware_start':False, 'boot_test_phase':'esc_off',
                      'setup_token':'boot'}
        self.state.update(changes)
        self.calls = []

    def status(self):
        return copy.deepcopy(self.state)

    def request(self, action, payload):
        self.calls.append((action, copy.deepcopy(payload)))
        return self.status()


class InertThread:
    """Capture the authorized transition without running its hardware worker."""
    def __init__(self, **kwargs):
        self.kwargs, self.started = kwargs, False

    def start(self):
        self.started = True

    def is_alive(self):
        return False

    def join(self, **kwargs):
        pass


def entry_payload(**changes):
    payload = {'client':'operator', 'setup_token':'boot', 'stationary_confirmed':True,
               'in_switch_zone':True, 'fb_mode_confirmed':True}
    payload.update(changes)
    return payload


@pytest.fixture
def make_drive(tmp_path, monkeypatch):
    def create(*, bundle=None, base=None, live=True):
        bundle = load_bundle(ROOT) if bundle is None else bundle
        monkeypatch.setattr(race_flow_console, 'load_bundle', lambda *args:copy.deepcopy(bundle))
        monkeypatch.setattr(race_flow_console.threading, 'Thread', InertThread)
        drive = race_flow_console.RaceFlowDrive(base or Base(), {}, tmp_path, live=live,
            clock=lambda:1., actuator_factory=lambda _:pytest.fail('status/entry must not open hardware'))
        return drive, bundle
    return create


def test_thirteen_real_parking_gaps_allow_stage_scoped_entry_without_fake_calibration(make_drive):
    drive, bundle = make_drive()
    before = copy.deepcopy(bundle)
    for _ in range(3):
        status = drive.status()
        report = status['race_flow']
        assert report['start_readiness']['ready'] and report['can_enter']
        assert report['start_readiness']['scope'] == 'through_cones'
        assert not report['start_readiness']['parking_ready']
        assert not report['readiness']['ready']
        assert len(report['readiness']['missing']) == 13
        assert report['parking_missing'] == report['readiness']['missing']
        assert not report['hardware_output'] and not status['hardware_output']
    assert bundle == before and drive.bundle['parking']['ground'] is None
    assert not drive.drive.calls and drive.thread is None

    entered = drive.request('race_enter', entry_payload())['race_flow']
    assert entered['active'] and entered['stage'] == 'handoff'
    assert not entered['hardware_output'] and drive.actuator is None
    assert drive.thread.started
    assert not entered['readiness']['ready'] and len(entered['parking_missing']) == 13
    assert drive.bundle == before and not drive.drive.calls


@pytest.mark.parametrize('change', [
    {'setup_token':'old'}, {'client':''}, {'stationary_confirmed':False},
    {'in_switch_zone':False}, {'fb_mode_confirmed':False},
])
def test_stage_scoped_entry_keeps_setup_identity_and_field_confirmations(make_drive, change):
    drive, _ = make_drive()
    with pytest.raises(ValueError):
        drive.request('race_enter', entry_payload(**change))
    assert not drive.active and drive.thread is None and drive.actuator is None


@pytest.mark.parametrize('base_state', [
    {'owner_client':'another-operator'},
    {'traffic_trial':{'active':True}},
    {'crosswalk_trial':{'active':True}},
    {'departure_trial':{'active':True}},
    {'parking_trial':{'active':True}},
    {'cone_trial':{'avoidance':{'active':True}}},
    {'cone_trial':{'short_test':{'active':True}}},
])
def test_entry_cannot_take_another_owner_or_parallel_task(make_drive, base_state):
    drive, _ = make_drive(base=Base(**base_state))
    with pytest.raises(ValueError):
        drive.request('race_enter', entry_payload())
    assert not drive.active and drive.thread is None


@pytest.mark.parametrize('gate', ['live_disabled', 'active', 'cleanup_error', 'old_worker'])
def test_nonparking_motion_gates_still_block_entry(make_drive, gate):
    drive, _ = make_drive(live=gate != 'live_disabled')
    if gate == 'active':
        drive.active = True
    elif gate == 'cleanup_error':
        drive.cleanup_error = 'neutral readback unconfirmed'
    elif gate == 'old_worker':
        drive.thread = SimpleNamespace(is_alive=lambda:True)
    assert not drive.status()['race_flow']['can_enter']
    with pytest.raises(ValueError):
        drive.request('race_enter', entry_payload())
    assert drive.actuator is None


def test_unloadable_configuration_cannot_become_start_ready(make_drive, monkeypatch):
    drive, _ = make_drive()
    drive.bundle, drive.error = None, 'configuration unavailable'
    report = drive.status()['race_flow']
    assert not report['start_readiness']['ready'] and not report['can_enter']
    def unavailable(*args):
        raise ValueError('configuration unavailable')
    monkeypatch.setattr(race_flow_console, 'load_bundle', unavailable)
    with pytest.raises(ValueError):
        drive.request('race_enter', entry_payload())
    assert not drive.active and drive.thread is None


def test_unverified_parking_never_gets_a_forward_whitelist_or_payment(make_drive, monkeypatch):
    drive, bundle = make_drive()
    captured = {}
    def capture(settings, **kwargs):
        captured.update(copy.deepcopy(settings))
        return SimpleNamespace(run=False)
    monkeypatch.setattr(race_flow_console, 'ConeTrialActuator', capture)
    drive._make_actuator(bundle)
    assert 'parking_forward_us' not in captured

    controller = RaceFlowController(bundle, 0.)
    controller._go('parking_search', 0., 'offline stage-boundary test')
    frames = Frames(controller)
    assert frames.step(layout=True)['esc_us'] == 1400
    assert controller.stage == 'parking_entry'
    for _ in range(190):
        output = frames.step(parking=parking_sample)
        assert output['esc_us'] <= 1500
        assert controller.stage not in ('payment', 'complete')
        assert not any(event['event']=='payment_window_opened' for event in output['events'])
        if controller.stage == 'fault':
            break
    assert controller.stage == 'fault' and controller.payment_started is None
    assert controller.status(frames.t)['parking_missing']


def test_measured_parking_enables_full_scope_and_can_complete_payment(make_drive):
    calibrated = measured_bundle(load_bundle(ROOT))
    assert parking_readiness(calibrated)['ready']
    drive, _ = make_drive(bundle=calibrated)
    report = drive.status()['race_flow']
    assert report['readiness']['ready'] and report['can_enter']
    assert report['start_readiness']['ready'] and report['start_readiness']['parking_ready']
    assert report['start_readiness']['scope'] == 'full_race'
    assert report['parking_missing'] == [] and not report['hardware_output']

    controller = RaceFlowController(calibrated, 0.)
    controller._go('parking_search', 0., 'offline calibrated stage test')
    frames = Frames(controller)
    frames.step(layout=True)
    for _ in range(10):
        frames.step(parking=parking_sample)
    assert controller.parking.selected == 'right' and controller.parking.path
    for pose in controller.parking.path[1:]:
        frames.step(dt=.15, parking=lambda t,i:parking_sample(t,i,pose,.10), moving=True)
    frames.until('payment', parking=lambda t,i:parking_sample(t,i,controller.parking.path[-1],0.))
    assert controller.payment_started == frames.t and frames.outputs[-1][1]['esc_us'] == 1500
    controller.report_paid(frames.t+1)
    assert controller.stage == 'complete' and controller.paid_in_time
