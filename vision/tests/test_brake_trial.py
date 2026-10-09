import json
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.brake_trial import BrakeTrialDrive, TrialCamera
from carvision.brake_parking import load_settings
from carvision.brake_controls import extend_controls

CONFIG = Path(__file__).parents[1]/'configs/brake-parking.json'


class Driver:
    def __init__(self):
        self.commands = []
        self.state = {'setup_token': 'boot', 'mode': 'disabled', 'boot_test_phase': 'esc_off',
                      'hardware_output': False, 'worker_started': False}

    def status(self):
        return dict(self.state)

    def request(self, action, payload):
        self.commands.append((action, payload))
        return self.status()


class DeferredThread:
    def __init__(self, **kwargs): pass
    def start(self): pass
    def join(self, **kwargs): pass


class Speech:
    def __init__(self): self.count = 0
    def start(self): self.count += 1
    def status(self): return {'state': 'command_sent' if self.count else 'not_requested'}
    def close(self): pass


@pytest.fixture
def trial(tmp_path, monkeypatch):
    monkeypatch.setattr('carvision.brake_trial.threading.Thread', DeferredThread)
    drive = Driver()
    now = [1.0]
    actuator_calls = []
    w = BrakeTrialDrive(drive, {}, CONFIG, tmp_path, clock=lambda: now[0],
                        actuator_factory=lambda: actuator_calls.append('created'), speech_factory=Speech)
    return w, drive, now, actuator_calls


def prepare(w, **changes):
    return w.request('crosswalk_prepare', {'client': 'page', 'setup_token': 'boot',
                     'esc_off': True, 'fb_mode_confirmed': True, **changes})


def owned(w, sequence=0, **changes):
    return {'client': 'page', 'run_token': w.token, 'trial_sequence': sequence, **changes}


def test_status_never_opens_hardware_and_manual_forward_still_reaches_driver(trial):
    w, drive, _, outputs = trial
    assert w.status()['crosswalk_trial']['can_prepare']
    assert w.status()['reverse_available'] is False
    w.request('command', {'motor': 'forward', 'steering': 'left'})
    assert drive.commands == [('command', {'motor': 'forward', 'steering': 'left'})]
    assert not outputs


def test_reverse_is_rejected_at_server_and_stops_previous_manual_motion(trial):
    w, drive, _, outputs = trial
    with pytest.raises(ValueError, match='F/B'):
        w.request('command', {'motor': 'reverse'})
    assert drive.commands == [('stop', {'stop_source': 'fb_reverse_rejected'})]
    assert not outputs


@pytest.mark.parametrize('change', [{'esc_off': False}, {'fb_mode_confirmed': False},
                                   {'setup_token': 'old'}, {'client': None}])
def test_preparation_requires_current_page_esc_off_and_fb_declaration(trial, change):
    w, _, _, outputs = trial
    with pytest.raises(ValueError): prepare(w, **change)
    assert not w.busy and not outputs


def test_prepare_explicitly_ends_manual_before_starting_parking_worker(trial):
    w, drive, _, outputs = trial
    drive.state.update(boot_test_phase='ready', mode='enabled', hardware_output=True)
    t=w.status()['crosswalk_trial']
    assert t['can_end_manual'] and t['can_prepare']
    prepare(w)
    assert not outputs and drive.commands[0][0]=='finish_test'
    assert w.phase=='preparing'


def test_start_and_heartbeat_require_owner_round_and_monotonic_sequence(trial):
    w, _, now, _ = trial
    prepare(w)
    for change in ({'client': 'other'}, {'run_token': 'old'}):
        with pytest.raises(ValueError): w.request('crosswalk_keepalive', owned(w, **change))
    w.phase = 'ready'
    w.request('crosswalk_start', owned(w, fb_mode_confirmed=True, esc_on_static=True, field_ready=True))
    assert w.phase == 'arming' and w.arm_until == now[0]+3
    before = w.lease_until
    now[0] += .1
    with pytest.raises(ValueError): w.request('crosswalk_keepalive', owned(w))
    assert w.lease_until == before
    w.request('crosswalk_keepalive', owned(w, 1))
    assert w.lease_until == now[0]+.5
    with pytest.raises(ValueError):
        w.request('crosswalk_start', owned(w, 2, fb_mode_confirmed=True, esc_on_static=True, field_ready=True))


def test_manual_commands_cannot_mix_with_trial_but_any_page_can_stop(trial):
    w, drive, _, _ = trial
    prepare(w)
    with pytest.raises(ValueError): w.request('command', {'motor': 'forward'})
    w.request('emergency', {'client': 'another-page'})
    assert w.stop.is_set() and w.phase == 'cancelled' and not drive.commands


def test_invalid_camera_fails_before_uart_is_opened(trial):
    w, _, _, outputs = trial
    prepare(w)
    w._work()
    assert w.phase == 'fault' and not w.busy and not outputs


@pytest.mark.parametrize('lose_page', [False, True])
def test_console_run_closes_actuator_on_completion_or_page_loss(trial, lose_page):
    w, _, now, _ = trial
    now[0] = 0
    writes = []

    class Actuator:
        def send(self, intent):
            writes.append(dict(intent))
            return {'hardware_output': False}
        def close(self): writes.append({'action': 'close'})

    class Reader:
        jpeg = None
        def __init__(self, *_): self.n = 0
        def read(self, unused):
            if w.phase=='complete':w.stop.set()
            self.n += 1
            now[0] = self.n/10
            if not lose_page or self.n < 3:
                w.lease_until = now[0]+.5
            return {'fresh': True, 'frame_id': self.n, 'captured_s': now[0],
                    'image_size': [480, 360], 'candidate': True,
                    'far_edge_y_normalized': .55 if lose_page or self.n < 3 else .76,
                    'lane_valid': True, 'lane_offset': 0, 'motion': {'valid': True, 'moving': False}}

    w.reader_factory = Reader
    w.actuator_factory = Actuator
    prepare(w)
    # The request path's real 3-second delay is checked separately above.
    w.phase, w.arm_until = 'arming', .1
    w._work()
    assert writes[-1]['action'] == 'close' and not w.busy
    assert any(row['action'] == 'search' for row in writes)
    assert w.phase == ('fault' if lose_page else 'complete')
    assert w.speech.count == (0 if lose_page else 1)
    report = json.loads((w.folder/'summary.json').read_text(encoding='utf-8'))
    assert report['phase'] == w.phase and not report['physical_stop_verified']


def test_camera_reader_reuses_raw_capture_and_does_not_renew_on_duplicates():
    ok, jpeg = cv2.imencode('.jpg', np.zeros((360, 480, 3), np.uint8)); assert ok
    state = SimpleNamespace(condition=threading.RLock(), raw_jpeg=jpeg.tobytes(),
                            raw_frame_id=1, raw_received_ms=1000, state='ok')
    r = TrialCamera({'secondary': state}, load_settings(CONFIG))
    assert r.read(1)['frame_id'] == 1
    assert r.read(1.1) is None
    with pytest.raises(ValueError): r.read(1.3)


def test_console_injection_preserves_manual_code_and_uses_real_trial_endpoints():
    original = '''<script>(()=>{let gamepadControl=null,pwmControl=null;
function render(s){renderLegacy(s);}
async function heartbeat(){originalHeartbeat();}
})();</script>'''
    controls = SimpleNamespace(SCRIPT=original, HTML='<p>existing camera console</p>')
    extend_controls(controls)
    assert 'originalHeartbeat();' in controls.SCRIPT
    assert 'if(renderBrake(s))return;' in controls.SCRIPT
    assert 'crosswalk_prepare' in controls.SCRIPT and 'crosswalk_start' in controls.SCRIPT
    assert 'trial_sequence:++brakeSequence' in controls.SCRIPT
    assert 'crosswalk-stopped' not in controls.HTML and 'crosswalk-continue' not in controls.HTML
    assert controls.HTML.startswith('<p>existing camera console</p>')
    assert 'type="checkbox"' not in controls.HTML
    assert 'crosswalk-save-distance' in controls.HTML


def test_distance_save_persists_without_opening_hardware(trial):
    w,drive,_,outputs=trial
    w.request('crosswalk_settings',{'client':'page','settings_revision':w.settings_revision,'brake_distance_cm':45})
    assert w.brake_reference['distance_m']==.45 and not outputs
    restored=BrakeTrialDrive(drive,{},CONFIG,w.root)
    assert restored.settings['brake_trigger_distance_m']==.45


@pytest.mark.parametrize('cm',[20,66,float('nan'),True])
def test_distance_rejects_outside_calibration_without_persisting(trial,cm):
    w,_,_,outputs=trial
    with pytest.raises(ValueError):
        w.request('crosswalk_settings',{'client':'page','settings_revision':w.settings_revision,'brake_distance_cm':cm})
    assert not w.settings_path.exists() and not outputs


def test_running_round_freezes_distance_and_next_round_uses_saved_value(trial):
    w,_,_,_=trial;prepare(w);w.phase='ready'
    flags={'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True}
    w.request('crosswalk_start',owned(w,**flags))
    w.request('crosswalk_settings',{'client':'page','settings_revision':w.settings_revision,'brake_distance_cm':40})
    assert w.active_settings['brake_trigger_distance_m']==.55
    assert w.settings['brake_trigger_distance_m']==.4
    w.phase='complete'
    w.request('crosswalk_start',owned(w,1,**flags))
    assert w.active_settings['brake_trigger_distance_m']==.4 and w.round_number==2


def test_other_page_and_stale_settings_revision_cannot_overwrite(trial):
    w,_,_,_=trial;prepare(w)
    for client,revision in [('other',w.settings_revision),('page','old')]:
        with pytest.raises(ValueError):
            w.request('crosswalk_settings',{'client':client,'settings_revision':revision,'brake_distance_cm':40})
    assert w.settings['brake_trigger_distance_m']==.55
