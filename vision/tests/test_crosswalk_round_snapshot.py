"""Run the real trial worker through two rounds without opening hardware."""
import copy
import json

import pytest

from carvision import brake_trial
from test_brake_trial import CONFIG, DeferredThread, Driver, Speech, owned, prepare
from test_crosswalk_profile_integration import (
    FEATURE, PIXELS, PROFILE_ID, profile as profile_fixture, write_profile,
)


@pytest.mark.parametrize('old_sample_has_candidate', [False, True])
def test_worker_rebuilds_reader_from_each_round_snapshot_and_discards_inflight_old_sample(
        tmp_path, monkeypatch, old_sample_has_candidate):
    # The camera fixture is independent of the physical car; both rounds use the
    # real profile loader, request path, worker and BrakeParkingController.
    write_profile(tmp_path, profile_fixture.__wrapped__())
    monkeypatch.setattr(brake_trial.threading, 'Thread', DeferredThread)
    now = [0.]
    frame = [0]
    readers, controllers, steps, writes, dropped = [], [], [], [], []
    saved = [False]
    real_controller = brake_trial.BrakeParkingController

    class TrackingController(real_controller):
        def __init__(self, settings, *args, **kwargs):
            self.test_trigger = settings['crosswalk_distance_trigger_m']
            controllers.append(self.test_trigger)
            super().__init__(settings, *args, **kwargs)

        def step(self, sample, t):
            steps.append((self.test_trigger, sample['reader_trigger'], sample['frame_id']))
            return super().step(sample, t)

    monkeypatch.setattr(brake_trial, 'BrakeParkingController', TrackingController)

    def start(sequence):
        w.request('crosswalk_start', owned(w, sequence, fb_mode_confirmed=True,
                  esc_on_static=True, field_ready=True))
        # Existing request tests cover the real three-second arming delay.
        w.arm_until = now[0]

    class Reader:
        jpeg = None

        def __init__(self, states, settings):
            self.settings = copy.deepcopy(settings)
            self.trigger = self.settings['crosswalk_distance_trigger_m']
            readers.append(self.trigger)

        def read(self, unused):
            frame[0] += 1
            assert frame[0] < 400, 'worker did not complete its first round'
            now[0] = frame[0] / 10
            w.lease_until = now[0] + .5  # The page remains connected throughout.
            sample = {
                'fresh': True, 'frame_id': frame[0], 'captured_s': now[0],
                'image_size': [480, 360], 'candidate': True,
                'source_camera': 'primary', 'calibration_id': PROFILE_ID,
                'image_feature': FEATURE,
                'near_edge_y_normalized': PIXELS['primary'][0 if self.trigger == .4 else 1] / 359,
                'near_limit_reached': False, 'source_changed': False,
                'lane_valid': True, 'lane_offset': 0,
                'motion': {'valid': True, 'moving': False},
                'reader_trigger': self.trigger,
            }
            if w.phase == 'complete':
                # Simulate a new HTTP start arriving while the previous reader
                # decodes its last frame. It must never reach the new controller.
                assert saved[0] and self.trigger == .4
                dropped.append(sample['frame_id'])
                start(1)
                sample['candidate'] = old_sample_has_candidate
                if not old_sample_has_candidate:
                    sample['near_edge_y_normalized'] = None
            return sample

    class Actuator:
        def send(self, intent):
            writes.append((w.round_number, copy.deepcopy(intent)))
            if intent['phase'] == 'ready' and w.round_number == 0:
                assert w.active_settings['crosswalk_distance_trigger_m'] == .4
                start(0)
            elif w.round_number == 1 and intent['action'] == 'brake' and not saved[0]:
                w.request('crosswalk_settings', {
                    'client': 'page', 'settings_revision': w.settings_revision,
                    'brake_distance_cm': 50,
                })
                saved[0] = True
                assert w.settings['crosswalk_distance_trigger_m'] == .5
                assert w.active_settings['crosswalk_distance_trigger_m'] == .4
            elif w.round_number == 2 and intent['action'] == 'brake':
                w.stop.set()
            return {'hardware_output': False}

        def close(self):
            writes.append(('closed', {}))

    w = brake_trial.BrakeTrialDrive(
        Driver(), {}, CONFIG, tmp_path, clock=lambda: now[0],
        reader_factory=Reader, actuator_factory=Actuator, speech_factory=Speech)
    # This test owns only the worker reader, not the separate idle UI reader.
    monkeypatch.setattr(w, '_distance_status', lambda: None)
    prepare(w)
    assert w.active_settings['crosswalk_distance_trigger_m'] == .4
    w._work()

    assert w.phase != 'fault', w.reason
    assert not w.busy and writes[-1][0] == 'closed'
    assert saved[0] and len(dropped) == 1
    assert readers == [.4, .4, .5]
    assert controllers == [.4, .5]
    assert all(controller_trigger == reader_trigger for controller_trigger, reader_trigger, _ in steps)
    assert dropped[0] not in [sample_id for _, _, sample_id in steps]
    assert any(round_id == 1 and intent['phase'] == 'done' for round_id, intent in writes)
    assert any(round_id == 2 and intent['action'] == 'brake' for round_id, intent in writes)
    assert not any(intent.get('action') in ('search', 'creep') for _, intent in writes)
    for round_number, distance in [(1, .4), (2, .5)]:
        snapshot = json.loads((w.folder / f'round-{round_number:03d}.settings.json').read_text(encoding='utf-8'))
        assert snapshot['crosswalk_distance_trigger_m'] == distance
    observations = [json.loads(line) for line in (w.folder / 'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    assert dropped[0] not in [row['sample']['frame_id'] for row in observations]
