"""Replay frame archives through the practice camera and controller without vehicle I/O."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'vision/src'))

from carvision.cone_execution import ConeRun
from carvision.cone_low_speed import DualLowSpeedNavigator, PROFILE
from carvision.cone_observer import ConeCamera, practice_reference
from carvision.cone_planning import load_settings
from carvision.cone_replay import read_jpeg
from carvision.motion_cache import GroundMotionCache
from carvision.web_preview import PreviewState


class RecordingOutput:
    """Collect requested output only. No serial port or camera device is opened."""
    run = False

    def send(self, intent):
        return {'hardware_output': False, 'esc_us': intent['esc_us']}

    def race_interrupted(self):
        return False

    def close(self):
        pass


def replay(folder, surface=None):
    config_dir = ROOT / 'vision/configs'
    settings = load_settings(config_dir / 'cone-trial.json')
    if surface is None and (folder / 'settings.json').exists():
        recorded = json.loads((folder / 'settings.json').read_text(encoding='utf-8-sig'))
        surface = recorded.get('cone', {}).get('scene', {}).get('surface')
    settings['scene']['surface'] = surface or settings['scene']['surface']
    settings = practice_reference(settings, config_dir)
    lines = (folder / 'frames.jsonl').read_text(encoding='utf-8-sig').splitlines()
    frames = sorted((json.loads(line) for line in lines if line.strip()), key=lambda row: row['captured_s'])
    if not frames:
        raise ValueError('录像没有帧')
    now = frames[0]['captured_s']
    states = {camera: PreviewState() for camera in ('primary', 'secondary')}
    motion = GroundMotionCache(states, clock=lambda: now)
    session = ConeRun(lambda _: RecordingOutput())
    base = {'mode': 'disabled', 'boot_test_phase': 'esc_off', 'setup_token': 'replay'}
    setup = {'profile': PROFILE, 'lane_feedback': 'weighted_rows', 'client': 'replay',
             'setup_token': 'replay', 'esc_off': True, 'fb_mode_confirmed': True}
    session.prepare(settings, base, setup, now)
    reader = ConeCamera(states, settings, lambda: now, turn_aware=True)
    reader.navigator = DualLowSpeedNavigator(session.s)
    reader.navigator.set_single_recovery_limit(1.)
    body = {'client': 'replay', 'run_token': session.token}
    counts = Counter()
    phases = Counter()
    recoveries = []
    started = False
    try:
        for i, row in enumerate(frames):
            now = row['captured_s']
            states[row['camera']].publish_raw(read_jpeg(folder, row), SimpleNamespace(
                frame_id=row['frame_id'], receive_monotonic_ms=now * 1000))
            previous = session.intent or {}
            target = previous.get('target_id')
            reader.navigator.set_avoidance_target(previous.get('target_camera') if target else None, target or None)
            sample = reader.read(render=False)
            if sample is None:
                continue
            nav = reader.navigation
            counts['samples'] += 1
            counts['weighted_lane_available'] += int('weighted_ego_fraction' in nav)
            motion.sample_once()
            if session.active:
                session.tick(nav, now, motion=motion.latest)
            if session.active:
                session.heartbeat({**body, 'heartbeat_sequence': i}, now)
                if not started and session.active and session.status(settings, base, now)['can_start']:
                    session.command('cone_start', {**body, 'trial_sequence': i,
                        'fb_mode_confirmed': True, 'esc_on_static': True, 'field_ready': True}, now)
                    started = True
            intent = session.intent or {}
            if intent.get('recovery_trigger'):
                recoveries.append({'frame':row['frame_id'], 'reason':intent['recovery_trigger']})
            phases[session.phase] += 1
            counts['requested_forward'] += int(intent.get('esc_us', 1500) > 1500)
            counts['weighted_steering'] += int(intent.get('steering_policy') == 'weighted_nine_rows'
                                              and intent.get('esc_us', 1500) > 1500)
        return {'clip': folder.name, 'surface': settings['scene']['surface'], 'frames': len(frames),
                **counts, 'start_gate_passed': started, 'last_phase': session.phase,
                'last_reason': session.reason, 'phases': dict(phases), 'recoveries':recoveries,
                'hardware_output': False}
    finally:
        if session.active:
            session.cancel('回放结束', now=now)
        reader.close()
        motion.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path, nargs='+')
    parser.add_argument('--surface', choices=('red_running_track','blue_running_track','indoor_white_line_track'))
    args = parser.parse_args()
    for folder in args.recording:
        print(json.dumps(replay(folder, args.surface), ensure_ascii=False))
