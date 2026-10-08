"""A new explicit round requires normal cleanup and rejects old direction packets."""
import importlib.util
import json
from pathlib import Path

import pytest

from carvision.manual_drive import DriveError, ManualDrive, RepeatableDrive, SimulationBackend


class Clock:
    now = 0.

    def __call__(self):
        return self.now


def setup():
    clock, workers, events = Clock(), [], []

    class Worker:
        def __init__(self):
            self.normal = False
            self.failed_close = False
            self.drive = ManualDrive(SimulationBackend(), clock=clock, settling_s=12,
                                     session_s=60, preparation_s=600)
            events.append('created')

        def status(self):
            return {**self.drive.status(), 'trial_completed_normally': self.normal}

        def request(self, action, payload):
            return self.drive.request(action, payload)

        def close(self):
            if self.failed_close:
                raise RuntimeError('cleanup failed')
            self.drive.close()
            events.append('closed')

    def factory():
        worker = Worker()
        workers.append(worker)
        return worker

    drive = RepeatableDrive(factory, {'ground_short_trial': True, 'test_session_s': 60})
    return drive, clock, workers, events


def request(drive, action, **values):
    return drive.request(action, {'client': 'round-test', 'bench_ready': True,
                                  'trial_token': drive.status()['trial_token'], **values})


def expire(drive, clock, workers):
    clock.now = workers[-1].drive.deadline+.01
    workers[-1].drive.tick()
    assert drive.status()['mode'] == 'expired'


def test_polling_never_starts_or_restarts_hardware_and_default_round_is_finite():
    drive, clock, workers, _ = setup()
    clock.now = 9999
    for _ in range(20):
        assert drive.status()['pending_hardware_start']
    assert not workers
    request(drive, 'enable'); drive.thread.join(timeout=2)
    assert workers[0].drive.deadline == clock.now+72
    expire(drive, clock, workers)
    workers[0].normal = True
    for _ in range(20):
        assert drive.status()['can_start_next_round']
    assert len(workers) == 1
    drive.close()


def test_expiry_without_confirmed_cleanup_and_fault_cannot_start_next_round():
    drive, clock, workers, _ = setup()
    request(drive, 'enable'); drive.thread.join(timeout=2)
    expire(drive, clock, workers)
    with pytest.raises(DriveError, match='正常结束'):
        request(drive, 'next_trial')
    workers[0].drive.mode = 'fault'
    workers[0].normal = True
    with pytest.raises(DriveError, match='正常结束'):
        request(drive, 'next_trial')
    assert len(workers) == 1
    drive.close()


def test_new_round_closes_old_owner_and_never_replays_held_direction():
    drive, clock, workers, events = setup()
    request(drive, 'enable'); drive.thread.join(timeout=2)
    clock.now = 12; workers[0].drive.tick()
    request(drive, 'command', sequence=1, motor='forward', steering='right')
    old_token = drive.status()['trial_token']
    expire(drive, clock, workers); workers[0].normal = True
    request(drive, 'next_trial', motor='forward', steering='right')
    drive.thread.join(timeout=2)
    assert events == ['created', 'closed', 'created']
    assert drive.status()['trial_number'] == 2
    assert drive.status()['trial_token'] != old_token
    assert workers[1].drive.motor == 'stop' and workers[1].drive.pulse == 1500
    clock.now += 12; workers[1].drive.tick()
    for action in ('enable', 'command', 'next_trial'):
        with pytest.raises(DriveError, match='本轮已更新'):
            drive.request(action, {'client': 'round-test', 'bench_ready': True,
                                   'trial_token': old_token, 'sequence': 100,
                                   'motor': 'forward', 'steering': 'right'})
    assert workers[1].drive.motor == 'stop'
    # Safety requests are accepted even from a previous round.
    drive.request('emergency', {'trial_token': old_token})
    assert drive.status()['mode'] == 'emergency'
    drive.close()


def test_next_round_requires_new_physical_ready_confirmation():
    drive, clock, workers, _ = setup()
    request(drive, 'enable'); drive.thread.join(timeout=2)
    expire(drive, clock, workers); workers[0].normal = True
    with pytest.raises(DriveError, match='停稳'):
        request(drive, 'next_trial', bench_ready=False)
    assert len(workers) == 1
    drive.close()


def test_cleanup_failure_latches_fault_without_creating_another_worker():
    drive, clock, workers, _ = setup()
    request(drive, 'enable'); drive.thread.join(timeout=2)
    expire(drive, clock, workers); workers[0].normal = True; workers[0].failed_close = True
    with pytest.raises(DriveError, match='清理失败'):
        request(drive, 'next_trial')
    assert drive.status()['mode'] == 'fault' and not drive.status()['can_start_next_round']
    assert len(workers) == 1
    drive.close()


def test_multiple_explicit_rounds_preserve_each_deadline_and_empty_start():
    drive, clock, workers, _ = setup()
    request(drive, 'enable'); drive.thread.join(timeout=2)
    for number in range(1, 5):
        assert drive.status()['trial_number'] == number
        assert workers[-1].drive.deadline == clock.now+72
        assert workers[-1].drive.motor == 'stop'
        expire(drive, clock, workers); workers[-1].normal = True
        request(drive, 'next_trial'); drive.thread.join(timeout=2)
    assert len(workers) == 5
    drive.close()


def owned_class():
    path = Path(__file__).parents[1]/'tools/serve_bench_drive.py'
    spec = importlib.util.spec_from_file_location('repeatable_bench_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.OwnedWorkerDrive


@pytest.mark.parametrize('bad', [None, 'exit', 'missing', 'boot', 'mode', 'cleanup', 'guardian'])
def test_owned_worker_requires_normal_exit_and_both_cleanup_results(tmp_path, monkeypatch, bad):
    class Process:
        def poll(self):
            return 1 if bad == 'exit' else 0

    monkeypatch.setattr('carvision.manual_drive.WorkerDrive.status',
                        lambda self: {'mode': 'expired', 'session_remaining_s': 0})
    completion = {'boot_id': 'boot', 'mode_before_close': 'expired', 'cleanup_completed': True}
    result = {'boot_id': 'boot', 'cleanup_errors': []}
    guardian = dict(result)
    if bad == 'boot': completion['boot_id'] = 'old-boot'
    if bad == 'mode': completion['mode_before_close'] = 'fault'
    if bad == 'cleanup': result['cleanup_errors'] = ['failed']
    if bad == 'guardian': guardian['cleanup_errors'] = ['failed']
    if bad != 'missing':
        (tmp_path/'trial-completion.json').write_text(json.dumps(completion))
    (tmp_path/'result.json').write_text(json.dumps(result))
    (tmp_path/'guardian-result.json').write_text(json.dumps(guardian))
    drive = owned_class()(tmp_path/'socket', process=Process(), worker_log=None,
                          output=tmp_path, expected_boot='boot')
    assert drive.status()['trial_completed_normally'] is (bad is None)
    assert drive.status()['mode'] == ('expired' if bad is None else 'fault')
