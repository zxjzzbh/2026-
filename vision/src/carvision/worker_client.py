"""Ownership-aware client for one prepared manual hardware worker.

A disappeared socket does not establish successful hardware cleanup. Normal
round completion requires the worker and guardian records on the same boot.
"""
import json
import subprocess

from .manual_drive import WorkerDrive


class OwnedWorkerDrive(WorkerDrive):
    """A finished socket alone is insufficient proof to create another owner."""
    def __init__(self, path, *, process, worker_log, output, expected_boot, **options):
        super().__init__(path, **options)
        self.process, self.worker_log = process, worker_log
        self.output, self.expected_boot = output, expected_boot
        self.closed = False

    def status(self):
        state = super().status()
        normal = False
        if self.process.poll() == 0:
            try:
                completed = json.loads((self.output/'trial-completion.json').read_text())
                result = json.loads((self.output/'result.json').read_text())
                guardian = json.loads((self.output/'guardian-result.json').read_text())
                normal = (completed['boot_id'] == result['boot_id'] == guardian['boot_id'] == self.expected_boot
                          and completed['mode_before_close'] == 'expired'
                          and completed['cleanup_completed'] is True
                          and result['cleanup_errors'] == guardian['cleanup_errors'] == [])
            except (OSError, ValueError, KeyError):
                pass
        if normal:
            state.update(mode='expired', reason='本轮已结束，确认停稳后可开始下一轮',
                         motor_direction='stop', motor_pulse_us=1500, session_remaining_s=0)
        elif self.process.poll() is not None:
            state.update(mode='fault', reason='本轮未完成正常清理，请关闭电调后检查')
        return {**state, 'trial_completed_normally': normal}

    def close(self):
        if self.closed:
            return
        try:
            super().close()
        finally:
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
            finally:
                self.worker_log.close()
                self.closed = True

