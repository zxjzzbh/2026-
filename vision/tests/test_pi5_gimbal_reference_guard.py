"""A failed stored-pose check must close the UART without moving the camera."""
import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest


def test_reference_tool_does_not_move_an_aligned_camera_when_precheck_fails(tmp_path, monkeypatch):
    path = Path(__file__).parents[1]/'tools/bench_pi5_gimbal.py'
    spec = importlib.util.spec_from_file_location('bench_pi5_gimbal_guard_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    review = tmp_path/'review.json'
    review.write_text(json.dumps({'boot_id': 'test-boot', 'user_esc_off_confirmed': True,
        'user_gimbal_free_confirmed': True, 'user_cable_slack_confirmed': True,
        'previous_observed_reference_range_us': [1500, 1550]}))
    read_text = Path.read_text
    monkeypatch.setattr(Path, 'read_text', lambda self, *args, **kwargs:
        'test-boot' if self.name == 'boot_id' else read_text(self, *args, **kwargs))
    monkeypatch.setattr(module.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(module, 'urlopen', lambda *args, **kwargs:
        io.BytesIO(json.dumps({'controls_paused': True, 'hardware_output': False}).encode()))
    sent = []
    class Board:
        fd = None
        def open(self): self.fd = 42
        def close(self): self.fd = None
        def read_position(self, channel): return 1650 if channel == 1 else 0
        def set_gimbal_reference(self, *args): sent.append(args)
    monkeypatch.setattr(module, 'RasAdapter', Board)
    output = tmp_path/'output'
    monkeypatch.setattr(sys, 'argv', ['bench', '--channel', '1', '--review', str(review), '--output', str(output)])
    with pytest.raises(AssertionError):
        module.main()
    assert sent == []
    result = json.loads((output/'result.json').read_text())
    assert result['gimbal_hardware_output'] is False
    assert result['final_reference_command_sent'] is False
