"""Carrier alone is insufficient to identify a disabled Wi-Fi interface."""
from pathlib import Path

import pytest

from carvision.boot_test import PiBootPreparation
from carvision.network_guard import interface_is_inactive


@pytest.mark.parametrize('flags,carrier,expected', [
    ('0x1002', '1', True), ('0x1002', '0', True),
    ('0x1003', '0', True), ('0x1003', '1', False)])
def test_other_uplink_requires_disabled_interface_or_no_carrier(monkeypatch, flags, carrier, expected):
    seen = []
    def read(path, *args, **kwargs):
        seen.append(path.name)
        return flags if path.name == 'flags' else carrier
    monkeypatch.setattr(Path, 'read_text', read)
    assert interface_is_inactive('wlan0') is expected
    if not int(flags, 16) & 1:
        assert seen == ['flags']


@pytest.mark.parametrize('failure', ['unknown_flags', 'carrier_unreadable'])
def test_unverifiable_interface_is_never_silently_treated_as_disconnected(monkeypatch, failure):
    def read(path, *args, **kwargs):
        if path.name == 'flags':
            return 'unknown' if failure == 'unknown_flags' else '0x1003'
        raise OSError('carrier cannot be read')
    monkeypatch.setattr(Path, 'read_text', read)
    with pytest.raises((ValueError, OSError)):
        interface_is_inactive('wlan0')


@pytest.mark.parametrize('wifi_enabled', [False, True])
def test_boot_health_accepts_disabled_wifi_but_rejects_an_active_competing_uplink(tmp_path, monkeypatch, wifi_enabled):
    monkeypatch.setattr(PiBootPreparation, 'current_boot', staticmethod(lambda: 'network-test'))
    def read(path, *args, **kwargs):
        if path.name == 'flags':
            return '0x1003' if '/eth0/' in path.as_posix() or wifi_enabled else '0x1002'
        if path.name == 'carrier':
            return '0' if '/eth0/' in path.as_posix() else '1'
        raise AssertionError('unexpected read: '+str(path))
    monkeypatch.setattr(Path, 'read_text', read)
    def output(command, **kwargs):
        if command[-1] == 'get_throttled': return 'throttled=0x0'
        if command[-1] == 'measure_temp': return "temp=45.0'C"
        return '[{"dev":"usb0"}]'
    monkeypatch.setattr('carvision.boot_test.subprocess.check_output', output)
    prep = PiBootPreparation(tmp_path, tmp_path/'template.json')
    if wifi_enabled:
        with pytest.raises(RuntimeError, match='纯 5G'):
            prep.health()
    else:
        assert prep.health()['route']['dev'] == 'usb0'
