import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from carvision.gimbal import GimbalConfig,ServoCalibration,gimbal_test


def calibrated():
    return GimbalConfig(supply_voltage_v=5,signal_3v3_confirmed=True,calibration_confirmed=True,
                        daemon_compatibility_confirmed=True,
                        pan=ServoCalibration('test-only',1400,1500,1600),
                        tilt=ServoCalibration('test-only',1450,1500,1550))


class FakeDriver:
    def __init__(self):
        self.calls=[]
    def open(self):
        self.calls.append('open')
    def mode(self,gpio):
        return 0
    def set_servo(self,gpio,pulse):
        self.calls.append((gpio,pulse))
    def release(self,gpio):
        self.calls.append(('release',gpio))
    def close(self):
        self.calls.append('close')


def arguments(tmp_path,config,run=False,**values):
    p=tmp_path/'gimbal.json'
    p.write_text(json.dumps(asdict(config)))
    return SimpleNamespace(gimbal_config=p,run=run,acknowledge_motion=True,
                           pan=values.get('pan',0),tilt=values.get('tilt',0),seconds=.11)


def test_unconfigured_gimbal_dry_run_never_opens_driver(tmp_path):
    fake=FakeDriver()
    result=gimbal_test(arguments(tmp_path,GimbalConfig()),fake)
    assert fake.calls==[] and result['hardware_output'] is False
    assert result['pan_us'] is None and result['tilt_us'] is None


def test_unconfirmed_gimbal_cannot_emit_pulses(tmp_path):
    fake=FakeDriver()
    with pytest.raises(ValueError,match='calibration incomplete'):
        gimbal_test(arguments(tmp_path,GimbalConfig(),True),fake)
    assert fake.calls==[]


def test_calibrated_gimbal_slews_gradually_then_disables_and_releases_both_pins(tmp_path):
    fake=FakeDriver()
    gimbal_test(arguments(tmp_path,calibrated(),True,pan=1,tilt=-1),fake)
    for gpio in (17,27):
        pulses=[c[1] for c in fake.calls if isinstance(c,tuple) and c[0]==gpio and c[1]!=0]
        assert pulses[0]==1500
        assert all(abs(a-b)<=10 for a,b in zip(pulses,pulses[1:]))
        assert (gpio,0) in fake.calls and ('release',gpio) in fake.calls
    assert fake.calls[-1]=='close'


def test_output_failure_still_releases_both_gimbal_axes(tmp_path):
    class Failing(FakeDriver):
        def set_servo(self,gpio,pulse):
            super().set_servo(gpio,pulse)
            if gpio==27 and pulse!=0:
                raise RuntimeError('simulated driver failure')
    fake=Failing()
    with pytest.raises(RuntimeError,match='simulated driver failure'):
        gimbal_test(arguments(tmp_path,calibrated(),True),fake)
    assert (17,0) in fake.calls and (27,0) in fake.calls
    assert fake.calls[-1]=='close'


@pytest.mark.parametrize('value',[float('nan'),float('inf'),2,-2,True])
def test_invalid_pan_position_is_rejected_before_driver_access(tmp_path,value):
    fake=FakeDriver()
    with pytest.raises(ValueError):
        gimbal_test(arguments(tmp_path,calibrated(),True,pan=value),fake)
    assert not fake.calls


def test_gimbal_pins_cannot_alias_vehicle_pwm():
    with pytest.raises(ValueError):
        GimbalConfig(pan_gpio=12).validate()
