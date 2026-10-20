"""Image-feedback behavior across frame rates and camera/target changes."""
import pytest

from carvision.steering import ImagePD


def test_approaching_error_gets_anticipation_and_recovers_without_integral_windup():
    pd=ImagePD(kp=1.,kd=.12)
    pd.update(.1,0.)
    assert pd.update(.2,.1)>.2
    assert pd.update(.03,.2)<0.  # Countersteer while a large error is falling.
    assert pd.update(0.,.3)==0.
    for i in range(4,30):result=pd.update(.03,i*.1)
    assert result==pytest.approx(.03,abs=1e-5)


def test_time_based_derivative_is_consistent_at_five_and_twenty_hz():
    values=[]
    for dt in (.2,.05):
        pd=ImagePD();pd.update(.1,0.)
        for i in range(1,round(.4/dt)+1):
            t=round(i*dt,6);value=pd.update(.1+.2*t,t)
        values.append(value)
    assert values[0]==pytest.approx(values[1],abs=1e-6)


def test_cached_or_changed_camera_frame_does_not_create_derivative_kick():
    pd=ImagePD();first=pd.update(.1,1.,context='left-camera')
    assert pd.update(.9,1.,context='left-camera')==first
    # Both cameras can be fresh even if the new camera's capture is older.
    assert pd.update(.3,.99,context='right-camera')==pytest.approx(.3)
    with pytest.raises(ValueError,match='backwards'):pd.update(.2,.98,context='right-camera')


@pytest.mark.parametrize('error',[float('nan'),float('inf'),True])
def test_nonfinite_or_boolean_observation_is_rejected(error):
    with pytest.raises(ValueError):ImagePD().update(error,0.)
