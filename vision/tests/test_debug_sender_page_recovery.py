"""Page ownership and decision freshness are independent on the shared sender."""
import pytest

from test_race_sender_cadence import rig, advance, FORWARD, NEUTRAL


def test_page_heartbeat_never_renews_old_forward_decision(rig):
    clock,actuator,sender=rig
    sender.send_race(FORWARD,run_until=3.,evidence_until=.3,page_deadline_s=3.)
    original=sender.race_deadline
    advance(rig,.6)
    sender.refresh_page_deadline(4.)
    assert sender.race_deadline==original and sender.race_until==4.
    advance(rig,.76)
    assert sender.race_interrupted() and actuator.guard.esc==1400
    assert sender.fault is None
    advance(rig,1.12)
    sender.send_race(NEUTRAL,run_until=4.,page_deadline_s=4.,recovered=True)
    assert not sender.race_interrupted()


def test_page_expiry_finishes_finite_brake_and_cannot_be_revived(rig):
    clock,actuator,sender=rig
    sender.send_race(FORWARD,run_until=.3,evidence_until=.3,page_deadline_s=.3)
    advance(rig,.3)
    assert actuator.guard.esc==1400 and sender.fault is None
    sender.refresh_page_deadline(10.)
    assert sender.race_page_deadline==.3
    advance(rig,.64)
    assert actuator.guard.esc==1400
    advance(rig,.66)
    assert actuator.guard.esc==1500 and '心跳' in sender.fault
    assert sum(c['esc_us']==1400 for c in actuator.commands)==1
    with pytest.raises(RuntimeError):
        sender.send_race(FORWARD,run_until=10.,evidence_until=1.,page_deadline_s=10.)


def test_expired_prepared_neutral_does_not_brake(rig):
    _,actuator,sender=rig
    sender.send_race(NEUTRAL,run_until=.3,page_deadline_s=.3)
    advance(rig,.31)
    assert sender.fault and set(c['esc_us'] for c in actuator.commands)=={1500}
