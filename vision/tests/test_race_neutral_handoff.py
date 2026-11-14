"""Replay the stopped handoff lease gap with real ownership and fake outputs."""
import pytest

from carvision.cone_command import ConeCommandSender
from carvision.race_flow_console import RaceFlowDrive
from test_shared_executor import rig, borrow, intent, CLIENT


def test_camera_gap_at_wait_board_keeps_neutral_owner_without_moving(rig):
    now, clock, backend, controller, owner, settings = rig
    actuator, _ = borrow(rig)
    sender = ConeCommandSender(actuator, clock=clock, threaded=False)
    flow = RaceFlowDrive.__new__(RaceFlowDrive)
    flow.offline_competition = False
    flow.lease_until = 3.
    flow._send_intent(sender, {'action':'neutral','esc_us':1500,'steering_us':1610})
    # Original field failure: the first camera read returned after 657 ms.
    for tick in range(1, 41):
        now[0] = tick * .025
        sender.tick()
        owner.tick()
    assert owner.active and owner.guard.esc == 1500
    assert sender.count >= 20
    assert not any(row[0] == 'motion' for row in backend.writes)
    sender.close()
    assert not sender.close_error and not owner.active


def test_neutral_supervision_expiry_cannot_keep_owner_forever(rig):
    now, clock, _, _, owner, _ = rig
    actuator, _ = borrow(rig)
    sender = ConeCommandSender(actuator, clock=clock, threaded=False)
    sender.send({'action':'neutral','esc_us':1500,'steering_us':1610,'page_deadline_s':.6})
    for tick in range(1, 25):
        now[0] = tick * .025
        sender.tick()
        owner.tick()
    assert sender.fault and owner.guard.esc == 1500
    now[0] = 1.11
    owner.tick()
    assert not owner.active
    sender.close()
    assert sender.close_error is None


def test_movement_after_neutral_does_not_inherit_page_cadence(rig):
    now, clock, _, _, owner, _ = rig
    actuator, _ = borrow(rig)
    sender = ConeCommandSender(actuator, clock=clock, threaded=False)
    sender.send({'action':'neutral','esc_us':1500,'steering_us':1610,'page_deadline_s':3.})
    sender.send(intent())
    now[0] = .21
    sender.tick()
    owner.tick()
    assert not owner.active and owner.guard.esc == 1400
    sender.close()


def test_expired_neutral_owner_cleanup_is_idempotent_but_commands_stay_rejected(rig):
    now, _, _, _, owner, _ = rig
    actuator, worker = borrow(rig)
    actuator.send({'action':'neutral','esc_us':1500,'steering_us':1610})
    now[0] = .21
    owner.tick()
    assert owner.guard is None and not owner.active
    with pytest.raises(ValueError, match='失效'):
        actuator.send(intent())
    actuator.close()
    assert actuator.close_error is None
    worker.request('autonomy_release', {'client':CLIENT,'worker_token':actuator.token})
    with pytest.raises(ValueError, match='失效'):
        worker.request('autonomy_release', {'client':'foreign','worker_token':actuator.token})
