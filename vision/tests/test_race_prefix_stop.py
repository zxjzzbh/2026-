"""Prefix boundary stops through the real shared guard, with no hardware.

These checks prove command rejection and a finite release pulse. They do not
verify a physical stopping point or substitute for parking measurements.
"""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.race_flow import load_bundle, parking_blockers
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from test_race_flow import measured_bundle
from test_shared_executor import CLIENT, Worker, rig

ROOT = Path(__file__).resolve().parents[2]


def run_boundary(monkeypatch, rig, bundle, target):
    now,clock,backend,manual,owner,_ = rig
    worker = Worker(owner)
    delegated = []
    view = SimpleNamespace(
        status=manual.status,
        request=lambda action,payload:delegated.append((action,payload)),
        borrow_actuator=lambda settings,client:BorrowedActuator(worker,settings,client,clock=clock))

    class Reader:
        preview = b'offline-preview'

        def __init__(self,*args):
            self.sequence = self.main_reads = 0

        def read(self,stage,t):
            self.sequence += 1
            if stage!='arming':
                self.main_reads += 1
                if self.main_reads>2:
                    raise EOFError('offline scenario complete')
            sample = {'sequence':self.sequence,'captured_s':t,
                      'motion':{'valid':True,'moving':False,'captured_s':t}}
            now[0] = round(t+.05,8)
            return sample

    class Controller:
        def __init__(self,*args,**kwargs):
            self.stage,self.reason = 'cones','offline prefix'
            self.calls = 0
            self.produced = []

        def step(self,sample,t,audio=None):
            self.calls += 1
            self.stage = 'cones' if self.calls==1 else target
            self.reason = 'previous stage forward' if self.calls==1 else 'would enter parking'
            intent = {'phase':self.stage,'action':'search','esc_us':1600,'steering_us':1610,
                      'reason':self.reason,'events':[],
                      'offline_marker':'before_boundary' if self.calls==1 else 'parking_boundary'}
            self.produced.append(intent)
            return intent

        def status(self,t):
            return {'stage':self.stage,'reason':self.reason}

        def frame_age_limit(self):
            return .25

        def fail(self,reason):
            self.stage,self.reason = 'fault',reason

    monkeypatch.setattr('carvision.race_flow_console.RaceFlowController',Controller)
    drive = RaceFlowDrive(view,{},ROOT,live=True,clock=clock,reader_factory=Reader)
    drive.bundle = copy.deepcopy(bundle)
    drive.owner,drive.token = CLIENT,'offline-run-token'
    drive.active,drive.lease_until = True,10.
    drive._work(drive.cancel_event,drive.token)
    return drive,worker,delegated


@pytest.mark.parametrize('target',['parking_search','parking_entry','payment'])
def test_missing_parking_rejects_boundary_forward_before_shared_actuator_send(monkeypatch,rig,target):
    bundle = load_bundle(ROOT)
    original = copy.deepcopy(bundle)
    missing = parking_blockers(bundle)
    assert len(missing)==13
    drive,worker,delegated = run_boundary(monkeypatch,rig,bundle,target)
    now,clock,backend,manual,owner,_ = rig
    assert len(drive.controller.produced)==2
    assert drive.controller.produced[1]['phase']==target
    assert drive.controller.produced[1]['esc_us']==1600
    commands = [payload['intent'] for action,payload in worker.calls if action=='autonomy_intent']
    forwards = [command for command in commands if command['esc_us']>1500]
    assert len(forwards)==1 and forwards[0]['esc_us']==1600
    assert drive.last_intent['offline_marker']=='before_boundary'
    acquired = next(payload['settings'] for action,payload in worker.calls if action=='autonomy_acquire')
    assert 'parking_forward_us' not in acquired
    assert drive.stage=='fault' and not drive.active and drive.actuator is None
    assert not drive.payment_scanned and not any(action.startswith('payment') for action,_ in delegated)
    assert all(reason in drive.reason for reason in missing)
    assert bundle==original and parking_blockers(drive.bundle)==missing

    # close() owns the stop: it aborts the preceding real 1600-us command,
    # rather than transmitting the rejected parking-search command.
    assert owner.guard is not None and owner.guard.esc==1400
    released_at = now[0]
    brake_end = owner.guard.emergency_until
    assert brake_end==pytest.approx(released_at+.35)
    while now[0]<brake_end:
        now[0] = min(brake_end,round(now[0]+.05,8))
        owner.tick()
    assert owner.guard is None and manual.mode=='emergency' and not backend.closed
    segments = [row for row in backend.writes if row[0]=='motion' and row[2]==1400]
    assert segments and all(t+duration<=brake_end+1e-9 for _,t,pulse,duration in segments)
    assert any(row[0]=='neutral' and row[1]>=brake_end for row in backend.writes)


def test_synthetic_fully_measured_parking_does_not_trigger_prefix_cutoff(monkeypatch,rig):
    original = load_bundle(ROOT)
    original_copy = copy.deepcopy(original)
    # Existing explicit unit-test calibration fixture, not a production flag
    # update or an assertion that the real car is ready for metric parking.
    ready = measured_bundle(original)
    assert not parking_blockers(ready)
    drive,worker,delegated = run_boundary(monkeypatch,rig,ready,'parking_search')
    commands = [payload['intent'] for action,payload in worker.calls if action=='autonomy_intent']
    assert len([command for command in commands if command['esc_us']==1600])==2
    assert drive.last_intent['offline_marker']=='parking_boundary'
    assert drive.reason=='offline scenario complete'
    acquired = next(payload['settings'] for action,payload in worker.calls if action=='autonomy_acquire')
    assert acquired['parking_forward_us']
    assert not drive.payment_scanned and not any(action.startswith('payment') for action,_ in delegated)
    assert original==original_copy and len(parking_blockers(original))==13
