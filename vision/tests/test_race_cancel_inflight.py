"""Operator exit during camera processing must not become an executor fault."""
import copy
import json
import threading

import pytest

from carvision.cone_command import ConeCommandSender
from carvision.race_flow_console import RaceFlowDrive
from test_race_flow import Base, bundle


class Actuator:
    run = False

    def __init__(self, close_error=False):
        self.closed = threading.Event()
        self.outputs = []
        self.fail_on_close = close_error

    def send(self, intent):
        assert not self.closed.is_set(), 'command sent after release'
        self.outputs.append(dict(intent))

    def close(self):
        self.closed.set()
        if self.fail_on_close:
            raise RuntimeError('release confirmation failed')


class Sender(ConeCommandSender):
    def __init__(self, actuator):
        super().__init__(actuator, clock=lambda: 1., threaded=False)
        self.queries = 0
        self.on_query = None

    def race_interrupted(self):
        self.queries += 1
        if self.on_query is not None:
            self.on_query()
        return super().race_interrupted()


class Reader:
    preview = None

    def __init__(self, result):
        self.result = result
        self.entered = threading.Event()
        self.resume = threading.Event()
        self.closed = False

    def read(self, *args):
        self.entered.set()
        if not self.resume.wait(3):
            raise RuntimeError('test reader was not released')
        if isinstance(self.result, Exception):
            raise self.result
        return self.result

    def close(self):
        self.closed = True


def make_flow(bundle, tmp_path, monkeypatch, reader, actuator=None):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle', lambda *a: copy.deepcopy(bundle))
    actuator = actuator or Actuator()
    sender = Sender(actuator)
    flow = RaceFlowDrive(Base(), {}, tmp_path, clock=lambda: 1.,
                         actuator_factory=lambda _: sender, reader_factory=lambda *a: reader,
                         offline_competition=True)
    flow.active = True
    flow.token = 'test-run'
    flow.thread = threading.Thread(target=flow._work)
    return flow, sender, actuator


@pytest.mark.parametrize('result', [None, {'sequence': 1, 'captured_s': 1.},
                                   RuntimeError('camera read interrupted')])
def test_exit_during_read_preserves_cancelled_and_never_queries_closed_sender(
        bundle, tmp_path, monkeypatch, result):
    reader = Reader(result)
    flow, sender, actuator = make_flow(bundle, tmp_path, monkeypatch, reader)
    flow.thread.start()
    assert reader.entered.wait(2)
    stopping = threading.Thread(target=flow.cancel, args=('operator exit',))
    stopping.start()
    try:
        assert actuator.closed.wait(2)
    finally:
        reader.resume.set()
        stopping.join(3)
        flow.thread.join(3)
    assert not stopping.is_alive() and not flow.thread.is_alive()
    assert flow.stage == flow.controller.stage == 'cancelled'
    assert flow.reason == 'operator exit' and flow.cleanup_error is None
    assert sender.queries == 0 and not flow.active and reader.closed
    assert len(actuator.outputs) == 1 and actuator.outputs[0]['esc_us'] == 1500


def test_exit_cannot_close_sender_while_interruption_query_is_in_progress(bundle, tmp_path, monkeypatch):
    reader = Reader(None)
    reader.resume.set()
    flow, sender, actuator = make_flow(bundle, tmp_path, monkeypatch, reader)
    querying, resume_query, cancelling = threading.Event(), threading.Event(), threading.Event()

    def query():
        querying.set()
        assert resume_query.wait(3)

    def cancel():
        cancelling.set()
        flow.cancel('operator exit')

    sender.on_query = query
    flow.thread.start()
    assert querying.wait(2)
    stopping = threading.Thread(target=cancel)
    stopping.start()
    try:
        assert cancelling.wait(2)
        assert not actuator.closed.wait(.1), 'cancel closed a sender still being queried'
    finally:
        resume_query.set()
        stopping.join(3)
        flow.thread.join(3)
    assert not stopping.is_alive() and not flow.thread.is_alive()
    assert actuator.closed.is_set() and flow.stage == 'cancelled'


def test_active_reader_failure_still_faults_and_releases_actuator(bundle, tmp_path, monkeypatch):
    reader = Reader(RuntimeError('camera decoder failed'))
    reader.resume.set()
    flow, sender, actuator = make_flow(bundle, tmp_path, monkeypatch, reader)
    flow.thread.start()
    flow.thread.join(3)
    assert not flow.thread.is_alive()
    assert flow.stage == flow.controller.stage == 'fault'
    assert flow.reason == 'camera decoder failed'
    assert actuator.closed.is_set() and not flow.active and reader.closed


def test_cancel_does_not_hide_unconfirmed_actuator_release(bundle, tmp_path, monkeypatch):
    reader = Reader(None)
    flow, sender, actuator = make_flow(bundle, tmp_path, monkeypatch, reader, Actuator(close_error=True))
    flow.thread.start()
    assert reader.entered.wait(2)
    stopping = threading.Thread(target=flow.cancel, args=('operator exit',))
    stopping.start()
    try:
        assert actuator.closed.wait(2)
    finally:
        reader.resume.set()
        stopping.join(3)
        flow.thread.join(3)
    assert not stopping.is_alive() and not flow.thread.is_alive()
    assert flow.stage == 'fault' and flow.cleanup_error == 'release confirmation failed'
    assert 'release confirmation failed' in flow.reason


def test_vision_breakdown_survives_reader_reuse_and_is_recorded(bundle, tmp_path, monkeypatch):
    timings = {'camera_and_road_s': .22, 'navigation_s': .04, 'lane_preview_s': .02, 'task_s': .11}
    expected = dict(timings)

    class TimingReader(Reader):
        reads = 0

        def read(self, *args):
            self.reads += 1
            if self.reads == 1:
                return {'sequence': 1, 'captured_s': 1., 'vision_processing_s': .4,
                        'vision_stages_s': timings}
            # The reader may reuse its scratch mapping for the next frame.
            timings['task_s'] = 999.
            flow.cancel('test complete')
            return None

    reader = TimingReader(None)
    flow, sender, actuator = make_flow(bundle, tmp_path, monkeypatch, reader)
    flow.record = tmp_path / 'record'
    flow.record.mkdir()
    flow.thread.start()
    flow.thread.join(3)
    assert not flow.thread.is_alive() and flow.stage == 'cancelled'
    state = flow.status()['race_flow']
    assert state['vision_processing_s'] == .4 and state['vision_stages_s'] == expected
    state['vision_stages_s']['task_s'] = -1.
    assert flow.vision_stages_s == expected
    rows = [json.loads(line) for line in (flow.record / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
    decision = next(row for row in rows if row['event'] == 'decision')
    assert decision['vision_processing_s'] == .4 and decision['vision_stages_s'] == expected
    summary = json.loads((flow.record / 'summary.json').read_text(encoding='utf-8'))
    assert summary['vision_stages_s'] == expected
