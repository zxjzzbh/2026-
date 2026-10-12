import json
import threading
import time

from carvision.parking_speech import ParkingSpeech, PARKING_PHRASE, TEAM_PARKING_PHRASE


def helper(tmp_path):
    path=tmp_path/'vision/tools/speak_parking.py'
    path.parent.mkdir(parents=True)
    path.write_text('# mock transport target for lifecycle test')
    return tmp_path


def test_parking_speech_exact_phrase_once_and_never_claims_heard(tmp_path):
    calls=[]
    class Process:
        returncode=0
        def communicate(self,timeout):return json.dumps({'playback_command_sent':True}).encode(),b''
        def poll(self):return 0
    def spawn(command,**kwargs):calls.append(command);return Process()
    speech=ParkingSpeech(helper(tmp_path),popen=spawn)
    assert speech.start()
    speech.thread.join(1)
    assert not speech.start()
    assert len(calls)==1 and calls[0][calls[0].index('--text')+1]==PARKING_PHRASE=='我停车了啊'
    state=speech.status()
    assert state['command_sent'] and state['playback_heard'] is None
    assert state['playback_completed_verified'] is False


def test_blocked_speech_does_not_block_control_thread(tmp_path):
    release=threading.Event()
    class Process:
        returncode=0
        def communicate(self,timeout):
            assert release.wait(timeout)
            return b'{"playback_command_sent":true}',b''
        def poll(self):return 0 if release.is_set() else None
        def terminate(self):release.set()
    speech=ParkingSpeech(helper(tmp_path),popen=lambda *a,**k:Process())
    started=time.monotonic();speech.start()
    assert time.monotonic()-started<.2
    assert speech.status()['state']=='sending'
    release.set();speech.thread.join(1)
    assert speech.status()['state']=='command_sent'


def test_missing_speaker_and_transport_error_are_reported(tmp_path):
    missing=ParkingSpeech(tmp_path)
    assert not missing.start() and missing.status()['state']=='failed'
    def fail(*a,**k):raise OSError('I2C helper cannot start')
    speech=ParkingSpeech(helper(tmp_path),popen=fail)
    assert speech.start();speech.thread.join(1)
    assert speech.status()['state']=='failed'
    assert not speech.status()['command_sent']


def test_close_before_start_cannot_play_stale_audio(tmp_path):
    speech=ParkingSpeech(helper(tmp_path),popen=lambda *a,**k:(_ for _ in ()).throw(AssertionError('started after close')))
    speech.close()
    assert not speech.start()


def test_team_announcement_is_sent_whole_once(tmp_path):
    calls=[]
    class Process:
        returncode=0
        def communicate(self,timeout):return b'{"playback_command_sent":true}',b''
        def poll(self):return 0
    def spawn(command,**kwargs):calls.append(command);return Process()
    speech=ParkingSpeech(helper(tmp_path),text=TEAM_PARKING_PHRASE,popen=spawn)
    speech.start();speech.thread.join(1);speech.start()
    assert len(calls)==1
    assert calls[0][calls[0].index('--text')+1]=='我是山东大学启航队的智能车，请为我加油'
    assert speech.status()['text']==TEAM_PARKING_PHRASE


def test_actual_helper_sends_long_phrase_as_one_raw_i2c_packet(monkeypatch):
    import importlib.util
    from pathlib import Path
    path=Path(__file__).parents[1]/'tools/speak_parking.py'
    spec=importlib.util.spec_from_file_location('tts_helper_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    packets=[]
    def write(fd,payload):packets.append(payload);return len(payload)
    monkeypatch.setattr(module.os,'write',write)
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    device=object.__new__(module.TTSModule);device.fd=123
    device.speak(TEAM_PARKING_PHRASE,9)
    assert len(packets)==1 and 32<len(packets[0])<=64
    assert packets[0][6:].decode('gb2312')=='[h0][v9]'+TEAM_PARKING_PHRASE
