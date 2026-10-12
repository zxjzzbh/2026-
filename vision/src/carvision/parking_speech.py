"""One nonblocking short TTS request per parking run, with honest status.

Uses the car's existing I2C 0x40 probe helper in a bounded child process.
Successful transmission does not prove sound was heard or synthesis completed.
"""
import json
import subprocess
import sys
import threading
from pathlib import Path


PARKING_PHRASE = "我停车了啊"
TEAM_PARKING_PHRASE = "我是山东大学启航队的智能车，请为我加油"


class ParkingSpeech:
    def __init__(self, root, *, text=PARKING_PHRASE, popen=subprocess.Popen):
        if not isinstance(text, str) or not text.strip():
            raise ValueError('parking announcement must be nonempty text')
        self.root, self.popen = Path(root), popen
        self.text = text
        self.lock = threading.RLock()
        self.thread = self.process = None
        self.started = self.cancelled = False
        self.state, self.error = "not_requested", None

    def start(self):
        with self.lock:
            if self.started or self.cancelled:
                return False
            helper = self.root/'vision/tools/speak_parking.py'
            self.started = True
            if not helper.is_file():
                self.state, self.error = "failed", "TTS helper missing"
                return False
            self.state = "sending"
            self.thread = threading.Thread(target=self._send, args=(helper,), name="parking-speech", daemon=True)
            self.thread.start()
            return True

    def _send(self, helper):
        process = None
        try:
            with self.lock:
                if self.cancelled:
                    return
                process = self.popen([sys.executable, str(helper), '--bus', '1', '--text', self.text,
                                      '--volume', '9'], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
                self.process = process
            stdout, stderr = process.communicate(timeout=3)
            data = json.loads(stdout.decode('utf-8'))
            if process.returncode != 0 or data.get('playback_command_sent') is not True:
                raise RuntimeError(data.get('error') or stderr.decode('utf-8',errors='replace') or 'TTS transmission not confirmed')
            with self.lock:
                if not self.cancelled:
                    self.state = "command_sent"
        except Exception as exc:
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate(timeout=1)
            with self.lock:
                if not self.cancelled:
                    self.state, self.error = "failed", str(exc)

    def status(self):
        with self.lock:
            return {'text':self.text, 'state':self.state, 'error':self.error,
                    'command_sent':self.state=='command_sent', 'playback_heard':None,
                    'playback_completed_verified':False}

    def close(self):
        with self.lock:
            self.cancelled = True
            if self.state == 'sending':
                self.state = 'cancelled'
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=1)
