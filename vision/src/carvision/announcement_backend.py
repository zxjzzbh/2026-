"""Simulation-friendly announcement lifecycle.

This backend does not claim that the physical I2C 0x40 module played audio.
The real backend must be supplied by the deployment owner after hardware
protocol and speaker completion have been verified.
"""

_DEFAULT_RESULT = object()


class AnnouncementBackend:
    text = "我是山东大学启航队的智能车，请为我加油！"

    def __init__(self, *, simulated=True, result=_DEFAULT_RESULT):
        if type(simulated) is not bool:
            raise ValueError("simulated must be a boolean")
        if result is _DEFAULT_RESULT:
            result = True if simulated else None
        if result is not None and type(result) is not bool:
            raise ValueError("result must be boolean or None")
        if not simulated and result is True:
            raise ValueError("real audio completion requires a verified hardware backend, not an injected success flag")
        self.simulated = simulated
        self.result = result
        self.started = False
        self.done = False
        self.failed = False

    def start(self, text):
        if text != self.text:
            raise ValueError("announcement text does not match the competition sentence")
        if self.started:
            return False
        self.started = True
        if self.result is False:
            self.failed = True
        elif self.simulated and self.result is True:
            self.done = True
        return True

    def complete(self, success):
        """Explicit simulator completion; never acknowledges a real speaker."""
        if not self.simulated:
            raise RuntimeError("real completion is unavailable in the simulator backend")
        if type(success) is not bool:
            raise ValueError("success must be a boolean")
        if not self.started:
            raise RuntimeError("announcement has not started")
        if self.done or self.failed:
            return False
        self.done = success
        self.failed = not success
        return True

    def status(self):
        return {
            "started": self.started,
            "done": self.done,
            "failed": self.failed,
            "simulated": self.simulated,
            # No method here talks to a speaker or verifies its completion.
            "hardware_verified": False,
        }
