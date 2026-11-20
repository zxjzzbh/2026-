"""Foreground keyboard controls; mission time can pause, hardware time cannot."""
import os
import select
import sys
import time


class TerminalKeys:
    """Read one S/D action per key burst, without swallowing Ctrl+C."""

    REPRESS_GAP = 1.2  # Includes the delayed first repeat (up to 1s) over SSH.

    def __init__(self, fd=None, clock=time.monotonic):
        self.fd, self.clock = fd, clock
        self.enabled = False
        self.saved = None
        self.escape = ''
        self.last_seen = {'s': float('-inf'), 'd': float('-inf')}

    def __enter__(self):
        if self.fd is None:
            try: self.fd = sys.stdin.fileno()
            except (AttributeError, OSError, ValueError): return self
        if os.name != 'posix' or not os.isatty(self.fd): return self
        import termios
        import tty
        self.saved = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd, termios.TCSANOW)
            self.enabled = True
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            self.saved = None
            raise
        return self

    def poll(self):
        if not self.enabled or not select.select([self.fd], [], [], .001)[0]: return None
        data = os.read(self.fd, 4096)
        if not data:
            self.enabled = False
            return None
        keys = []
        for char in data.decode('ascii', 'ignore'):
            if self.escape == 'esc':
                self.escape = 'sequence' if char in '[O' else ''
                continue
            if self.escape == 'sequence':
                if '@' <= char <= '~': self.escape = ''
                continue
            if char == '\x1b':
                self.escape = 'esc'
            elif char.lower() in ('s', 'd'):
                keys.append(char.lower())
        if not keys: return None
        now = self.clock()
        eligible = {key for key in keys if now - self.last_seen[key] >= self.REPRESS_GAP}
        # Refresh every seen key even if S wins a mixed batch; switching keys
        # must not release a held key's suppression. Never queue skips.
        for key in keys: self.last_seen[key] = now
        return 's' if 's' in eligible else ('d' if 'd' in eligible else None)

    def close(self):
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            self.saved = None
        self.enabled = False

    def __exit__(self, *exc): self.close()


class MissionClock:
    def __init__(self, clock=time.monotonic):
        self.real_clock = clock
        self.offset = 0.
        self.frozen_at = None

    @property
    def paused(self): return self.frozen_at is not None

    def __call__(self):
        return (self.frozen_at if self.paused else self.real_clock()) - self.offset

    def translate(self, captured_at): return captured_at - self.offset

    def pause(self, now):
        if not self.paused: self.frozen_at = now

    def resume(self, now):
        if self.paused:
            self.offset += now - self.frozen_at
            self.frozen_at = None


class ManualControls:
    def __init__(self, clock=time.monotonic):
        self.clock = MissionClock(clock)
        self.paused = False
        self.pending_skip = False
        self.brake_until = float('-inf')
        self.fresh_after = None

    def request(self, key, now, moving):
        import config as C
        if key == 'd':
            if self.pending_skip: return False
            self.pending_skip = True
        elif key == 's':
            self.paused = not self.paused
        else:
            return False
        if self.paused or self.pending_skip:
            self.clock.pause(now)
            if moving and now >= self.brake_until:
                self.brake_until = now + C.CROSS_FR_BRAKE_TIME
        self.update(now)
        return True

    def update(self, now):
        holding = self.paused or self.pending_skip or now < self.brake_until
        if not holding and self.clock.paused:
            self.clock.resume(now)
            self.fresh_after = now
        return holding

    def ready_to_skip(self, now):
        return self.pending_skip and now >= self.brake_until

    def complete_skip(self, now):
        self.pending_skip = False
        self.fresh_after = now
        self.update(now)

    def stop_command(self, now):
        import config as C
        from fusion_logic import Command
        braking = now < self.brake_until
        return Command('BRAKE' if braking else 'MANUAL_PAUSE', 'none',
                       C.STEER_CENTER_US, C.PARK_MANUAL_BRAKE_DUTY if braking else C.STOP_DUTY,
                       'MANUAL_SKIP' if self.pending_skip else 'MANUAL_PAUSE')


class ParkingDeadline:
    """Independent real-time guard for the remaining applied parking motion."""
    def __init__(self): self.deadline = None

    def suspend(self): self.deadline = None

    def observe(self, policy, mission_now, real_now, forward):
        parking = policy.parking
        if (policy.stage != 'PARKING' or parking.state != 'PASS_TIMED'
                or parking.pass_started_at is None or not forward):
            self.suspend()
        elif self.deadline is None:
            remaining = max(0., parking.cfg['pass_seconds'] - (mission_now - parking.pass_started_at))
            self.deadline = real_now + remaining + .05

    def expired(self, now):
        return self.deadline is not None and now >= self.deadline


def manual_allowed(controller, car=None):
    p = controller.policy
    return not (
        (car is not None and bool(car.reason))
        or getattr(p, 'state', None) in ('HALT', 'FAULT_BRAKE')
        or bool(getattr(p, 'reason', ''))
        or bool(getattr(p, 'light_fault', ''))
        or getattr(p.parking, 'state', None) in ('HALT', 'FAULT_BRAKE')
        or bool(getattr(getattr(controller, 'cone_result', None), 'stop', False))
    )


def skip_controller(controller, factory, now):
    """Reset module/detector history; preserve only genuine completion evidence."""
    if not manual_allowed(controller): raise ValueError('Manual skip cannot clear a fault')
    p = controller.policy
    if p.stage in ('PARK_SEARCH', 'PARK_SETTLE', 'PARKING', 'QR_CAPTURE'):
        return controller, 'MANUAL_FINISHED'
    target = p.order[p.order.index(p.stage) + 1]
    fresh = factory()
    while fresh.policy.stage != target:
        fresh.policy.advance(now, 'MANUAL_SKIP_' + p.stage)
    fresh.policy.cross_done = True  # Suppress re-entering a completed/skipped task.
    for attr in ('completed_cones', 'attempted_cones'):
        setattr(fresh.policy, attr, set(getattr(p, attr)))
    fresh.policy.first_cone_completed_at = p.first_cone_completed_at
    fresh.policy.unconfirmed_cones_at_parking = p.unconfirmed_cones_at_parking
    if target == 'PARK_SEARCH':
        fresh.policy.unconfirmed_cones_at_parking = max(0, p.cfg['expected_cones'] - len(p.completed_cones))
    return fresh, target
