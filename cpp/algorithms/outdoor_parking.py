"""Run the 16:18 outdoor parking policy on the race process's own camera frames."""
from pathlib import Path
import sys
import cv2

VENDOR = Path(__file__).resolve().parent / "outdoor_parking_1618"


class OutdoorParking:
    def __init__(self):
        # The upstream modules import siblings by name. Keep the race modules first.
        for path in (VENDOR, VENDOR / "parking_modules"):
            if str(path) not in sys.path:
                sys.path.append(str(path))
        from standalone_parking_20261119 import parking_only_test as parking

        self.parking = parking
        cfg = parking.load_config(VENDOR / "race_master_config.json")
        cfg.update(cruise_duty=12700, parking_duty=12700, parking_side="AUTO",
                   lane_mode="original", white_offset=.22, entry_stop_y=.90, left_pink=True)
        self.roi = cfg["lane_roi"]
        with parking.runtime_config(True, self.roi):
            self.controller = parking.parking_controller(cfg)
        self.now = 0.
        self.controller.policy.clock = lambda: self.now
        self.previous = None
        self.started = None

    def update(self, sample, now):
        policy = self.controller.policy
        result = dict(stage=policy.stage, side=policy.parking.selected_side or "",
                      reason="WAIT_FRESH_PARKING_IMAGE", duty=12000, steer=1750,
                      brake=0, done=0, captured_s=-1.)
        if sample is None:
            return result
        image, frame_id, captured = sample
        self.now = now
        if self.started is None:
            self.started = now
        guard = self.parking.guard_reason(self.controller, self.started, captured, now, 60)
        if guard and policy.stage != "QR_CAPTURE":
            policy.halt(guard)
        with self.parking.runtime_config(True, self.roi):
            frame = cv2.resize(image, (352, 288), interpolation=cv2.INTER_AREA)
            self.parking.prepare_step_timing(self.controller)
            cmd = self.controller.step(frame, frame_id, captured, now)
            if cmd is None:
                return result
            output = self.parking.drive_command(None, cmd, captured)
            key = (policy.stage, cmd.state)
            if key != self.previous:
                policy.mark_applied(cmd, now)
            self.previous = key
        result.update(stage=policy.stage, side=policy.parking.selected_side or "",
                      reason=cmd.reason, duty=int(output.duty), steer=int(output.steer_us),
                      brake=int(output.state == "BRAKE" and output.duty < 12000),
                      done=int(policy.stage == "QR_CAPTURE" and cmd.reason == "PARKED_QR_READY"),
                      captured_s=captured)
        return result
