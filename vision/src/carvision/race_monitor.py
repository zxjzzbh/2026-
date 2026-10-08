"""Live race cues for the preview. Cues never authorize vehicle motion."""

import time

import cv2


class RaceMonitor:
    def __init__(self, config):
        self.config = config
        self.qr = cv2.QRCodeDetector()
        self.qr_values = []
        self.qr_scan_at = 0.0

    def inspect(self, image, result):
        now = time.monotonic()
        if now - self.qr_scan_at >= 0.5:
            self.qr_scan_at = now
            values = []
            try:
                found, decoded, _, _ = self.qr.detectAndDecodeMulti(image)
                if found:
                    values = [v for v in decoded if v]
            except cv2.error:
                pass
            self.qr_values = values
        return {"mode": "observation_only", "hardware_output": False,
                "phase": "waiting_for_calibrated_vehicle_feedback",
                "blue_board": result.presence.get("blue_board", "unknown"),
                "crosswalk": result.presence.get("crosswalk", "unknown"),
                "traffic_light": result.traffic_light_state,
                "cone_candidates": sum(d.label == "cone" for d in result.detections),
                "qr_values": self.qr_values,
                "crosswalk_hold_s": self.config.crosswalk_hold_s,
                "payment_window_s": self.config.payment_window_s,
                "announcement_configured": self.config.announcement is not None,
                "announcement": self.config.announcement,
                "reason": "missing_calibrated_distance_telemetry_and_execution_adapter"}
