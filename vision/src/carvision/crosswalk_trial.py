"""A finite crosswalk parking trial attached to the existing, prepared driver.

No new UART/camera owner. Start is explicit, uses the same console client,
and requires a measured image reference. A neutral command is NOT stopped
feedback: use verified telemetry or an explicit on-site stopped confirmation.
This is task testing, not the full-race autonomous-readiness certification.
"""
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .crosswalk import CrosswalkDetector, CrosswalkVisionConfig
from .config import load_config
from .lane import LaneDetector


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_reference(data):
    if not isinstance(data, dict) or data.get("schema_version") not in (1, 2) or data.get("measured") is not True:
        raise ValueError("先实测并保存停车参考位置")
    if data.get("camera") not in ("primary", "secondary"):
        raise ValueError("停车参考摄像头无效")
    if data["schema_version"] == 1:
        trigger = target = data.get("front_distance_m")
        if not finite(trigger) or not 0 < trigger < .3:
            raise ValueError("旧版停车参考距离必须在 30 cm 内")
    else:
        trigger, target = data.get("trigger_distance_m"), data.get("parking_target_distance_m")
        if not finite(target) or not 0 < target < .3:
            raise ValueError("最终停稳目标必须位于斑马线前 0–30 cm 内")
        if not finite(trigger) or not target <= trigger <= 1.5:
            raise ValueError("提前回零位置须实测、在目标位置之前且不超过 1.5 m")
    if not finite(data.get("far_edge_y_normalized")) or not .4 < data["far_edge_y_normalized"] < .95:
        raise ValueError("停车参考线不在有效观察区域")
    if not isinstance(data.get("image_size"), list) or len(data["image_size"]) != 2 or any(type(v) is not int or v < 160 for v in data["image_size"]):
        raise ValueError("停车参考图像尺寸无效")
    return {**data, "trigger_distance_m": trigger, "parking_target_distance_m": target}


def load_reference(path):
    return validate_reference(json.loads(Path(path).read_text(encoding="utf-8-sig")))


class ParkingTrialLogic:
    def __init__(self, reference, *, hold_s=3, maximum_approach_s=8, resume_s=.5, mode="lane", feedback="operator"):
        if mode not in ("lane", "straight") or feedback not in ("operator", "telemetry"):
            raise ValueError("invalid trial mode/feedback")
        if hold_s != 3 or not 0 < maximum_approach_s <= 10 or not 0 < resume_s <= .8:
            raise ValueError("trial limits exceeded")
        self.reference, self.mode, self.feedback = reference, mode, feedback
        self.hold_s, self.maximum_approach_s, self.resume_s = hold_s, maximum_approach_s, resume_s
        self.phase = "idle"
        self.reason = "待开始"
        self.started = self.hold_started = self.resume_started = None
        self.last_t = None
        self.seen = False
        self.last_seen_at = None
        self.operator_stopped_at = None

    def start(self, now):
        if self.phase not in ("idle", "done", "cancelled", "fault"):
            raise ValueError("停车试验已在运行")
        self.phase, self.started, self.last_t = "approach", now, None
        self.hold_started = self.resume_started = self.operator_stopped_at = None
        self.seen = False
        self.last_seen_at = None

    def cancel(self, reason, fault=False):
        self.phase, self.reason = "fault" if fault else "cancelled", reason
        self.hold_started = None

    def confirm_stopped(self, now):
        if self.phase != "braking" or self.feedback != "operator":
            raise ValueError("仅在已发停车指令后，由现场确认车轮已停稳")
        self.operator_stopped_at = now

    def continue_after_measurement(self, now):
        if self.phase != "stopped":
            raise ValueError("须先停稳并等待满 3 秒，再单独继续")
        self.phase, self.resume_started, self.started = "resume", now, now
        self.last_t = None
        self.reason = "测量后单独继续"

    def intent(self, motor="stop", steering="center"):
        return {"motor": motor, "steering": steering, "phase": self.phase, "reason": self.reason}

    def step(self, sample, now):
        if self.phase in ("idle", "stopped", "done", "cancelled", "fault"):
            return self.intent()
        if self.last_t is not None and (now <= self.last_t or now - self.last_t > .5):
            self.cancel("处理间隔异常", True)
            return self.intent()
        self.last_t = now
        if now - self.started > 20:
            self.cancel("本轮总时限已到，已停止", True)
            return self.intent()
        if not sample.get("fresh"):
            self.cancel("摄像头画面过期，已停止", True)
            return self.intent()
        if sample.get("image_size") != self.reference["image_size"]:
            self.cancel("画面尺寸与停车参考不一致", True)
            return self.intent()
        lane_ok = sample.get("lane_valid") is True and finite(sample.get("lane_offset")) and abs(sample["lane_offset"]) <= 1
        steering = "center"
        if self.mode == "lane" and lane_ok:
            steering = "right" if sample["lane_offset"] > .08 else "left" if sample["lane_offset"] < -.08 else "center"
        if self.phase == "approach":
            if now - self.started > self.maximum_approach_s:
                self.cancel("接近时间已到，未执行继续前进", True)
                return self.intent()
            # Temporal confirmation is useful for display, but must not be the
            # permission to begin searching. Use a geometrically valid row to
            # update its position even while the display confirmation warms up.
            present = sample.get("candidate", sample.get("presence") == "present") is True
            y = sample.get("far_edge_y_normalized")
            if present and finite(y):
                self.seen = True
                self.last_seen_at = now
                if y >= self.reference["far_edge_y_normalized"]:
                    self.phase, self.reason = "braking", "已到提前回零位置，等待停稳反馈"
                    return self.intent()
            elif self.seen and now - self.last_seen_at > .25:
                self.cancel("接近后斑马线丢失，已停止", True)
                return self.intent()
            if self.mode == "lane" and not lane_ok:
                self.reason = "未识别左右赛道边线"
                return self.intent()
            self.reason = "低速接近停车参考线" if present else "低速前进搜索斑马线"
            return self.intent("forward", steering)
        if self.phase in ("braking", "hold"):
            if self.feedback == "telemetry":
                speed, age = sample.get("speed_mps"), sample.get("telemetry_age_s")
                stopped = (sample.get("telemetry_verified") is True and finite(speed) and abs(speed) <= .01
                           and finite(age) and 0 <= age <= .25)
            else:
                stopped = self.operator_stopped_at is not None
            if not stopped:
                self.phase, self.hold_started = "braking", None
                self.reason = "等待实测车速或现场停稳确认"
                return self.intent()
            if self.hold_started is None:
                self.hold_started, self.phase = now, "hold"
            if now - self.hold_started < self.hold_s:
                self.reason = f"停车等待 {max(0, self.hold_s - (now-self.hold_started)):.1f} 秒"
                return self.intent()
            self.phase = "stopped"
            self.reason = "3 秒已到，保持停止；请测量首次停车距离"
            return self.intent()
        if now - self.resume_started >= self.resume_s:
            self.phase, self.reason = "done", "本轮停车试验结束，已回零"
            return self.intent()
        if self.mode == "lane" and not lane_ok:
            self.cancel("恢复时赛道边线不可靠，保持停车", True)
            return self.intent()
        self.reason = "停车后有限继续"
        return self.intent("forward", steering)


class LocalCameraReader:
    def __init__(self, states, reference, feedback_path=None):
        self.state = states[reference["camera"]]
        self.detector = CrosswalkDetector(CrosswalkVisionConfig(confirm_ms=250, max_gap_ms=500))
        self.lane = LaneDetector(load_config(None).lane)
        self.last_id = None
        self.latest = None
        self.latest_jpeg = None
        self.feedback_path = Path(feedback_path) if feedback_path else None

    def read(self, now):
        state = self.state
        with state.condition:
            jpeg, frame_id, captured = state.raw_jpeg, state.raw_frame_id, state.raw_received_ms
            camera_error = state.state in ("error", "stopped")
        if camera_error or jpeg is None or not finite(captured) or not 0 <= now - captured / 1000 <= .5:
            return {"fresh": False}
        if frame_id != self.last_id:
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return {"fresh": False}
            zebra, _ = self.detector.detect(image, captured)
            clean = image.copy()
            for x, y, w, h in zebra["stripes_xywh"]:
                clean[y:y+h, x:x+w] = 0
            lane, _ = self.lane.detect(clean)
            self.latest = {"fresh": True, "image_size": [image.shape[1], image.shape[0]],
                           "frame_id": frame_id, "captured_monotonic_s": captured / 1000,
                           "candidate": zebra["candidate"],
                           "presence": zebra["presence"], "far_edge_y_normalized": zebra["far_edge_y_normalized"],
                           "lane_valid": lane.valid, "lane_offset": lane.offset_normalized}
            self.last_id = frame_id
            self.latest_jpeg = jpeg
        result = dict(self.latest or {"fresh": False})
        if self.feedback_path:
            try:
                packet = json.loads(self.feedback_path.read_text(encoding="utf-8"))
                t = packet.get("captured_monotonic_s")
                result.update(speed_mps=packet.get("speed_mps"), telemetry_verified=packet.get("verified") is True,
                              telemetry_age_s=now - t if finite(t) else None)
            except (OSError, ValueError):
                result["telemetry_verified"] = False
        return result


class CrosswalkTrialDrive:
    """Wrap a current BootTestDrive; keep its prepare/enable/ownership controls."""
    def __init__(self, drive, states, reference_path, log_path, *, feedback_path=None, speech_factory=None, clock=time.monotonic):
        self.drive, self.states, self.reference_path = drive, states, Path(reference_path)
        self.feedback_path, self.clock = feedback_path, clock
        self.log_path = Path(log_path)
        self.lock = threading.RLock()
        self.logic = self.reader = self.client = None
        self.thread = None
        self.stop = threading.Event()
        self.sequence = -1
        self.lease_until = 0
        self.speech_factory = speech_factory
        self.speech = None
        self.run_id = None

    def active(self):
        return self.logic is not None and self.logic.phase in ("approach", "braking", "hold", "resume")

    def status(self):
        with self.lock:
            try:
                reference = load_reference(self.reference_path)
                reference_available = True
                trial_enabled = reference.get("trial_enabled", True) is True
                reference_text = f"回零触发 {reference['trigger_distance_m']*100:g} cm；目标停稳 {reference['parking_target_distance_m']*100:g} cm"
            except (OSError, ValueError):
                reference_available, trial_enabled, reference, reference_text = False, False, None, "需实测停车参考后开始"
            trial = {"phase": self.logic.phase if self.logic else "idle",
                     "reason": self.logic.reason if self.logic else
                         (reference_text + "，等待准备和启用" if reference_available else reference_text),
                     "active": self.active(), "reference_available": reference_available,
                     "trial_enabled": trial_enabled,
                     "reference_summary": reference_text,
                     "trigger_distance_m": reference['trigger_distance_m'] if reference else None,
                     "parking_target_distance_m": reference['parking_target_distance_m'] if reference else None,
                     "can_continue": trial_enabled and self.logic is not None and self.logic.phase == "stopped",
                     "path_mode": self.logic.mode if self.logic else None,
                     "speech": self.speech.status() if self.speech else {"state":"not_requested"}}
            if reference_available and not trial_enabled:
                trial['reason'] = reference.get('disabled_reason', '固定回零短测已停用，等待低速与制动标定')
            return {**self.drive.status(), "crosswalk_trial": trial}

    def _stop_output(self, reason):
        self.drive.request("stop", {"stop_source": "stop_button"})
        if self.logic and self.active():
            self.logic.cancel(reason)

    def request(self, action, payload):
        with self.lock:
            if not action.startswith("crosswalk_"):
                if self.logic and self.logic.phase == "stopped" and action in ("stop", "emergency", "finish_test", "reset"):
                    self.logic.cancel("人工操作已结束停车验收")
                if self.active() and action not in ("status",):
                    self._stop_output("人工操作已取消自动停车试验")
                    if action == "command":
                        return self.status()  # never mix an old manual command into this trial
                return self.drive.request(action, payload)
            if not isinstance(payload, dict) or payload.get("client") != getattr(self.drive, "owner", None) or not payload.get("client"):
                raise ValueError("请使用当前已启用控制的同一页面")
            if payload.get("setup_token") != self.drive.status().get("setup_token"):
                raise ValueError("控制准备状态已变化，请刷新")
            if action == "crosswalk_start":
                if self.active() or self.thread is not None and self.thread.is_alive():
                    raise ValueError("上一轮停车试验尚未结束")
                state = self.drive.status()
                if (state.get("mode") != "enabled" or state.get("motor_pulse_us") != 1500
                        or state.get("steering_center_us") != 1610 or state.get("test_mode") != "driving"):
                    raise ValueError("先完成当前电调准备并保持零油门，使用 1610 中位驾驶模式")
                if payload.get("field_ready") is not True:
                    raise ValueError("先确认测试区域空旷、有人能断开电调")
                reference = load_reference(self.reference_path)
                if reference.get("trial_enabled", True) is not True:
                    raise ValueError(reference.get('disabled_reason', '固定回零短测已停用，等待低速与制动标定'))
                self.reader = LocalCameraReader(self.states, reference, self.feedback_path)
                if self.speech:
                    self.speech.close()
                self.speech = self.speech_factory() if self.speech_factory else None
                self.run_id = str(time.time_ns())
                self.logic = ParkingTrialLogic(reference, mode=payload.get("path_mode", "lane"),
                                               feedback="telemetry" if self.feedback_path else "operator")
                self.client = payload["client"]
                self.sequence = state.get("last_command_sequence", -1)
                self.logic.start(self.clock())
                self.stop = threading.Event()
                self.lease_until = self.clock() + .5
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                self.thread = threading.Thread(target=self._work, name="crosswalk-parking-trial", daemon=True)
                self.thread.start()
            elif action == "crosswalk_continue":
                state = self.drive.status()
                if (self.logic is None or self.active() or self.thread is not None and self.thread.is_alive()
                        or state.get("mode") != "enabled" or state.get("motor_pulse_us") != 1500):
                    raise ValueError("首次停车尚未结束或底层控制未就绪")
                policy = load_reference(self.reference_path)
                if policy.get("trial_enabled", True) is not True:
                    raise ValueError(policy.get('disabled_reason', '固定回零短测已停用，等待低速与制动标定'))
                self.logic.continue_after_measurement(self.clock())
                self.stop = threading.Event()
                self.lease_until = self.clock() + .5
                self.thread = threading.Thread(target=self._work, name="crosswalk-parking-continue", daemon=True)
                self.thread.start()
            elif action == "crosswalk_keepalive":
                if self.active() and payload["client"] == self.client:
                    self.lease_until = self.clock() + .5
            elif action == "crosswalk_stopped":
                if not self.active() or payload.get("wheels_stopped") is not True or self.drive.status().get("motor_pulse_us") != 1500:
                    raise ValueError("必须先发出停车指令并现场确认车轮停稳")
                self.logic.confirm_stopped(self.clock())
            elif action == "crosswalk_cancel":
                self._stop_output("用户取消停车试验")
            else:
                raise ValueError("unknown crosswalk trial action")
            return self.status()

    def _work(self):
        try:
            with self.log_path.open("a", encoding="utf-8") as log:
                while not self.stop.is_set():
                    with self.lock:
                        if not self.active():
                            break
                        now = self.clock()
                        if now > self.lease_until:
                            self.logic.cancel("控制页心跳中断", True)
                            break
                        sample = self.reader.read(now)
                        previous_phase = self.logic.phase
                        intent = self.logic.step(sample, now)
                        state = self.drive.status()
                        if state.get("mode") != "enabled":
                            self.logic.cancel("底层控制状态失效", True)
                            break
                        self.sequence = max(self.sequence, state.get("last_command_sequence", -1)) + 1
                        token = {"client": self.client, "sequence": self.sequence,
                                 "setup_token": state.get("setup_token"), "trial_token": state.get("trial_token"),
                                 "input_source": "crosswalk_trial"}
                        if intent["motor"] == "forward" and state.get("release_required"):
                            self.drive.request("command", {**token, "motor": "stop", "steering": "center"})
                            self.sequence += 1
                            token["sequence"] = self.sequence
                        self.drive.request("command", {**token, "motor": intent["motor"], "steering": intent["steering"]})
                        if previous_phase != intent["phase"]:
                            if intent["phase"] in ("braking", "hold") and self.reader.latest_jpeg:
                                suffix = "stop-command" if intent["phase"] == "braking" else "stopped-confirmed"
                                (self.log_path.parent / f"crosswalk-{self.run_id}-{suffix}.jpg").write_bytes(self.reader.latest_jpeg)
                            # The zero-speed command is sent BEFORE spawning audio.
                            if intent["phase"] == "hold" and self.speech:
                                self.speech.start()
                        log.write(json.dumps({"t_s": now, "intent": intent, "sample": sample,
                                              "feedback_mode": self.logic.feedback, "run_id":self.run_id,
                                              "path_mode":self.logic.mode,
                                              "speech":self.speech.status() if self.speech else None,
                                              "full_race_verified": False}) + "\n")
                        log.flush()
                    self.stop.wait(.05)
        except Exception as exc:
            with self.lock:
                self.logic.cancel(str(exc), True)
        finally:
            with self.lock:
                stop_sent = False
                try:
                    self.drive.request("stop", {"stop_source": "stop_button"})
                    stop_sent = True
                except Exception as exc:
                    self.logic.cancel("停车输出失败：" + str(exc), True)
                try:
                    with self.log_path.open("a", encoding="utf-8") as terminal:
                        terminal.write(json.dumps({"record_type":"terminal", "t_s":self.clock(),
                            "run_id":self.run_id, "intent":self.logic.intent(), "stop_command_sent":stop_sent,
                            "speech":self.speech.status() if self.speech else None},ensure_ascii=False)+"\n")
                except OSError:
                    # Disk failure must not prevent the neutral command above.
                    pass

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
        if self.speech:
            self.speech.close()
        # The original boot-console main owns and closes the underlying driver.
