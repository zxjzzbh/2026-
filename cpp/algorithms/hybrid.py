"""In-process adapters for the 2026-11-15 competition algorithms. No hardware I/O."""
from pathlib import Path
import json
import numpy as np

from legacy.crosswalk import CrosswalkDetector, CrosswalkVisionConfig
from legacy.crosswalk_calibration import validate_profile, estimate_distance
from legacy.crosswalk_selection import CrosswalkSelection
from legacy.traffic_signal import TrafficSignalDetector, load_profile
from legacy.traffic_voting import DualSignalVotes
from legacy.traffic_tracking import TrafficApproachGuard, same_fixture, stop_color_evidence
from legacy.cone_vision import ConeDetector, ImageConeTracker
from legacy.cone_road import ConeRoadContext
from legacy.cone_low_speed import trial_settings, DualLowSpeedNavigator
from legacy.cone_continuous import ContinuousConeController
from legacy.cone_recovery import FeedbackPause
from legacy.runtime_timing import VISION_MAX_AGE_S
from legacy.cone_planning import readiness, project_tracks

CAMERAS = ("secondary", "primary")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


class Crosswalk(CrosswalkSelection):
    def __init__(self, profile, white_masks):
        self.profile = validate_profile(profile, require_ready=True)
        self.trigger = self.profile["overlap_range_m"][0]
        self.active_source = "secondary"
        self.primary_locked = False
        self.confirmation_id = None
        self.confirmed_primary_frames = 0
        self.detectors = {}
        self.shadow_detectors = {}
        for camera in CAMERAS:
            white = {**self.profile["cameras"][camera].get("white_mask", {}), **white_masks.get(camera,{})}
            self.detectors[camera] = CrosswalkDetector(CrosswalkVisionConfig(
                white_s_max=white.get("s_max", 80 if camera == "primary" else 100),
                white_v_min=white.get("v_min", 200 if camera == "primary" else 170)))
            if "fallback_s_max" in white:
                self.shadow_detectors[camera] = CrosswalkDetector(CrosswalkVisionConfig(
                    white_s_max=white["fallback_s_max"], white_v_min=white["v_min"]))
        self.cache = {}

    def update(self, views):
        self.rows = {}
        primary_changed = False
        for cam, (image, fid, stamp) in views.items():
            prev = self.cache.get(cam)
            if prev is not None and (fid, stamp) == prev[:2]:
                self.rows[cam] = prev[2]
                continue
            detected, _ = self.detectors[cam].detect(image, stamp * 1000)
            if not detected["candidate"] and cam in self.shadow_detectors:
                detected, _ = self.shadow_detectors[cam].detect(image, stamp * 1000)
            strips = detected["stripes_xywh"]
            near = None
            distance = None
            status = "not_detected"
            # A row can remain visible when its measurable near edge leaves the frame
            # or disappears behind the bonnet. Require three comparable, complete feet.
            feet = [y+h-1 for x,y,w,h in strips if x>1 and x+w<image.shape[1]-1 and y+h<image.shape[0]-1]
            median_y = float(np.median(feet)) if feet else -1.
            tolerance = max(6., .25 * float(np.median([h for _,_,_,h in strips]))) if strips else 0.
            feet = [y for y in feet if abs(y-median_y)<=tolerance]
            if detected["bottom_clipped"] or (strips and len(feet)<3):
                status = "clipped"
            elif strips:
                near = float(np.median(feet))
                refs = self.profile["cameras"][cam]["references"]
                ys = [r["near_edge_y_normalized"] for r in refs]
                y = near / (image.shape[0]-1)
                if y < min(ys):
                    status = "beyond_far"
                elif y > max(ys):
                    status = "too_near"
                else:
                    distance = estimate_distance(self.profile, cam, y)["distance_m"]
                    status = "inside"
            row = dict(frame_id=fid, raw_frame_id=fid, captured_s=stamp, candidate=detected["candidate"],
                       metric_valid=distance is not None, near_limit_reached=status in ("clipped","too_near"),
                       distance_estimate_m=distance, status=status, stripes=len(strips), near=near,
                       presence=detected["presence"])
            self.rows[cam] = row
            self.cache[cam] = (fid, stamp, row)
            primary_changed |= cam == "primary"
        if not self.rows:
            return {"available":0}
        self._select(primary_changed)
        row = self.rows[self.active_source]
        return dict(available=1, source=self.active_source, detected=int(row["candidate"]),
                    front_m=row["distance_estimate_m"] if row["metric_valid"] else -1.,
                    status=row["status"], stripes=row["stripes"], near=row["near"] if row["near"] is not None else -1.,
                    trigger=self.trigger)


class Signals:
    def __init__(self, config):
        s = read_json(config/"legacy/traffic-driving.json")
        self.age = s["max_frame_age_s"]
        self.guard = TrafficApproachGuard(s)
        self.red_sources = set()
        self.stop_boxes = {}
        self.detector = TrafficSignalDetector(load_profile(config/"legacy/traffic-signal.json"))
        self.votes = DualSignalVotes(s["red_window_s"], s["red_confirm_percent"], self.age,
                                     green_window_s=s["green_window_s"],
                                     **{k:s[k] for k in ("fast_red_frames","fast_red_max_span_s",
                                                         "fast_red_max_gap_s","fast_red_max_age_s")})
        self.cache = {}
        self.stage = None

    def update(self, views, now, stage):
        if stage == "traffic_wait_green" and self.stage != stage:
            self.votes.begin_green_wait()
        self.stage = stage
        rows = {}
        for cam, (image, fid, stamp) in views.items():
            old = self.cache.get(cam)
            if old is None or (old["frame_id"], old["captured_s"]) != (fid, stamp):
                reference = self.guard.tracks[cam].box or self.stop_boxes.get(cam) or self.votes.boxes[cam]
                old = dict(camera_id=cam, frame_id=fid, captured_s=stamp, image_size=[480,360],
                           signal=self.detector.detect(image, reference_box=reference))
                self.cache[cam] = old
            rows[cam] = old
        approaching = stage in ("traffic_approach", "observe")
        filtered, evidence = self.guard.update(rows, now, approaching=approaching)
        for cam in evidence["current_stop_color_sources"]:
            _, box = stop_color_evidence(rows[cam]["signal"], self.guard.tracks[cam].box)
            if box is not None and cam not in self.stop_boxes:
                self.stop_boxes[cam] = box
        v = self.votes.update(filtered, now)
        self.red_sources.update(v["red_sources"])
        green_sources = set(v["current_green_sources"]) if v["green_confirmed"] else set()
        if stage == "traffic_wait_green":
            green_sources = {cam for cam in green_sources if cam in self.red_sources or
                             same_fixture(self.stop_boxes.get(cam), self.guard.tracks[cam].box)}
        if evidence["current_stop_color_sources"]:
            green_sources.clear()
        return dict(available=int(v["any_fresh"]), red=int(bool(v["red_confirmed"] or v["fast_red_sources"])),
                    stop=int(bool((approaching and evidence["stop_reason"]) or
                                  v["fast_red_timeout_sources"] or v["yellow_present"])),
                    green=int(bool(green_sources)), raw=v["raw_state"],
                    green_until=max((rows[k]["captured_s"]+self.age for k in green_sources),default=-1.))


class Cones:
    def __init__(self, settings, primary_ground, now):
        self.settings = settings
        self.roads = {k:ConeRoadContext(settings,k,turn_aware=True) for k in CAMERAS}
        self.detectors = {k:ConeDetector(settings) for k in CAMERAS}
        self.trackers = {k:ImageConeTracker(k, settings, near_field_recovery=True) for k in CAMERAS}
        self.navigator = DualLowSpeedNavigator(settings, primary_ground)
        self.controller = ContinuousConeController(settings, now, initialized=True, handoff_after_two=True)
        self.waiting_for_feedback = False
        self.last_forward_view = None
        self.metric_ready = readiness(settings)["metric_planning_ready"]
        self.cache = {}

    def update(self, views, now):
        images, rows = {}, {}
        for cam,(img,fid,stamp) in views.items():
            row = self.cache.get(cam)
            if row is None or (fid,stamp) != (row["frame_id"], row["captured_s"]):
                det, mask = self.detectors[cam].detect(img)
                road = self.roads[cam].inspect(img,mask,frame_id=fid,captured_s=stamp)
                self.roads[cam].enrich(det,img,road)
                tracks = self.trackers[cam].update(det,fid,stamp)
                metric, unresolved = (project_tracks(tracks,self.settings)
                                      if self.metric_ready and cam==self.settings["camera"] else ([],[]))
                row = dict(camera_id=cam,frame_id=fid,captured_s=stamp,tracks=tracks,
                           lane_valid=road["valid"],lane_reason=road["reason"],road_context=road,
                           metric_cones=metric,metric_unresolved=unresolved,
                           tracking_diagnostics=self.trackers[cam].latest_diagnostics)
                self.cache[cam] = row
            rows[cam], images[cam] = row, img
        nav = self.navigator.update_views(images,rows,now)
        try:
            stamp = (nav or {}).get("captured_s")
            if stamp is None or not 0 <= now - stamp <= VISION_MAX_AGE_S:
                raise FeedbackPause("等待新鲜的避障画面")
            if nav.get("boundary_danger") or nav.get("ready") is False or nav.get("action") == "stop":
                raise FeedbackPause(nav.get("reason") or "等待可通行的边界")
            cmd = self.controller.step(nav,now)
            self.waiting_for_feedback = False
        except FeedbackPause as pause:
            if not self.waiting_for_feedback:
                self.controller.recover_control(now)
                self.waiting_for_feedback = True
            cmd = dict(phase="waiting_for_feedback", esc_us=1500, steering_us=1610, reason=str(pause))
        phase = cmd["phase"]
        count = self.controller.count
        camera = (nav or {}).get("source_camera") or "secondary"
        visible = sum(t.get("visible",False) and t.get("confirmed",False) for t in rows.get(camera,{}).get("tracks",[]))
        pwm = cmd["esc_us"]
        current = rows.get(camera, {})
        view_key = (camera, current.get("frame_id"), current.get("captured_s"))
        repeat_forward = False
        if not cmd.get("fresh_control", True) and pwm > 1500:
            repeat_forward = self.last_forward_view == view_key
            if not repeat_forward:
                pwm = 1500
        self.last_forward_view = view_key if pwm > 1500 else None
        return dict(pwm=pwm, steering=cmd["steering_us"], passed=count,
                    repeat_forward=int(repeat_forward),
                    done=int(phase=="complete" and count>=2), phase=phase,
                    reason=cmd["reason"],source=camera,visible=int(visible),
                    captured_s=rows.get(camera,{}).get("captured_s",-1.))


class Hybrid:
    def __init__(self, config_file, primary_id, secondary_id):
        path = Path(config_file)
        cfg = read_json(path)
        self.profile = read_json(path.parent/cfg["zebra_distance_profile"])
        self.crosswalk = Crosswalk(self.profile, cfg.get("legacy_crosswalk_white_masks", {}))
        self.signals = Signals(path.parent)
        self.cone_settings = trial_settings(read_json(path.parent/cfg["legacy_cone_settings"]),
            {"test_forward_us":cfg["forward_us"],"test_steering_gain":cfg["obstacle_gain"]})
        self.primary_ground = read_json(path.parent/"legacy/cone-primary-ground-reference.json")
        self.cones = None
        def camera_family(identity):
            identity = identity.lower()
            h65 = "h65" in identity
            usb2 = "yt-fay-220613" in identity or "usb2.0" in identity
            if h65 == usb2:
                return None
            return "h65" if h65 else "usb2"

        self.calibrated = {}
        for role, identity, family in (("primary", primary_id, "usb2"),
                                       ("secondary", secondary_id, "h65")):
            recorded = self.profile["cameras"][role]["identity"]
            self.calibrated[role] = camera_family(identity) == camera_family(recorded) == family

    def step(self, stage, road_bytes, rid, rt, primary_bytes, pid, pt, now):
        def view(data):
            return np.frombuffer(data,dtype=np.uint8).reshape(360,480,3)
        views = {}
        for cam,raw,fid,ts in (("secondary",road_bytes,rid,rt),("primary",primary_bytes,pid,pt)):
            if raw is not None and 0 <= now-ts <= .25+1e-9:
                views[cam] = (view(raw),fid,ts)
        metric_views = {k:v for k,v in views.items() if self.calibrated[k]}
        out = {}
        if stage in ("zebra_approach","observe"):
            out["zebra"] = self.crosswalk.update(metric_views)
        if stage.startswith("traffic_") or stage=="observe":
            out["signal"] = self.signals.update(views,now,stage)
        if stage in ("two_cones","observe"):
            if self.cones is None:
                self.cones = Cones(self.cone_settings,self.primary_ground,now)
            out["cone"] = self.cones.update(metric_views,now)
        return json.dumps(out,ensure_ascii=False,allow_nan=False)

