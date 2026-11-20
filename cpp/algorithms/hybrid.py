"""In-process adapters for the 2026-11-15 competition algorithms. No hardware I/O."""
from pathlib import Path
import json
import numpy as np

from legacy.crosswalk import CrosswalkDetector, CrosswalkVisionConfig
from legacy.crosswalk_calibration import validate_profile, estimate_distance
from legacy.crosswalk_selection import CrosswalkSelection
from legacy.traffic_signal import TrafficSignalDetector, load_profile
from legacy.traffic_voting import DualSignalVotes
from legacy.traffic_tracking import TrafficApproachGuard, fixture_box, same_fixture, stop_color_evidence
from legacy.cone_vision import ConeDetector, ImageConeTracker
from legacy.cone_road import ConeRoadContext
from legacy.cone_low_speed import trial_settings, DualLowSpeedNavigator
from legacy.cone_continuous import ContinuousConeController
from legacy.cone_recovery import FeedbackPause
from legacy.runtime_timing import VISION_MAX_AGE_S
from legacy.cone_planning import load_settings as load_cone_settings, readiness, project_tracks
from legacy.cone_navigation import reuse_legacy_ground

CAMERAS = ("secondary", "primary")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


class Crosswalk(CrosswalkSelection):
    def __init__(self, profile, white_masks, trigger_distance_m=None):
        self.profile = validate_profile(profile, require_ready=True)
        lo,hi=self.profile["overlap_range_m"]
        self.trigger = lo if trigger_distance_m is None else float(trigger_distance_m)
        if not lo<=self.trigger<=hi:raise ValueError('crosswalk trigger outside measured camera overlap')
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


from legacy.traffic_photometry import RedPhotometry


class Signals:
    def __init__(self, config, one_metre_span=None):
        if one_metre_span is not None and not 0 < one_metre_span < 480:
            raise ValueError("invalid one-metre primary lamp span")
        self.one_metre_span = one_metre_span
        self.near_box = None
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
        self.photometry = {c:RedPhotometry() for c in CAMERAS}
        self.green_streak = {c:None for c in CAMERAS}
        self.stage = None

    def update(self, views, now, stage):
        if stage == "traffic_wait_green" and self.stage != stage:
            self.votes.begin_green_wait()
            self.green_streak = {c:None for c in CAMERAS}
        self.stage = stage
        rows = {}
        for cam, (image, fid, stamp) in views.items():
            old = self.cache.get(cam)
            if old is None or (old["frame_id"], old["captured_s"]) != (fid, stamp):
                reference = self.guard.tracks[cam].box or self.stop_boxes.get(cam) or self.votes.boxes[cam]
                ranging = cam == "primary" and self.one_metre_span is not None
                if ranging:
                    reference = self.near_box
                raw=self.detector.detect(image, reference_box=reference,
                                         near_span=self.one_metre_span if ranging else None)
                raw=self.photometry[cam].update(raw,fid,stamp,now)
                old = dict(camera_id=cam, frame_id=fid, captured_s=stamp, image_size=[480,360],
                           signal=raw)
                self.cache[cam] = old
            rows[cam] = old
        approaching = stage in ("traffic_approach", "observe")
        filtered, evidence = self.guard.update(rows, now, approaching=approaching)
        for cam in evidence["confirmed_stop_color_sources"]:
            _, box = stop_color_evidence(rows[cam]["signal"], self.guard.tracks[cam].box)
            # The guard may confirm a weaker current emission frame after
            # two strong red frames. Preserve its locked real fixture even
            # when the unfiltered third row still says unknown, so that this
            # same fixture can later authorize green release.
            box = box if box is not None else self.guard.tracks[cam].box
            if box is not None and cam not in self.stop_boxes:
                self.stop_boxes[cam] = box
        for cam in CAMERAS:
            row=filtered.get(cam) or {}
            raw=row.get('signal') or {}
            stamp=row.get('captured_s',-1);fid=row.get('frame_id',-1)
            previous=self.green_streak[cam]
            if raw.get('state')!='green' or not 0<=now-stamp<=self.age:
                self.green_streak[cam]=None
            elif previous is None:
                self.green_streak[cam]=dict(first_s=stamp,last_s=stamp,fid=fid,count=1,box=raw['bbox_xyxy'])
            elif fid>previous['fid'] and stamp>previous['last_s']:
                if stamp-previous['last_s']>self.age+1e-9 or not same_fixture(previous['box'],raw['bbox_xyxy']):
                    self.green_streak[cam]=dict(first_s=stamp,last_s=stamp,fid=fid,count=1,box=raw['bbox_xyxy'])
                else:previous.update(last_s=stamp,fid=fid,count=previous['count']+1,box=raw['bbox_xyxy'])
        v = self.votes.update(filtered, now)
        self.red_sources.update(v["red_sources"])
        green_sources = set(v["current_green_sources"]) if v["green_confirmed"] else set()
        green_sources={cam for cam in green_sources if self.green_streak[cam] is not None
                       and self.green_streak[cam]['last_s']-self.green_streak[cam]['first_s']>=self.votes.green_window_s-1e-9
                       and self.green_streak[cam]['count']>=3}
        if stage == "traffic_wait_green" and self.one_metre_span is None:
            green_sources = {cam for cam in green_sources if cam in self.red_sources or
                             same_fixture(self.stop_boxes.get(cam), self.guard.tracks[cam].box)}
        if evidence["current_stop_color_sources"]:
            green_sources.clear()
            self.green_streak = {c:None for c in CAMERAS}
        stop_reason = evidence["stop_reason"] if approaching else None
        if not stop_reason and v["fast_red_timeout_sources"]:
            stop_reason = 'fast_red_timeout:' + ','.join(v["fast_red_timeout_sources"])
        if not stop_reason and v["yellow_present"]:
            stop_reason = 'confirmed_current_yellow'
        span = -1.
        stop_until = -1.
        if self.one_metre_span is not None:
            primary = rows.get("primary", {})
            raw = primary.get("signal", {})
            box = fixture_box(raw)
            centers = raw.get("lamp_centers", [])
            near = False
            if box and len(centers) == 3 and box[0] <= 240 <= box[2]:
                span = centers[2][0] - centers[0][0]
                near = span >= self.one_metre_span
            if approaching and near:
                self.near_box = box
                stop_until = primary["captured_s"] + self.age
            matched = same_fixture(self.near_box, box)
            if matched and not approaching:
                self.near_box = box
            green_sources &= {"primary"} if matched and stage == "traffic_wait_green" else set()
            stop_reason = "one_metre_fixture" if approaching and near else None
        observations=[]
        for cam,row in rows.items():
            raw=row['signal']
            brightness='/'.join(format(f['median_v'],'.1f') for f in raw.get('lamp_features',[]))
            candidate=evidence['stop_confirmation'].get(cam) or {}
            observations.append(cam+'='+raw.get('state','unknown')+':'+raw.get('reason','')
                                +':V='+brightness+':S='+('/'.join(format(f.get('median_s',-1),'.1f') for f in raw.get('lamp_features',[])))+':red_soft='+str((raw.get('lamp_features') or [{}])[0].get('soft_red_fraction',0))+':stop_frames='+str(candidate.get('frames',0))
                                +':photometry='+json.dumps(raw.get('red_photometry',{}),separators=(',',':')))
        return dict(available=int(v["any_fresh"] if self.one_metre_span is None else "primary" in rows), red=int(bool(v["red_confirmed"] or v["fast_red_sources"])),
                    stop=int(bool(stop_reason)), stop_until=stop_until, span_px=span,
                    green=int(bool(green_sources)), raw=v["raw_state"],
                    stop_reason=stop_reason or '', observation=';'.join(observations),
                    green_until=max((rows[k]["captured_s"]+self.age for k in green_sources),default=-1.))


class Cones:
    def __init__(self, settings, primary_ground, now):
        self.settings = settings
        self.roads = {k:ConeRoadContext(settings,k,turn_aware=True) for k in CAMERAS}
        self.detectors = {k:ConeDetector(settings) for k in CAMERAS}
        self.trackers = {k:ImageConeTracker(k, settings, near_field_recovery=True) for k in CAMERAS}
        self.navigator = DualLowSpeedNavigator(settings, primary_ground)
        self.navigator.set_single_recovery_limit(1.)
        self.controller = ContinuousConeController(settings, now, initialized=True, handoff_after_two=True)
        self.waiting_for_feedback = False
        self.last_forward_view = None
        self.metric_ready = readiness(settings)["metric_planning_ready"]
        self.cache = {}

    def update(self, views, now):
        images, rows = {}, {}
        import time
        began=time.perf_counter()
        for cam,(img,fid,stamp) in views.items():
            row = self.cache.get(cam)
            if row is None or (fid,stamp) != (row["frame_id"], row["captured_s"]):
                if row is not None and fid < row["frame_id"]:
                    self.trackers[cam] = ImageConeTracker(cam, self.settings, near_field_recovery=True)
                det, mask = self.detectors[cam].detect(img)
                road = self.roads[cam].inspect(img,mask,frame_id=fid,captured_s=stamp)
                self.roads[cam].enrich(det,img,road)
                tracks = self.trackers[cam].update(det,fid,stamp)
                metric, unresolved = (project_tracks(tracks,self.settings)
                                      if self.metric_ready and cam==self.settings["camera"] else ([],[]))
                row = dict(camera_id=cam,frame_id=fid,captured_s=stamp,tracks=tracks,
                           raw_candidates=sum(item.get('kind') == 'cone_candidate' for item in det),
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
        except ValueError as pause:
            # A narrow corridor is reported as ValueError by the pinned controller.
            if not isinstance(pause, FeedbackPause) and str(pause) != "白线内没有可容纳当前车体包络的空间":
                raise
            if not self.waiting_for_feedback:
                self.controller.recover_control(now)
                self.waiting_for_feedback = True
            cmd = dict(phase="waiting_for_feedback", esc_us=1500, steering_us=1610, reason=str(pause))
        phase = cmd["phase"]
        target = cmd.get("target_id")
        self.navigator.set_avoidance_target(cmd.get("target_camera") if target else None, target or None)
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
                    lane_valid=int((nav or {}).get('line_state')=='paired' and (nav or {}).get('boundary_observed',False)),
                    raw_primary=rows.get('primary',{}).get('raw_candidates',0),
                    raw_secondary=rows.get('secondary',{}).get('raw_candidates',0),
                    confirmed_primary=sum(t.get('confirmed',False) and t.get('visible',False) for t in rows.get('primary',{}).get('tracks',[])),
                    confirmed_secondary=sum(t.get('confirmed',False) and t.get('visible',False) for t in rows.get('secondary',{}).get('tracks',[])),
                    processing_ms=(time.perf_counter()-began)*1000,
                    repeat_forward=int(repeat_forward),
                    done=int(phase=="complete" and count>=2), phase=phase,
                    reason=cmd["reason"],source=camera,visible=int(visible),
                    captured_s=rows.get(camera,{}).get("captured_s",-1.))


class Hybrid:
    def __init__(self, config_file, primary_id, secondary_id):
        path = Path(config_file)
        cfg = read_json(path)
        self.profile = read_json(path.parent/cfg["zebra_distance_profile"])
        self.crosswalk = Crosswalk(self.profile, cfg.get("legacy_crosswalk_white_masks", {}),cfg.get("zebra_front_trigger_m",.4))
        self.signals = Signals(path.parent, cfg["traffic_one_metre_span_px"])
        cone_base = load_cone_settings(path.parent/"legacy/cone-trial.json")
        cone_base = reuse_legacy_ground(cone_base, read_json(path.parent/"legacy/cone-legacy-ground-candidate.json"))
        cone_base["geometry"]["lane_width_m"] = cfg["lane_width_m"]
        self.cone_settings = trial_settings(cone_base, {"test_forward_us": cfg["cone_forward_us"], "test_run_s": 120})
        self.primary_ground = read_json(path.parent/"legacy/cone-primary-ground-reference.json")
        self.cones = None
        self.parking = None
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
        views, cone_views = {}, {}
        for cam,raw,fid,ts in (("secondary",road_bytes,rid,rt),("primary",primary_bytes,pid,pt)):
            if raw is not None and 0 <= now-ts <= VISION_MAX_AGE_S:
                sample = (view(raw),fid,ts)
                if self.calibrated[cam]:
                    cone_views[cam] = sample
                if now-ts <= .25+1e-9:
                    views[cam] = sample
        metric_views = {k:v for k,v in views.items() if self.calibrated[k]}
        out = {}
        if stage in ("zebra_approach","observe"):
            out["zebra"] = self.crosswalk.update(metric_views)
        if stage.startswith("traffic_") or stage=="observe":
            out["signal"] = self.signals.update(metric_views,now,stage)
        if stage in ("two_cones","observe"):
            if self.cones is None:
                self.cones = Cones(self.cone_settings,self.primary_ground,now)
            out["cone"] = self.cones.update(cone_views,now)
        if stage in ("parking_approach", "observe"):
            if self.parking is None:
                from outdoor_parking import OutdoorParking
                self.parking = OutdoorParking()
            out["parking"] = self.parking.update(metric_views.get("secondary"), now)
        return json.dumps(out,ensure_ascii=False,allow_nan=False)

