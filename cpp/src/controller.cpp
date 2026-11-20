#include "smartcar/controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace smartcar {
namespace {
double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
// Upper bound on how long a locked-but-unseen bay may still drive the car forward.
// Beyond this the car waits for a fresh bay observation instead of driving blind.
constexpr double kBayMemoryLimitS = 8.0;
constexpr double kDistanceRoundoffM = 1e-9; // arithmetic only, not a physical stopping tolerance
} // namespace
const char *Controller::name(Stage s) {
    switch (s) {
    case Stage::WaitBoard:
        return "wait_board";
    case Stage::ZebraApproach:
        return "zebra_approach";
    case Stage::ZebraBrake:
        return "zebra_brake";
    case Stage::ZebraHold:
        return "zebra_hold";
    case Stage::TrafficApproach:
        return "traffic_approach";
    case Stage::TrafficBrake:
        return "traffic_brake";
    case Stage::TrafficWait:
        return "traffic_wait_green";
    case Stage::Obstacles:
        return "two_cones";
    case Stage::ParkingApproach:
        return "parking_approach";
    case Stage::ParkingBrake:
        return "parking_brake";
    case Stage::ParkingHold:
        return "parking_hold";
    case Stage::Finished:
        return "finished";
    }
    return "unknown";
}
Controller::Controller(Config c, Task t) : c_(c) {
    const double scalars[] = {c.lane_width,      c.body_width,       c.wheelbase,        c.front_extent,
                              c.rear_extent,     c.clearance,        c.braking_distance, c.brake_s,
                              c.settle_s,        c.zebra_hold_s,     c.zebra_stop_gap,   c.parking_stop_gap,
                              c.parking_length,  c.tape_width,       c.lookahead,        c.recovery_s,
                              c.cone_loss_s,     c.steering_gain,
                              c.max_steering_rate, c.traffic_max_approach_s};
    for (double value : scalars)
        if (!std::isfinite(value))
            throw std::invalid_argument("nonfinite controller configuration");
    if (c.body_width <= 0 || c.wheelbase <= 0 || c.front_extent < c.wheelbase || c.rear_extent < 0 ||
        c.clearance <= 0 || c.settle_s <= 0 || c.zebra_stop_gap < 0 || c.parking_stop_gap < 0 ||
        c.tape_width <= 0 || c.lookahead <= 0 || c.recovery_s <= 0 || c.cone_loss_s <= 0 ||
        c.steering_gain <= 0 || c.max_steering_rate <= 0 || c.confirm_frames < 1 ||
        c.traffic_max_approach_s <= 0)
        throw std::invalid_argument("invalid controller geometry, gain or timing");
    if (c.forward_min_us < 1570 || c.forward_us < c.forward_min_us || c.forward_us > 1600 ||
        c.neutral_us != 1500 || c.brake_us != 1400 || c.brake_s <= 0 || c.brake_s > 0.5 ||
        c.zebra_hold_s < 10 || c.lane_width <= c.body_width + 2 * c.clearance || c.braking_distance <= 0 ||
        c.parking_length <= c.front_extent + c.rear_extent + c.parking_stop_gap ||
        !(c.steering_right_us < c.steering_center_us && c.steering_center_us < c.steering_left_us))
        throw std::invalid_argument("invalid vehicle/rule configuration");
    reset(t);
}
void Controller::reset(Task t) {
    task_ = t;
    stage_ = Stage::WaitBoard;
    if (t == Task::Zebra)
        stage_ = Stage::ZebraApproach;
    if (t == Task::Traffic)
        stage_ = Stage::TrafficApproach;
    if (t == Task::Obstacle)
        stage_ = Stage::Obstacles;
    if (t == Task::Parking)
        stage_ = Stage::ParkingApproach;
    entered_ = 0;
    hold_started_ = -1;
    last_time_ = -1;
    last_lane_time_ = -1;
    last_bay_time_ = -1;
    last_frame_ = 0;
    seen_frame_ = false;
    brake_feedback_required_ = false;
    board_seen_ = false;
    board_count_ = board_absent_ = cones_passed_ = 0;
    steering_ = c_.steering_center_us;
    announced_ = false;
    bay_locked_ = false;
    lane_prediction_valid_ = false;
    bay_side_ = 0;
    zebra_seen_ = false;
    zebra_mapping_pending_ = false;
    outdoor_brake_latched_ = false;
    outdoor_brake_start_ = -1.;
    parking_correction_active_ = false;
    zebra_y_ = 0;
    brake_event_id_ = 0;
    zebra_brake_retries_ = zebra_retry_motion_frames_ = 0;
    zebra_approach_confirmed_=false;
    zebra_visual_frames_=0;
    zebra_visual_first_s_=zebra_visual_last_s_=-1;
    zebra_visual_first_px_=zebra_visual_last_px_=-1;
    zebra_visual_source_.clear();
    zebra_retry_motion_since_ = -1;
    bay_ = Bay{};
    last_lane_ = Lane{};
    last_command_ = neutral("not started");
}
void Controller::transition(Stage s, double now) {
    stage_ = s;
    entered_ = now;
    hold_started_ = -1;
    // Every fresh brake entry gets its own strictly increasing, non-zero event ID so the
    // output policy executes it exactly once. Repeated frames inside the same brake pulse
    // reuse the same ID, so repeated requests never extend or re-arm the pulse.
    if (s == Stage::ZebraBrake || s == Stage::TrafficBrake || s == Stage::ParkingBrake)
        brake_event_id_ = ++brake_event_seq_;
    else
        brake_event_id_ = 0;
    if (s == Stage::ParkingBrake)
        parking_correction_active_ = false;
}
bool Controller::brakeComplete(const Observation &o) const {
    // Once a live actuator has supplied feedback, losing it cannot silently switch
    // this run to the offline timing model. Only this transaction's completed
    // neutral write is authority to leave a brake stage.
    if (brake_feedback_required_)
        return o.brake_feedback_valid && brake_event_id_ != 0 &&
               o.brake_completed_event_id == brake_event_id_;
    return o.time - entered_ >= c_.brake_s;
}
Command Controller::neutral(const std::string &reason) const {
    Command q;
    q.throttle_us = c_.neutral_us;
    q.steering_us = c_.steering_center_us;
    q.state = name(stage_);
    q.reason = reason;
    q.cones_passed = cones_passed_;
    q.done = stage_ == Stage::Finished;
    return q;
}
Command Controller::follow(const Observation &o, double target) {
    Command q = neutral("following current geometry");
    const Lane &lane = o.lane.valid ? o.lane : last_lane_;
    if (!lane.valid || (!o.lane.valid && o.time - last_lane_time_ > c_.recovery_s))
        return neutral("lane unavailable; awaiting current geometry");
    const double half = c_.body_width / 2 + c_.clearance;
    if (!o.lane.valid) {
        const double reserve=half+lane.lateral_uncertainty_m+.08;
        if(!lane_prediction_valid_ || (!lane.metric && !lane.model_projected) ||
           o.time-last_lane_time_>.25 || lane.left_x>-reserve || lane.right_x<reserve)
            return neutral("lane unavailable; no safe current-boundary recovery");
    }
    if (lane.right_x - lane.left_x < 2 * half)
        return neutral("corridor narrower than vehicle");
    target = clamp(target, lane.left_x + half, lane.right_x - half);
    // Pure-pursuit target in metres with heading correction.
    double error = std::atan2(target, std::max(.35, c_.lookahead)) + .45 * lane.heading;
    double requested = c_.steering_center_us - c_.steering_gain * 420 * error;
    double dt = last_time_ < 0 ? .05 : clamp(o.time - last_time_, 0, .2);
    requested = clamp(requested, c_.steering_right_us, c_.steering_left_us);
    steering_ += clamp(requested - steering_, -c_.max_steering_rate * dt, c_.max_steering_rate * dt);
    q.throttle_us = o.lane.valid ? c_.forward_us : c_.forward_min_us;
    q.steering_us = static_cast<int>(std::lround(steering_));
    q.target_x = target;
    return q;
}
void Controller::moveRememberedTargets(const Observation &o) {
    if (!o.motion_valid) {
        // Without a measured odometry increment the remembered/predicted geometry is no
        // longer trustworthy: drop the prediction chain now and only restore it when a
        // fresh measurement relocalises this frame (handled by the caller below).
        lane_prediction_valid_ = false;
        return;
    }
    auto transform = [&](Point p) {
        double x = p.x - o.lateral_delta, y = p.y - o.forward_delta;
        return Point{std::cos(o.yaw_delta) * x - std::sin(o.yaw_delta) * y,
                     std::sin(o.yaw_delta) * x + std::cos(o.yaw_delta) * y};
    };
    if (last_lane_.valid && lane_prediction_valid_) {
        const double heading = last_lane_.heading - o.yaw_delta, slope = std::tan(heading);
        auto lateral = [&](double x) {
            auto p = transform({x, last_lane_.reference_y});
            return p.x + slope * (last_lane_.reference_y - p.y);
        };
        const double left = lateral(last_lane_.left_x), right = lateral(last_lane_.right_x);
        if (!std::isfinite(left) || !std::isfinite(right) || std::abs(std::cos(heading)) < .15) {
            lane_prediction_valid_ = false;
            last_lane_.valid = false;
        } else {
            last_lane_.left_x = left;
            last_lane_.right_x = right;
            last_lane_.heading = heading;
            const double dt = last_time_ < 0 ? 0 : std::max(0.0, o.time - last_time_);
            last_lane_.confidence =
                std::min(.45, last_lane_.confidence) * std::exp(-dt / std::max(.1, c_.cone_loss_s));
            last_lane_.both = false; // predicted, not a current observation of both boundaries
        }
    }
    // Bay memory is only a fallback; detector supplies a motion-associated current Bay when possible.
    if (bay_locked_) {
        auto entry = transform({0, bay_.entry_y}), end = transform({0, bay_.end_y});
        const double heading = bay_.heading - o.yaw_delta, slope = std::tan(heading);
        auto lateral = [&](double x) {
            auto p = transform({x, c_.lookahead});
            return p.x + slope * (c_.lookahead - p.y);
        };
        bay_.left_x = lateral(bay_.left_x);
        bay_.divider_x = lateral(bay_.divider_x);
        bay_.right_x = lateral(bay_.right_x);
        bay_.entry_y = entry.y + slope * entry.x;
        bay_.end_y = end.y + slope * end.x;
        bay_.heading = heading;
    }
}
Command Controller::outdoorParking(const Observation &o) {
    const auto &d = o.outdoor_parking;
    if (!d || d->captured_s < 0 || o.time < d->captured_s || o.time - d->captured_s > .25)
        return neutral("waiting for fresh outdoor parking frame");
    Command q = neutral(d->reason);
    q.outdoor_parking_command = true;
    const bool recent_forward = last_command_.throttle_us > c_.neutral_us && o.time - last_time_ <= .25;
    if (d->brake && !outdoor_brake_latched_ && outdoor_brake_start_ < 0 && recent_forward) {
        brake_event_id_ = ++brake_event_seq_;
        outdoor_brake_start_ = o.time;
    }
    outdoor_brake_latched_ = d->brake;
    if (outdoor_brake_start_ >= 0) {
        bool completed = brake_feedback_required_ ?
            (o.brake_feedback_valid && o.brake_completed_event_id == brake_event_id_) :
            o.time - outdoor_brake_start_ >= .25;
        if (!completed) {
            q.brake_event_id = brake_event_id_;
            q.throttle_us = c_.brake_us;
            return q;
        }
        outdoor_brake_start_ = -1.;
    }
    if (d->complete && o.stopped_confirmed) {
        transition(Stage::Finished, o.time);
        return neutral("outdoor parking sequence complete; stationary confirmed");
    }
    if (d->brake || d->complete || d->duty <= 12000) return q;
    if (d->duty > 12700 || d->steering < 1500 || d->steering > 2000)
        return neutral("outdoor parking command outside calibrated bounds");
    // Donor ESC uses 200 Hz and 40000 duty counts. Translate its servo range to S3.
    const int target = std::max(c_.forward_min_us, static_cast<int>(std::lround(d->duty / 8.0)));
    const double dt = clamp(o.time-last_time_, 0., .1);
    // S3 accepts forward commands starting at 1570 us. Preserve the donor's
    // 600 counts/s rise above that first supported step (75 us/s).
    const int previous = recent_forward ? last_command_.throttle_us : c_.forward_min_us;
    q.throttle_us = recent_forward ? std::min(target, previous + static_cast<int>(std::lround(75*dt)))
                                   : c_.forward_min_us;
    const int span = d->steering >= 1750 ? c_.steering_left_us - c_.steering_center_us
                                        : c_.steering_center_us - c_.steering_right_us;
    q.steering_us = c_.steering_center_us + static_cast<int>(std::lround((d->steering - 1750) * span / 250.0));
    return q;
}
Command Controller::parking(const Observation &o) {
    if (c_.outdoor_parking) return outdoorParking(o);
    if (o.bay.valid) {
        if (!bay_locked_) {
            if (o.bay.left_clear && !o.bay.left_blocked)
                bay_side_ = -1;
            else if (o.bay.right_clear && !o.bay.right_blocked)
                bay_side_ = 1;
            else
                return neutral("no confirmed clear bay");
            bay_locked_ = true;
        }
        bay_ = o.bay;
        last_bay_time_ = o.time;
    } else if (bay_locked_ && !o.motion_valid)
        return neutral("bay landmarks absent; awaiting tracking recovery");
    if (!bay_locked_)
        return follow(o, (o.lane.left_x + o.lane.right_x) / 2);
    if (!o.bay.valid && last_bay_time_ >= 0 && o.time - last_bay_time_ > kBayMemoryLimitS)
        return neutral("bay landmarks lost too long; awaiting relocalization");
    if ((bay_side_ < 0 && bay_.left_blocked) || (bay_side_ > 0 && bay_.right_blocked))
        return neutral("selected bay is blocked");
    double lo = bay_side_ < 0 ? bay_.left_x : bay_.divider_x + c_.tape_width / 2;
    double hi = bay_side_ < 0 ? bay_.divider_x - c_.tape_width / 2 : bay_.right_x;
    double target = (lo + hi) / 2;
    if (hi - lo < c_.body_width + 2 * c_.clearance)
        return neutral("parking half is too narrow");
    double end_gap = bay_.end_y - c_.front_extent;
    // Never treat an invisible entrance as proof that the wheels have entered.
    if (end_gap <= c_.parking_stop_gap + c_.braking_distance + kDistanceRoundoffM) {
        transition(Stage::ParkingBrake, o.time);
        Command q = neutral("endpoint braking threshold");
        q.throttle_us = c_.brake_us;
        q.brake_event_id = brake_event_id_;
        return q;
    }
    Observation guide = o;
    guide.lane.valid = true;
    guide.lane.left_x = bay_.left_x;
    guide.lane.right_x = bay_.right_x;
    guide.lane.heading = bay_.heading;
    return follow(guide, target);
}
Command Controller::update(const Observation &o) {
    if (!std::isfinite(o.time))
        throw std::invalid_argument("invalid observation time");
    brake_feedback_required_ = brake_feedback_required_ || o.brake_feedback_valid;
    if (seen_frame_ && (o.frame_id <= last_frame_ || o.time < last_time_)) {
        steering_ = c_.steering_center_us;
        last_command_ = neutral("duplicate or out-of-order observation");
        return last_command_;
    }
    seen_frame_ = true;
    last_frame_ = o.frame_id;
    // Bring the stage-entry clock to the first observed timestamp. A single-item run can
    // start at an arbitrary monotonic value; entering a stage with a stale 0 would make the
    // approach timeout fire immediately.
    if (last_time_ < 0)
        entered_ = o.time;
    if (!o.frame_ok) {
        zebra_retry_motion_frames_ = 0;
        zebra_retry_motion_since_ = -1;
        if (zebra_seen_) {
            zebra_seen_ = false;
            zebra_mapping_pending_ = true;
        }
        lane_prediction_valid_ = false;
        board_count_ = board_absent_ = 0;
        steering_ = c_.steering_center_us;
        last_time_ = o.time;
        if (parking_correction_active_) {
            transition(Stage::ParkingBrake, o.time);
            Command brake = neutral("camera unavailable during final approach; requesting new brake pulse");
            brake.throttle_us = c_.brake_us;
            brake.brake_event_id = brake_event_id_;
            return brake;
        }
        last_command_ = neutral("camera unavailable");
        return last_command_;
    }
    moveRememberedTargets(o);
    if (o.lane.valid) {
        last_lane_ = o.lane;
        last_lane_time_ = o.time;
        lane_prediction_valid_ = true;
    }
    Command q = neutral("waiting");
    auto laneTarget = [&]() {
        return o.lane.valid ? (o.lane.left_x + o.lane.right_x) / 2
                            : (last_lane_.left_x + last_lane_.right_x) / 2;
    };
    if (task_ == Task::Observe) {
        q.reason = "observe only";
        last_time_ = o.time;
        return q;
    }
    switch (stage_) {
    case Stage::WaitBoard:
        q.reason = board_seen_ ? "start board removal not yet confirmed"
                               : "waiting to confirm blue start board";
        if (o.board_observation_valid) {
            if (o.board_visible) {
                board_absent_ = 0;
                board_count_ = std::min(board_count_ + 1, c_.confirm_frames);
                if (board_count_ >= c_.confirm_frames)
                    board_seen_ = true;
                q.reason = board_seen_ ? "start board confirmed; waiting for removal"
                                       : "confirming blue start board";
            } else {
                board_count_ = 0;
                if (board_seen_) {
                    board_absent_ = std::min(board_absent_ + 1, c_.confirm_frames);
                    q.reason = "confirming start board removal";
                    if (board_absent_ >= c_.confirm_frames) {
                        // Removal evidence is independent of lane detection. Keep its
                        // diagnosis explicit, but launch only with current steering guidance.
                        if (o.lane.valid) {
                            transition(Stage::ZebraApproach, o.time);
                            q.reason = "blue board removed";
                            q.launch_beep = true;
                        } else {
                            q.reason = "start board removed; waiting for visible lane";
                        }
                    }
                }
            }
        } else {
            board_count_ = board_absent_ = 0;
        }
        break;
    case Stage::ZebraApproach: {
        // A confirmed paper row approaching through the image must not be
        // forgotten when its contours fragment before reaching the measured
        // distance band. Stop conservatively; do not invent metre distance.
        if(o.zebra_detected && o.zebra_near_px>=0 && !o.zebra_source.empty()) {
            if(zebra_visual_source_!=o.zebra_source || zebra_visual_last_s_<0 ||
               o.time-zebra_visual_last_s_>.35 || o.zebra_near_px<zebra_visual_last_px_-15) {
                zebra_visual_frames_=0;
                zebra_visual_first_s_=o.time;
                zebra_visual_first_px_=o.zebra_near_px;
            }
            ++zebra_visual_frames_;
            zebra_visual_last_s_=o.time;
            zebra_visual_last_px_=o.zebra_near_px;
            zebra_visual_source_=o.zebra_source;
            if(zebra_visual_frames_>=3 && o.time-zebra_visual_first_s_>=.15 &&
               o.zebra_near_px-zebra_visual_first_px_>=12 && o.zebra_near_px>=180)
                zebra_approach_confirmed_=true;
        } else if(!o.zebra_detected && zebra_approach_confirmed_ &&
                  zebra_visual_last_s_>=0 && o.time-zebra_visual_last_s_>=.12) {
            transition(Stage::ZebraBrake,o.time);
            q=neutral("confirmed approaching crosswalk lost; conservative brake without a verified stop distance");
            q.throttle_us=c_.brake_us;
            q.brake_event_id=brake_event_id_;
            break;
        }
        if (o.zebra_detected &&
            (o.zebra_range_status == "too_near" || o.zebra_range_status == "clipped")) {
            transition(Stage::ZebraBrake, o.time);
            q = neutral(o.zebra_range_status == "too_near"
                            ? "confirmed crosswalk closer than calibrated range; braking without a verified stop distance"
                            : "confirmed crosswalk near edge unavailable; controlled brake without a verified stop distance");
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
            break;
        }
        if (o.zebra_detected || o.zebra)
            zebra_mapping_pending_ = true;
        if (o.zebra && std::isfinite(o.zebra_y)) {
            zebra_seen_ = true;
            zebra_mapping_pending_ = false;
            zebra_y_ = o.zebra_y;
        } else if (o.zebra_detected && o.zebra_range_status == "beyond_far") {
            // The dedicated measured table supplies a bounded ordering (> its far
            // range), not a fabricated metre estimate. Continue toward that range.
            zebra_seen_ = false;
            zebra_mapping_pending_ = false;
        } else if (zebra_seen_ && o.motion_valid && std::isfinite(o.forward_delta)) {
            // Hold the last confirmed crosswalk and integrate measured forward travel so a
            // brief detection dropout a few centimetres before the line does not skip the
            // brake window (the measured failure was ~3.9 cm short of the trigger distance).
            zebra_y_ -= o.forward_delta;
        } else if (zebra_seen_) {
            zebra_seen_ = false;
            zebra_mapping_pending_ = true;
        }
        if (!zebra_seen_ && zebra_mapping_pending_) {
            q = neutral("crosswalk detected; awaiting valid distance mapping: " +
                        o.zebra_distance_reason);
            break;
        }
        q = follow(o, laneTarget());
        if (zebra_seen_ && zebra_y_ - c_.front_extent <=
                               o.zebra_front_trigger.value_or(c_.zebra_stop_gap + c_.braking_distance) + kDistanceRoundoffM) {
            transition(Stage::ZebraBrake, o.time);
            q = neutral("zebra brake");
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
        }
        break;
    }
    case Stage::ZebraBrake:
        if (!brakeComplete(o)) {
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
            q.reason = "zebra brake pulse";
            break;
        }
        transition(Stage::ZebraHold, o.time);
        [[fallthrough]]; // Begin the timed neutral hold on the acknowledged pulse boundary.
    case Stage::ZebraHold: {
        const double dt = last_time_ < 0 ? 0.0 : std::max(0.0, o.time - last_time_);
        // Field-confirmed F/B ESC only. Never infer stillness from a UART ACK.
        // Three distinct, continuous motion estimates plus neutral settling
        // may request a new transaction, at most twice per zebra stop.
        const bool rolling = !o.stopped_confirmed && o.motion_valid && dt > 0 && dt <= .25 &&
            std::hypot(o.forward_delta,o.lateral_delta)/dt > .03;
        if (rolling) {
            if (!zebra_retry_motion_frames_) zebra_retry_motion_since_ = o.time;
            ++zebra_retry_motion_frames_;
        } else {
            zebra_retry_motion_frames_ = 0;
            zebra_retry_motion_since_ = -1;
        }
        if (c_.zebra_brake_retry_enabled && zebra_brake_retries_<2 &&
            o.time-entered_>=.2 && zebra_retry_motion_frames_>=3 &&
            o.time-zebra_retry_motion_since_>=.15) {
            ++zebra_brake_retries_;
            zebra_retry_motion_frames_ = 0;
            zebra_retry_motion_since_ = -1;
            transition(Stage::ZebraBrake,o.time);
            q=neutral("confirmed rolling after zebra brake; bounded F/B brake retry");
            q.throttle_us=c_.brake_us;
            q.brake_event_id=brake_event_id_;
            break;
        }
        // The countdown is a control decision. Optical flow may contradict it, but
        // absent flow does not delay its start or pause ten seconds at neutral.
        const bool displaced = o.motion_valid &&
            (std::hypot(o.forward_delta, o.lateral_delta) > .03 || std::abs(o.yaw_delta) > .08);
        const bool moving = rolling || displaced;
        if (moving)
            hold_started_ = -1;
        else if (hold_started_ < 0)
            hold_started_ = o.time;
        q.zebra_hold_started = hold_started_ >= 0;
        q.zebra_hold_elapsed_s = q.zebra_hold_started ? o.time - hold_started_ : 0;
        q.zebra_hold_remaining_s = std::max(0.0, c_.zebra_hold_s - q.zebra_hold_elapsed_s);
        if (q.zebra_hold_started && !announced_) {
            q.announce = true;
            announced_ = true;
        }
        q.reason = moving ? "measured movement during neutral hold" : "timed zebra hold after brake pulse";
        if (c_.zebra_brake_retry_enabled && zebra_brake_retries_>=2 && moving)
            q.reason = "zebra brake retry limit reached; waiting at neutral";
        if (q.zebra_hold_started && q.zebra_hold_elapsed_s >= c_.zebra_hold_s)
            transition(task_ == Task::Zebra ? Stage::Finished : Stage::TrafficApproach, o.time);
        break;
    }
    case Stage::TrafficApproach:
        q = follow(o, laneTarget());
        if (o.signal_stop) {
            transition(Stage::TrafficBrake, o.time);
            q = neutral("fixture reached one-metre reference");
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
        } else if (o.time - entered_ >= c_.traffic_max_approach_s)
            q = neutral("waiting for the one-metre fixture reference");
        break;
    case Stage::TrafficBrake:
        if (!brakeComplete(o)) {
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
        } else
            transition(Stage::TrafficWait, o.time);
        q.reason = "traffic brake pulse then neutral";
        break;
    case Stage::TrafficWait:
        if (o.stopped_confirmed && o.green && !o.red)
            transition(task_ == Task::Traffic ? Stage::Finished : Stage::Obstacles, o.time);
        q.reason = "wait for associated confirmed green";
        break;
    case Stage::Obstacles:
        if (o.cone_drive) {
            const auto &d = *o.cone_drive;
            if (d.passed < 0 || (d.complete && d.passed < 2) ||
                d.steering_us < c_.steering_right_us || d.steering_us > c_.steering_left_us ||
                (d.throttle_us != c_.neutral_us && d.throttle_us != c_.brake_us &&
                 (d.throttle_us < c_.forward_min_us || d.throttle_us > c_.forward_us)))
                throw std::runtime_error("invalid legacy cone intent");
            if (d.captured_s < 0 || o.time < d.captured_s || o.time - d.captured_s > .5) {
                q = neutral("cone observation expired");
                break;
            }
            cones_passed_ = d.passed;
            if (d.complete) {
                transition(task_ == Task::Obstacle ? Stage::Finished : Stage::ParkingApproach, o.time);
                q = task_ == Task::Obstacle ? neutral("legacy two cones passed and lane recentered") : parking(o);
            } else {
                q = neutral(d.reason);
                q.throttle_us = d.throttle_us;
                q.steering_us = d.steering_us;
                q.keep_existing_output = d.repeat_forward && d.throttle_us > c_.neutral_us;
                if (q.throttle_us == c_.brake_us) {
                    if (!brake_event_id_) brake_event_id_ = ++brake_event_seq_;
                    q.brake_event_id = brake_event_id_;
                } else {
                    brake_event_id_ = 0;
                }
            }
        } else {
            q = neutral("waiting for 2026-11-15 cone navigation");
        }
        break;
    case Stage::ParkingApproach:
        q = parking(o);
        break;
    case Stage::ParkingBrake:
        if (!brakeComplete(o)) {
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
        } else
            transition(Stage::ParkingHold, o.time);
        q.reason = "endpoint brake pulse then neutral";
        break;
    case Stage::ParkingHold: {
        if (o.bay.valid)
            bay_ = o.bay;
        // A freshly observed blockage of the selected half must never be reported as a
        // completed park, even if the car is already stationary inside the footprint.
        if (o.bay.valid && ((bay_side_ < 0 && bay_.left_blocked) ||
                            (bay_side_ > 0 && bay_.right_blocked))) {
            hold_started_ = -1;
            if (parking_correction_active_) {
                transition(Stage::ParkingBrake, o.time);
                q = neutral("braking final approach after new bay blockage");
                q.throttle_us = c_.brake_us;
                q.brake_event_id = brake_event_id_;
            } else
                q = neutral("selected bay reported blocked; not claiming completion");
            break;
        }
        double lo = bay_side_ < 0 ? bay_.left_x : bay_.divider_x + c_.tape_width / 2;
        double hi = bay_side_ < 0 ? bay_.divider_x - c_.tape_width / 2 : bay_.right_x;
        double half = c_.body_width / 2 + c_.clearance;
        // Bay lateral coordinates are measured at y=lookahead. Compare each body
        // corner in that same reference; tilted entry/end lines also constrain x.
        bool inside = true;
        const double slope = std::tan(bay_.heading);
        for (double x : {-half, half})
            for (double y : {-c_.rear_extent, c_.front_extent}) {
                const double x_reference = x - slope * (y - c_.lookahead);
                const double y_reference = y + slope * x;
                if (x_reference < lo || x_reference > hi || y_reference < bay_.entry_y ||
                    y_reference > bay_.end_y)
                    inside = false;
            }
        // A completion may only be judged against an endpoint that is currently observed,
        // not against predicted memory that has drifted past the far line.
        const bool endpoint_ok = o.bay.valid && o.bay.end_visible;
        const double end_gap = bay_.end_y - c_.front_extent;
        // Target remaining gap is ~50 cm; both a 10 cm (overshoot) and a 70 cm (short)
        // stop are wrong and must not be counted as a completed park.
        const bool gap_ok = std::abs(end_gap - c_.parking_stop_gap) <= c_.parking_gap_tolerance + kDistanceRoundoffM;
        const bool too_far = end_gap > c_.parking_stop_gap + c_.parking_gap_tolerance + kDistanceRoundoffM;
        const bool correction_room = end_gap > c_.parking_stop_gap + c_.braking_distance + kDistanceRoundoffM;
        if (too_far && correction_room && inside && endpoint_ok) {
            // Correction uses the same calibrated stopping allowance as first approach.
            // Do not drive down to the target and only then start a 45 cm brake.
            Observation guide = o;
            guide.lane.valid = true;
            guide.lane.left_x = bay_.left_x;
            guide.lane.right_x = bay_.right_x;
            guide.lane.heading = bay_.heading;
            hold_started_ = -1;
            q = follow(guide, (lo + hi) / 2);
            parking_correction_active_ = q.throttle_us >= c_.forward_min_us;
            q.reason = "continuing final approach toward 50 cm target gap";
            break;
        }
        if (parking_correction_active_) {
            transition(Stage::ParkingBrake, o.time);
            q = neutral(endpoint_ok ? "final approach requires a new brake pulse"
                                    : "braking final approach after endpoint observation loss");
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
            break;
        }
        if (endpoint_ok && inside && gap_ok && o.stopped_confirmed) {
            if (hold_started_ < 0)
                hold_started_ = o.time;
            if (o.time - hold_started_ >= c_.settle_s)
                transition(Stage::Finished, o.time);
        } else
            hold_started_ = -1;
        if (!endpoint_ok)
            q.reason = "parking endpoint not yet confirmed";
        else if (!inside)
            q.reason = "stopped; four-wheel entry not confirmed";
        else if (too_far && !correction_room)
            q.reason = "remaining correction shorter than calibrated braking distance";
        else if (!gap_ok)
            q.reason = "remaining gap off the 50 cm target";
        else
            q.reason = "verifying parked stationary near 50 cm";
        break;
    }
    case Stage::Finished:
        q = neutral("任务已完成，保持中位，等待手动终止");
        break;
    }
    q.state = name(stage_);
    q.cones_passed = cones_passed_;
    q.zebra_brake_retries = zebra_brake_retries_;
    q.done = stage_ == Stage::Finished;
    if (q.throttle_us > c_.neutral_us)
        q.throttle_us = std::max(c_.forward_min_us, q.throttle_us);
    steering_ = q.steering_us;
    last_time_ = o.time;
    last_command_ = q;
    return q;
}
} // namespace smartcar
