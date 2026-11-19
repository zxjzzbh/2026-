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
bool qualifiedModelLane(const Lane &lane) {
    return lane.valid && lane.model_projected && lane.both &&
           std::isfinite(lane.lateral_uncertainty_m) && lane.lateral_uncertainty_m >= 0 &&
           lane.lateral_uncertainty_m <= .10;
}
bool qualifiedDistanceLane(const Lane &lane) {
    return lane.valid && (lane.metric || qualifiedModelLane(lane));
}
bool qualifiedCone(const Cone &cone, const Lane &lane) {
    return std::isfinite(cone.base.x) && std::isfinite(cone.base.y) &&
           std::isfinite(cone.radius) && cone.radius > 0 &&
           (!cone.model_projected ||
            ((qualifiedModelLane(lane) || (lane.valid && lane.metric && lane.both)) &&
             std::isfinite(cone.uncertainty_m) &&
             cone.uncertainty_m >= 0 && cone.uncertainty_m <= .10));
}
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
                              c.cone_loss_s,     c.steering_gain,    c.obstacle_gain,    c.cone_pass_distance,
                              c.direction_tie_m, c.max_steering_rate, c.traffic_max_approach_s};
    for (double value : scalars)
        if (!std::isfinite(value))
            throw std::invalid_argument("nonfinite controller configuration");
    if (c.body_width <= 0 || c.wheelbase <= 0 || c.front_extent < c.wheelbase || c.rear_extent < 0 ||
        c.clearance <= 0 || c.settle_s <= 0 || c.zebra_stop_gap < 0 || c.parking_stop_gap < 0 ||
        c.tape_width <= 0 || c.lookahead <= 0 || c.recovery_s <= 0 || c.cone_loss_s <= 0 ||
        c.steering_gain <= 0 || c.obstacle_gain <= 0 || c.cone_pass_distance < 0 || c.direction_tie_m < 0 ||
        c.max_steering_rate <= 0 || c.confirm_frames < 1 || c.traffic_max_approach_s <= 0)
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
    board_count_ = board_absent_ = cones_passed_ = center_count_ = 0;
    steering_ = c_.steering_center_us;
    target_x_ = 0;
    cone_seen_ = -1;
    returning_ = false;
    announced_ = false;
    bay_locked_ = false;
    lane_prediction_valid_ = false;
    bay_side_ = cone_side_ = 0;
    zebra_seen_ = false;
    zebra_mapping_pending_ = false;
    parking_correction_active_ = false;
    zebra_y_ = 0;
    brake_event_id_ = 0;
    hold_elapsed_ = 0;
    prev_stopped_ = false;
    last_passed_cone_id_ = 0;
    cone_.reset();
    bay_ = Bay{};
    last_lane_ = Lane{};
    last_command_ = neutral("not started");
}
void Controller::transition(Stage s, double now) {
    stage_ = s;
    entered_ = now;
    hold_started_ = -1;
    hold_elapsed_ = 0;
    prev_stopped_ = false;
    center_count_ = 0;
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
Command Controller::follow(const Observation &o, double target, double gain) {
    Command q = neutral("following current geometry");
    const Lane &lane = o.lane.valid ? o.lane : last_lane_;
    const bool obstacle_prediction = stage_ == Stage::Obstacles && cone_ && o.motion_valid &&
                                     lane_prediction_valid_ && o.time - cone_seen_ <= c_.cone_loss_s;
    const double lane_age_limit = obstacle_prediction ? c_.cone_loss_s : c_.recovery_s;
    if (!lane.valid || (!o.lane.valid && o.time - last_lane_time_ > lane_age_limit))
        return neutral("lane unavailable; awaiting current geometry");
    if (!o.lane.valid && obstacle_prediction)
        q.reason = "avoiding cone with motion-predicted lane; awaiting visible white lines";
    const double half = c_.body_width / 2 + c_.clearance;
    if (lane.right_x - lane.left_x < 2 * half)
        return neutral("corridor narrower than vehicle");
    target = clamp(target, lane.left_x + half, lane.right_x - half);
    // Pure-pursuit target in metres with heading correction; gain only differs during avoidance.
    double error = std::atan2(target, std::max(.35, c_.lookahead)) + .45 * lane.heading;
    double requested = c_.steering_center_us - gain * c_.steering_gain * 420 * error;
    double dt = last_time_ < 0 ? .05 : clamp(o.time - last_time_, 0, .2);
    requested = clamp(requested, c_.steering_right_us, c_.steering_left_us);
    steering_ += clamp(requested - steering_, -c_.max_steering_rate * dt, c_.max_steering_rate * dt);
    q.throttle_us = c_.forward_us;
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
    if (cone_)
        cone_->base = transform(cone_->base);
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
Command Controller::obstacle(const Observation &o) {
    auto laneCenter = [&]() {
        const auto &lane = o.lane.valid ? o.lane : last_lane_;
        return (lane.left_x + lane.right_x) / 2;
    };
    if (cone_ && ((!qualifiedDistanceLane(o.lane.valid ? o.lane : last_lane_)) ||
                  !qualifiedCone(*cone_, o.lane)))
        return neutral("locked cone requires current qualified distance geometry");
    if (returning_) {
        if (o.lane.valid && std::abs((o.lane.left_x + o.lane.right_x) / 2) < .08 &&
            std::abs(o.lane.heading) < .12)
            ++center_count_;
        else
            center_count_ = 0;
        if (center_count_ >= c_.confirm_frames) {
            returning_ = false;
            cone_.reset();
            cone_side_ = 0;
            if (cones_passed_ >= 2) {
                transition(task_ == Task::Obstacle ? Stage::Finished : Stage::ParkingApproach, o.time);
                return neutral("both cones passed and lane recentered");
            }
        }
        // A close second cone takes priority over complete recentering.
        auto ahead = std::find_if(o.cones.begin(), o.cones.end(), [&](const Cone &v) {
            return qualifiedCone(v, o.lane) && v.base.y > c_.front_extent + .10 && v.base.y < 1.5 &&
                   (!cone_ || v.id != cone_->id);
        });
        if (cones_passed_ < 2 && ahead != o.cones.end()) {
            returning_ = false;
            cone_.reset();
            cone_side_ = 0;
        } else
            return follow(o, laneCenter(), 1.0);
    }
    if (!cone_) {
        const Cone *nearest = nullptr;
        bool unmapped_ahead = false;
        for (const auto &v : o.cones)
            if (v.base.y > c_.front_extent && v.base.y < 2.3) {
                if (!qualifiedCone(v, o.lane))
                    unmapped_ahead = true;
                else if (!nearest || v.base.y < nearest->base.y)
                    nearest = &v;
            }
        if (!nearest)
            return unmapped_ahead ? neutral("cone seen; awaiting qualified distance geometry")
                                  : follow(o, laneCenter());
        if (!o.lane.valid)
            return neutral("cone acquired; lane geometry needed to select passing side");
        // A pixel-ratio lane estimate (metric=false) is not a metre measurement and must
        // never drive the 20 cm / 50 cm side-selection decision.
        if (!qualifiedDistanceLane(o.lane))
            return neutral("cone side selection requires measured or qualified model geometry");
        // Compare the cone and the white lines at the SAME longitudinal position. The lane
        // boundaries are reported at lookahead; a cone a few dm away must be compared there,
        // not at the lookahead row, or a converging lane biases the side choice.
        double pad = c_.body_width / 2 + c_.clearance;
        const double slope = std::tan(o.lane.heading);
        const double left_at = o.lane.left_x + slope * (nearest->base.y - o.lane.reference_y);
        const double right_at = o.lane.right_x + slope * (nearest->base.y - o.lane.reference_y);
        // Distance-error allowance: conservative extra corridor so a near-threshold gap is
        // not treated as safe from a slightly optimistic ground projection.
        const double margin = std::max(0.0, o.lane.lateral_uncertainty_m) +
                              std::max(0.0, nearest->uncertainty_m) + 0.03;
        const double required = c_.body_width + 2 * c_.clearance + margin;
        double left_gap = nearest->base.x - nearest->radius - left_at;
        double right_gap = right_at - nearest->base.x - nearest->radius;
        bool left_ok = left_gap >= required;
        bool right_ok = right_gap >= required;
        if (!left_ok && !right_ok)
            return neutral("no vehicle-width passing corridor");
        cone_side_ = (!right_ok || (left_ok && left_gap - right_gap >= c_.direction_tie_m)) ? -1 : 1;
        // Requested right-boundary preference applies only when the left route fits.
        if (right_at < .50 && left_ok)
            cone_side_ = -1;
        cone_ = *nearest;
        cone_seen_ = o.time;
        target_x_ = cone_->base.x + cone_side_ * (cone_->radius + pad);
    }
    const Cone *match = nullptr;
    for (const auto &v : o.cones)
        if (v.id == cone_->id || std::hypot(v.base.x - cone_->base.x, v.base.y - cone_->base.y) < .25) {
            if (!qualifiedCone(v, o.lane))
                return neutral("tracked cone observation lacks qualified distance geometry");
            match = &v;
            break;
        }
    if (match) {
        cone_ = *match;
        cone_seen_ = o.time;
    } else if (!o.motion_valid || o.time - cone_seen_ > c_.cone_loss_s)
        return neutral("cone temporarily lost without reliable motion; retain target");
    if (cone_->base.y < -c_.rear_extent - cone_->radius - c_.cone_pass_distance) {
        // Count each physical target at most once. A duplicate/stale track id must not
        // increment the pass count a second time.
        if (cone_->id == 0 || cone_->id != last_passed_cone_id_) {
            last_passed_cone_id_ = cone_->id;
            ++cones_passed_;
        }
        returning_ = true;
        center_count_ = 0;
        return follow(o, laneCenter(), 1.0);
    }
    const Lane &lane = o.lane.valid ? o.lane : last_lane_;
    const double pad = c_.body_width / 2 + c_.clearance;
    target_x_ = cone_->base.x + cone_side_ * (cone_->radius + pad);
    if (target_x_ < lane.left_x + pad || target_x_ > lane.right_x - pad)
        return neutral("locked passing side has insufficient clearance");
    return follow(o, target_x_, c_.obstacle_gain);
}
Command Controller::parking(const Observation &o) {
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
        return neutral("duplicate or out-of-order observation");
    }
    seen_frame_ = true;
    last_frame_ = o.frame_id;
    // Bring the stage-entry clock to the first observed timestamp. A single-item run can
    // start at an arbitrary monotonic value; entering a stage with a stale 0 would make the
    // approach timeout fire immediately.
    if (last_time_ < 0)
        entered_ = o.time;
    if (!o.frame_ok) {
        if (zebra_seen_) {
            zebra_seen_ = false;
            zebra_mapping_pending_ = true;
        }
        lane_prediction_valid_ = false;
        board_count_ = board_absent_ = 0;
        prev_stopped_ = false; // an unreadable frame cannot credit stationary hold time
        steering_ = c_.steering_center_us;
        last_time_ = o.time;
        if (parking_correction_active_) {
            transition(Stage::ParkingBrake, o.time);
            Command brake = neutral("camera unavailable during final approach; requesting new brake pulse");
            brake.throttle_us = c_.brake_us;
            brake.brake_event_id = brake_event_id_;
            return brake;
        }
        return neutral("camera unavailable");
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
    case Stage::ZebraApproach:
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
    case Stage::ZebraBrake:
        if (!brakeComplete(o)) {
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
            q.reason = "zebra brake pulse";
        } else {
            transition(Stage::ZebraHold, o.time);
            q.reason = "waiting for stationary evidence";
        }
        break;
    case Stage::ZebraHold: {
        const double dt = last_time_ < 0 ? 0.0 : std::max(0.0, o.time - last_time_);
        // "Confirmed moving" is distinguishable from "cannot tell": only a valid motion
        // estimate with real displacement restarts the hold. An unreadable frame must not
        // keep zeroing (nor credit) the 10 second stationary window.
        const bool moving = o.motion_valid &&
                            (std::abs(o.forward_delta) > 0.005 || std::abs(o.lateral_delta) > 0.005 ||
                             std::abs(o.yaw_delta) > 0.02);
        if (o.stopped_confirmed) {
            if (hold_started_ < 0)
                hold_started_ = o.time;
            // Only intervals bordered by two verified-stationary frames count; the first
            // stopped frame after any unknown/moving gap contributes zero.
            if (prev_stopped_)
                hold_elapsed_ += dt;
            if (!announced_) {
                q.announce = true;
                announced_ = true;
            }
            if (hold_elapsed_ >= c_.zebra_hold_s)
                transition(task_ == Task::Zebra ? Stage::Finished : Stage::TrafficApproach, o.time);
        } else if (moving) {
            hold_started_ = -1;
            hold_elapsed_ = 0;
        }
        // Unknown frames (neither stopped-confirmed nor measured-moving) fall through here:
        // they preserve hold_elapsed_ instead of resetting it.
        prev_stopped_ = o.stopped_confirmed;
        q.reason = "10 second stationary zebra hold";
        break;
    }
    case Stage::TrafficApproach:
        q = follow(o, laneTarget());
        if (o.red || o.signal_stop) {
            transition(Stage::TrafficBrake, o.time);
            q = neutral(o.red ? "red confirmed" : "current signal requests braking");
            q.throttle_us = c_.brake_us;
            q.brake_event_id = brake_event_id_;
        } else if (o.green) {
            transition(task_ == Task::Traffic ? Stage::Finished : Stage::Obstacles, o.time);
            q.reason = "green confirmed; proceeding";
        } else if (o.time - entered_ >= c_.traffic_max_approach_s) {
            // Missing evidence never completes a rule task, including in the full race.
            // Keep processing new light observations here so a recovered red/green
            // can resume this same task without a page action or controller reset.
            q = neutral("traffic light unresolved; awaiting current signal evidence");
        }
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
        if (o.green && !o.red)
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
            q = obstacle(o);
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
    q.done = stage_ == Stage::Finished;
    if (q.throttle_us > c_.neutral_us)
        q.throttle_us = std::max(c_.forward_min_us, q.throttle_us);
    steering_ = q.steering_us;
    last_time_ = o.time;
    last_command_ = q;
    return q;
}
} // namespace smartcar
