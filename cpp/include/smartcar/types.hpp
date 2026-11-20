#pragma once
#include <optional>
#include <string>
#include <vector>
#include <cstdint>

namespace smartcar {
struct Point {
    double x = 0, y = 0;
}; // metres, rear axle origin, right/forward positive
struct Config {
    double lane_width = 1.17, body_width = 0.30, wheelbase = 0.245;
    double front_extent = 0.335, rear_extent = 0.085, clearance = 0.04;
    double braking_distance = 0.45, brake_s = 0.35, settle_s = 0.7;
    double zebra_hold_s = 10.0, zebra_stop_gap = 0.15, parking_stop_gap = 0.50;
    double parking_length = 1.50, tape_width = 0.10, lookahead = 0.85;
    double parking_gap_tolerance = 0.10;
    double recovery_s = 1.0, cone_loss_s = 5.0, steering_gain = 1.0,
         traffic_max_approach_s = 25.0;
    double max_steering_rate = 500.0;
    int neutral_us = 1500, brake_us = 1400, forward_min_us = 1570, forward_us = 1570;
    int steering_center_us = 1610, steering_left_us = 1870, steering_right_us = 1350;
    int confirm_frames = 3;
    bool outdoor_parking = false;
    bool zebra_brake_retry_enabled = false;
};
struct Lane {
    bool valid = false, both = false;
    double left_x = 0, right_x = 0, heading = 0, confidence = 0;
    double reference_y = 0.85;
    bool metric = false; // true: measured ground projection; false: width-ratio image estimate
    bool model_projected = false; // qualified homography extrapolation, not measured calibration
    double lateral_uncertainty_m = 0.0; // conservative decision allowance, not a verified error bound
};
struct Bay {
    bool valid = false, entry_visible = false, end_visible = false;
    bool left_clear = false, right_clear = false, left_blocked = false, right_blocked = false;
    double left_x = 0, divider_x = 0, right_x = 0, entry_y = 0, end_y = 0, heading = 0;
    double confidence = 0;
    std::uint64_t generation = 0; // increases only after fresh consecutive geometry confirmation
    bool current_geometry = false; // current image anchors the bay; not solely predicted memory
};
struct LegacyConeDrive {
    int throttle_us, steering_us, passed;
    bool complete;
    double captured_s;
    std::string reason;
    bool repeat_forward = false;
};
struct OutdoorParkingDrive {
    int duty, steering;
    bool brake, complete;
    double captured_s;
    std::string stage, side, reason;
};
struct Observation {
    std::optional<OutdoorParkingDrive> outdoor_parking;
    std::optional<LegacyConeDrive> cone_drive;
    std::optional<double> zebra_front_trigger;
    std::uint64_t frame_id = 0;
    double time = 0;
    bool frame_ok = true, board_visible = false, board_observation_valid = false;
    Lane lane;
    Bay bay;
    bool zebra = false;
    double zebra_y = 0;
    bool zebra_detected = false; // confirmed image row, independent of distance availability
    std::string zebra_distance_reason;
    std::string zebra_range_status = "not_detected";
    double zebra_near_px = -1;
    std::string zebra_source;
    bool red = false, green = false; // current qualified light candidates, not arbitrary green pixels
    bool signal_stop = false; // stop-only guard evidence can precede a formal red vote
    bool motion_valid = false;
    double forward_delta = 0, lateral_delta = 0, yaw_delta = 0;
    bool stopped_confirmed = false;
    bool brake_feedback_valid = false;
    std::uint64_t brake_completed_event_id = 0;
};
struct Command {
    int throttle_us = 1500, steering_us = 1610;
    std::string state, reason;
    bool announce = false, done = false;
    bool launch_beep = false;
    bool outdoor_parking_command = false;
    bool zebra_hold_started = false;
    double zebra_hold_elapsed_s = 0, zebra_hold_remaining_s = 10;
    int zebra_brake_retries = 0;
    bool keep_existing_output = false; // no new source frame; leave the previous command's expiry intact
    int cones_passed = 0;
    double target_x = 0;
    std::uint64_t brake_event_id = 0; // one persistent identifier per brake transaction
};
} // namespace smartcar
