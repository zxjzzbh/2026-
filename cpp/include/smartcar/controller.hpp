#pragma once
#include "types.hpp"
#include <optional>

namespace smartcar {
enum class Task { Race, Zebra, Traffic, Obstacle, Parking, Observe };
enum class Stage {
    WaitBoard,
    ZebraApproach,
    ZebraBrake,
    ZebraHold,
    TrafficApproach,
    TrafficBrake,
    TrafficWait,
    Obstacles,
    ParkingApproach,
    ParkingBrake,
    ParkingHold,
    Finished
};
class Controller {
  public:
    explicit Controller(Config config = {}, Task task = Task::Race);
    Command update(const Observation &observation);
    void reset(Task task);
    Stage stage() const { return stage_; }
    static const char *name(Stage stage);

  private:
    Config c_;
    Task task_;
    Stage stage_ = Stage::WaitBoard;
    double entered_ = 0, hold_started_ = -1, last_time_ = -1, last_lane_time_ = -1, last_bay_time_ = -1;
    std::uint64_t last_frame_ = 0, brake_event_seq_ = 0, brake_event_id_ = 0;
    bool seen_frame_ = false, board_seen_ = false, brake_feedback_required_ = false;
    int board_count_ = 0, board_absent_ = 0, cones_passed_ = 0;
    double steering_ = 1610;
    bool announced_ = false, bay_locked_ = false, lane_prediction_valid_ = false;
    int bay_side_ = 0;
    bool zebra_seen_ = false, zebra_mapping_pending_ = false, parking_correction_active_ = false;
    double zebra_y_ = 0;
    double hold_elapsed_ = 0;
    bool prev_stopped_ = false, zebra_rebraked_ = false;
    int forward_roll_frames_ = 0;
    double forward_roll_since_ = -1;
    Bay bay_;
    Lane last_lane_;
    Command last_command_;
    void transition(Stage next, double now);
    bool brakeComplete(const Observation &o) const;
    Command neutral(const std::string &reason) const;
    Command follow(const Observation &o, double target);
    void moveRememberedTargets(const Observation &o);
    Command parking(const Observation &o);
};
} // namespace smartcar
