#pragma once
#include "smartcar/ground.hpp"

namespace smartcar {
class ParkingDetector {
  public:
    ParkingDetector(Config config, const GroundMap &ground);
    Bay detect(const cv::Mat &bgr, const Lane &lane);
    // Call once per fresh measured motion update, before detect(). Positive yaw is right.
    bool predictMotion(double forward, double lateral, double yaw);
    // Call on a missing/invalid motion increment. Repeated calls in one loss
    // episode preserve new image candidates, so a stationary car can relocalize.
    void invalidateMotion();
    // Increments only after a fresh multi-frame image acquisition succeeds.
    std::uint64_t generation() const { return generation_; }
    void reset();

  private:
    Config config_;
    GroundMap ground_;
    Bay tracked_, candidate_;
    int candidate_frames_ = 0;
    bool motion_updated_ = false;
    bool motion_lost_ = false;
    std::uint64_t generation_ = 0;
    void clearTracking();
};
} // namespace smartcar
