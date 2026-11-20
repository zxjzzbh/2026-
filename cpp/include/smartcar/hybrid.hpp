#pragma once
#include "types.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>

namespace smartcar {
struct HybridEvidence {
    int stripes = 0;
    int cones_visible = 0;
    int cone_raw_primary = 0, cone_raw_secondary = 0, cone_confirmed_primary = 0, cone_confirmed_secondary = 0;
    bool cone_lane_valid = false;
    double cone_processing_ms = 0;
    double near_px = -1;
    double green_until = -1, signal_stop_until = -1, signal_span_px = -1;
    std::string zebra_camera, cone_phase, cone_camera, signal_color;
    std::string signal_stop_reason, signal_observation;
};

// The embedded interpreter receives copies of camera pixels. Only the C++ actuator writes commands.
class HybridVision {
  public:
    HybridVision(const std::string &modules, const std::string &config, const std::string &primary,
                 const std::string &secondary);
    ~HybridVision();
    HybridEvidence update(const std::string &stage, const cv::Mat &road, const cv::Mat &primary,
                          std::uint64_t primary_id, double primary_stamp, double now, const Config &config,
                          Observation &observation);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace smartcar
