#pragma once
#include "smartcar/types.hpp"
#include "smartcar/ground.hpp"
#include "smartcar/zebra_distance.hpp"
#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>
#include <memory>

namespace smartcar {
struct PerceptionDiagnostics {
    int white_components = 0, rejected_tapes = 0;
    int motion_features = 0, motion_inliers = 0;
    double motion_residual_m = 0;
    bool fresh = false;
    bool board_candidate = false, board_reference_valid = false;
    double board_clear_fraction = 0, board_blue_fraction = 0;
    int image_lane_candidates = 0;
    int zebra_image_stripes = 0;
    double zebra_near_edge_px = -1;
    std::string zebra_image_reason;
};

// No capture, UI, GPIO or network side effects. Times are monotonic seconds.
class RoadPerception {
  public:
    RoadPerception(const Config &config, const GroundMap &ground);
    void setZebraDistanceProfile(std::shared_ptr<const ZebraDistanceProfile> profile);
    Observation update(const cv::Mat &bgr, std::uint64_t frame_id, double time);
    const PerceptionDiagnostics &diagnostics() const { return diagnostics_; }
    void reset();

  private:
    Config config_;
    GroundMap ground_;
    std::shared_ptr<const ZebraDistanceProfile> zebra_profile_;
    Observation cached_;
    PerceptionDiagnostics diagnostics_;
    cv::Mat previous_gray_, previous_feature_mask_;
    cv::Mat calibrated_mask_;
    double previous_time_ = 0, stopped_since_ = -1;
    std::uint64_t last_frame_id_ = 0;
    bool have_frame_ = false;
    int lane_streak_ = 0, zebra_streak_ = 0, board_streak_ = 0;
    int stopped_frames_ = 0;
    Lane previous_lane_;
    cv::Rect board_candidate_, board_reference_;
    double previous_zebra_y_ = 0;
    int zebra_image_streak_ = 0;
    double previous_zebra_edge_ = -1;
    void motion(const cv::Mat &gray, const cv::Mat &mask, Observation &out);
};
} // namespace smartcar
