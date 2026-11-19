#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// Derived from the user-supplied 红绿灯识别demo.zip; see docs/traffic-demo-integration.md.
namespace ImageProcess {
enum class TrafficLightAction { NONE = 0, RED_STOP, GREEN_GO };
enum class TrafficLightState { IDLE = 0, WAIT_GREEN, COOLDOWN, GREEN_CONFIRMED = COOLDOWN };

struct TrafficLightConfig {
    int red1_h_min = 0, red1_h_max = 12, red2_h_min = 165, red2_h_max = 179;
    int red_s_min = 60, red_v_min = 55;
    int green_h_min = 40, green_h_max = 90, green_s_min = 55, green_v_min = 50;
    double roi_x_ratio_start = 0.03, roi_x_ratio_end = 0.97;
    double roi_y_ratio_start = 0.05, roi_y_ratio_end = 0.85;
    int dual_roi_enable = 0;
    double lower_roi_y_ratio_start = 0.85, lower_roi_y_ratio_end = 1.0;
    int red_confirm_frames = 2, green_confirm_frames = 3;
    double red_circle_min = 0.55, red_area_min = 60.0, red_area_max_ratio = 0.12;
    double red_aspect_min = 0.45, red_aspect_max = 2.20;
    int morph_kernel_size = 3, housing_v_max = 100;
    double confirmation_gap_s = 0.5;
};

struct TrafficLightDebugInfo {
    int state = 0, frame_width = 0, frame_height = 0, roi_count = 0;
    int red_pixels = 0, green_pixels = 0, contour_count = 0;
    int area_pass_count = 0, aspect_pass_count = 0, circle_pass_count = 0;
    double best_circle = 0, best_area = 0;
    bool red_found = false, green_found = false, red_qualified = false, green_qualified = false;
    bool fresh_frame = false, housing_found = false;
    int red_count = 0, green_count = 0;
    cv::Rect tracked_housing;
    std::string reason;
};

class TrafficLightDetector {
  public:
    TrafficLightDetector();
    void setConfig(const TrafficLightConfig &config);
    TrafficLightConfig getConfig() const { return config_; }
    void reset(double now = 0.0);
    TrafficLightAction update(const cv::Mat &bgrFrame, std::uint64_t frame_id, double time_s);
    // Compatibility helper for isolated demos only. Live consumers must supply real capture IDs/timestamps.
    TrafficLightAction update(const cv::Mat &bgrFrame);
    TrafficLightState getState() const { return state_; }
    // A historical state alone is not authority to drive. The latest observed
    // signal must still be the associated green and be fresh at control time.
    bool currentGreen(double now) const;
    TrafficLightDebugInfo getLastDebugInfo() const { return last_debug_info_; }

  private:
    struct Candidate {
        cv::Rect bulb, housing;
        cv::Point2d centre;
        double area = 0, circle = 0;
        bool horizontal = true;
    };
    TrafficLightConfig config_;
    TrafficLightState state_ = TrafficLightState::IDLE;
    TrafficLightDebugInfo last_debug_info_;
    Candidate track_, pending_red_, pending_green_;
    bool have_track_ = false, have_pending_red_ = false, have_pending_green_ = false, have_frame_ = false;
    bool reconfirm_red_after_green_ = false;
    int red_count_ = 0, green_count_ = 0;
    std::uint64_t last_frame_id_ = 0, compatibility_frame_id_ = 0;
    double last_time_ = -std::numeric_limits<double>::infinity();
    cv::Size frame_size_;

    cv::Mat roiMask(const cv::Size &size, int &roi_count) const;
    cv::Mat preprocessMask(const cv::Mat &mask) const;
    std::vector<Candidate> candidates(const cv::Mat &mask, const std::vector<cv::Rect> &housings, bool red,
                                      TrafficLightDebugInfo &debug) const;
    static bool sameHousing(const Candidate &a, const Candidate &b);
    static bool sameBulb(const Candidate &a, const Candidate &b);
    static bool greenMatchesRed(const Candidate &red, const Candidate &green);
    void breakConfirmation();
    void interruptEvidence(); // withdraw an unconsumed green latch after capture discontinuity
};
using TrafficDetector = TrafficLightDetector;
} // namespace ImageProcess
