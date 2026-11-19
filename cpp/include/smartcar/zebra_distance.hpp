#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace smartcar {
struct ZebraRange {
    bool valid = false;
    // Already measured from the vehicle front to the nearest paper edge.
    // Do not subtract the rear-axle-to-front offset again.
    double front_distance_m = 0;
    std::string status = "unavailable";
};

// A fixed-camera, measured near-edge table. This is not a ground homography
// and never extrapolates beyond its measured distances. Supply a whole frame;
// uniform resize is supported, but cropping or changing camera pose is not.
class ZebraDistanceProfile {
  public:
    ZebraDistanceProfile(const std::string &path, const std::string &cameraName,
                         const std::string &cameraIdentity);
    ZebraRange measure(double near_edge_px, cv::Size image_size) const;

  private:
    struct Sample {
        double y;
        double distance;
    };
    cv::Size size_;
    bool identity_matches_ = false;
    std::vector<Sample> samples_;
};
} // namespace smartcar
