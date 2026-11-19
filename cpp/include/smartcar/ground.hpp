#pragma once
#include "smartcar/types.hpp"
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace smartcar {
// Coordinates are metres relative to the rear axle, x right, y forward.
// Input must be the whole calibrated frame, optionally uniformly resized.
class GroundMap {
  public:
    explicit GroundMap(const std::string &path);
    GroundMap(cv::Matx33d homography, cv::Size image_size, std::vector<cv::Point2d> valid_polygon);
    std::optional<Point> project(cv::Point2d pixel, cv::Size frame_size) const;
    // Same calibrated model, limited lateral extension and original measured depth.
    // Callers must retain the model source and cross-check current lane width.
    std::optional<Point> projectModel(cv::Point2d pixel, cv::Size frame_size,
                                      double max_lateral_extension_m = .55) const;
    std::optional<cv::Point2d> unproject(Point ground, cv::Size frame_size) const;
    cv::Size imageSize() const { return size_; }
    const std::vector<cv::Point2d> &validPolygon() const { return polygon_; }

  private:
    cv::Matx33d h_, inverse_;
    cv::Size size_;
    std::vector<cv::Point2d> polygon_;
    void validate();
    bool compatible(cv::Size size) const;
    bool inside(cv::Point2d point) const;
};
} // namespace smartcar
