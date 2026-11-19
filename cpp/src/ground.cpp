#include "smartcar/ground.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>

namespace smartcar {
GroundMap::GroundMap(const std::string &path) {
    std::ifstream input(std::filesystem::u8path(path), std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot read ground calibration: " + path);
    std::ostringstream content;
    content << input.rdbuf();
    cv::FileStorage f(content.str(),
                      cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    if (!f.isOpened())
        throw std::runtime_error("Cannot open ground calibration: " + path);
    std::string frame;
    f["coordinate_frame"] >> frame;
    if (frame != "rear_axle_x_right_y_forward")
        throw std::runtime_error("Ground coordinate frame mismatch");
    auto size = f["image_size"], h = f["homography"], polygon = f["valid_polygon_px"];
    if (size.size() != 2 || h.size() != 3 || polygon.size() < 3)
        throw std::runtime_error("Malformed ground calibration");
    size_ = {int(size[0]), int(size[1])};
    for (int r = 0; r < 3; ++r) {
        if (h[r].size() != 3)
            throw std::runtime_error("Malformed ground matrix");
        for (int c = 0; c < 3; ++c)
            h_(r, c) = double(h[r][c]);
    }
    for (auto row : polygon) {
        if (row.size() != 2)
            throw std::runtime_error("Malformed ground polygon");
        polygon_.emplace_back(double(row[0]), double(row[1]));
    }
    validate();
}
GroundMap::GroundMap(cv::Matx33d h, cv::Size size, std::vector<cv::Point2d> polygon)
    : h_(h), size_(size), polygon_(std::move(polygon)) {
    validate();
}
void GroundMap::validate() {
    if (size_.width < 2 || size_.height < 2 || polygon_.size() < 3)
        throw std::runtime_error("Invalid ground image domain");
    for (double x : h_.val)
        if (!std::isfinite(x))
            throw std::runtime_error("Nonfinite ground matrix");
    if (std::abs(cv::determinant(h_)) < 1e-12)
        throw std::runtime_error("Singular ground matrix");
    std::vector<cv::Point2f> poly;
    double sign = 0;
    for (auto p : polygon_) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x > size_.width - 1 ||
            p.y > size_.height - 1)
            throw std::runtime_error("Ground polygon outside image");
        const double w = h_(2, 0) * p.x + h_(2, 1) * p.y + h_(2, 2);
        if (std::abs(w) < 1e-8 || (sign && sign * w < 0))
            throw std::runtime_error("Ground domain crosses projective horizon");
        sign = w;
        poly.emplace_back(float(p.x), float(p.y));
    }
    if (!cv::isContourConvex(poly) || std::abs(cv::contourArea(poly)) < 1)
        throw std::runtime_error("Invalid ground polygon");
    inverse_ = h_.inv();
}
bool GroundMap::compatible(cv::Size s) const {
    return s.width > 1 && s.height > 1 &&
           std::abs(double(s.width) / size_.width - double(s.height) / size_.height) < 1e-6;
}
bool GroundMap::inside(cv::Point2d p) const {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        return false;
    std::vector<cv::Point2f> poly;
    for (auto v : polygon_)
        poly.emplace_back(float(v.x), float(v.y));
    return cv::pointPolygonTest(poly, cv::Point2f(float(p.x), float(p.y)), true) >= -1e-4;
}
std::optional<Point> GroundMap::project(cv::Point2d p, cv::Size size) const {
    if (!compatible(size))
        return std::nullopt;
    // OpenCV resize samples pixel centres, not image edges.
    p = {(p.x + .5) * size_.width / size.width - .5, (p.y + .5) * size_.height / size.height - .5};
    if (!inside(p))
        return std::nullopt;
    auto q = h_ * cv::Vec3d(p.x, p.y, 1);
    if (std::abs(q[2]) < 1e-8)
        return std::nullopt;
    Point result{q[0] / q[2], q[1] / q[2]};
    if (!std::isfinite(result.x) || !std::isfinite(result.y))
        return std::nullopt;
    return result;
}
std::optional<cv::Point2d> GroundMap::unproject(Point p, cv::Size size) const {
    if (!compatible(size) || !std::isfinite(p.x) || !std::isfinite(p.y))
        return std::nullopt;
    auto q = inverse_ * cv::Vec3d(p.x, p.y, 1);
    if (std::abs(q[2]) < 1e-8)
        return std::nullopt;
    cv::Point2d pixel(q[0] / q[2], q[1] / q[2]);
    if (!inside(pixel))
        return std::nullopt;
    return cv::Point2d((pixel.x + .5) * size.width / size_.width - .5,
                       (pixel.y + .5) * size.height / size_.height - .5);
}
std::optional<Point> GroundMap::projectModel(cv::Point2d p, cv::Size size, double extension) const {
    if (!compatible(size) || !std::isfinite(extension) || extension < 0 || extension > .55 ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
        p.x > size.width - 1 || p.y > size.height - 1)
        return std::nullopt;
    p = {(p.x + .5) * size_.width / size.width - .5,
         (p.y + .5) * size_.height / size.height - .5};
    const auto q = h_ * cv::Vec3d(p.x, p.y, 1);
    const auto anchor = h_ * cv::Vec3d(polygon_[0].x, polygon_[0].y, 1);
    if (std::abs(q[2]) < 1e-8 || q[2] * anchor[2] <= 0)
        return std::nullopt;
    Point result{q[0] / q[2], q[1] / q[2]};
    if (!std::isfinite(result.x) || !std::isfinite(result.y))
        return std::nullopt;
    std::vector<cv::Point2f> polygon;
    double min_y = std::numeric_limits<double>::max(), max_y = -min_y;
    for (auto v : polygon_) {
        const auto a = h_ * cv::Vec3d(v.x, v.y, 1);
        const double x = a[0] / a[2], y = a[1] / a[2];
        polygon.emplace_back(float(x), float(y));
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }
    // No near/far extension: distant cones above the calibrated horizon remain unknown.
    if (result.y < min_y || result.y > max_y ||
        cv::pointPolygonTest(polygon, cv::Point2f(float(result.x), float(result.y)), true) < -extension)
        return std::nullopt;
    return result;
}
} // namespace smartcar
