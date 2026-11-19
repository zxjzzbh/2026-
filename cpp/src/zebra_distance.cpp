#include "smartcar/zebra_distance.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace smartcar {
namespace {
std::string stringValue(const cv::FileNode &node) {
    if (!node.isString())
        throw std::runtime_error("Malformed crosswalk calibration string");
    return std::string(node);
}
double numberValue(const cv::FileNode &node) {
    if ((!node.isInt() && !node.isReal()) || !std::isfinite(double(node)))
        throw std::runtime_error("Malformed crosswalk calibration number");
    return double(node);
}
bool trueValue(const cv::FileNode &node) {
    return node.isInt() && int(node) == 1;
}
std::string cameraFamily(std::string identity) {
    std::transform(identity.begin(), identity.end(), identity.begin(),
                   [](unsigned char ch) { return char(std::tolower(ch)); });
    const bool h65 = identity.find("h65") != std::string::npos;
    const bool usb2 = identity.find("yt-fay-220613") != std::string::npos ||
                      identity.find("usb2.0") != std::string::npos;
    if (h65 == usb2)
        return {}; // Unknown or contradictory identities must not select a table.
    return h65 ? "h65" : "usb2.0";
}
} // namespace

ZebraDistanceProfile::ZebraDistanceProfile(const std::string &path, const std::string &cameraName,
                                         const std::string &cameraIdentity) {
    std::ifstream input(std::filesystem::u8path(path), std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot read crosswalk calibration: " + path);
    std::ostringstream content;
    content << input.rdbuf();
    cv::FileStorage file(content.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                            cv::FileStorage::FORMAT_JSON);
    if (!file.isOpened() || numberValue(file["schema_version"]) != 1 ||
        stringValue(file["distance_reference"]) != "vehicle_front_to_nearest_paper_edge_m" ||
        stringValue(file["image_feature"]) != "paper_row_near_edge_y_normalized")
        throw std::runtime_error("Crosswalk calibration schema or distance reference mismatch");
    // These are the original stationary measurement flags. In particular, this
    // does not promote runtime_detector_verified or physical_stop_verified.
    if (!trueValue(file["enabled"]) || !trueValue(file["verified"]) ||
        (cameraName != "primary" && cameraName != "secondary"))
        throw std::runtime_error("Crosswalk calibration disabled or invalid camera role");
    const auto camera = file["cameras"][cameraName];
    if (camera.empty() || !trueValue(camera["verified"]) ||
        !trueValue(camera["pose"]["fixed"]) || !trueValue(camera["pose"]["verified"]))
        throw std::runtime_error("Crosswalk calibration has no measured fixed-camera profile");
    const std::string recordedFamily = cameraFamily(stringValue(camera["identity"]));
    const std::string roleFamily = cameraName == "secondary" ? "h65" : "usb2.0";
    if (recordedFamily != roleFamily)
        throw std::runtime_error("Crosswalk calibration role and recorded camera disagree");
    identity_matches_ = cameraFamily(cameraIdentity) == recordedFamily;
    const auto imageSize = camera["image_size"];
    if (!imageSize.isSeq() || imageSize.size() != 2)
        throw std::runtime_error("Malformed crosswalk image size");
    const double width = numberValue(imageSize[0]), height = numberValue(imageSize[1]);
    if (width < 2 || height < 2 || width > 100000 || height > 100000 ||
        std::floor(width) != width || std::floor(height) != height)
        throw std::runtime_error("Invalid crosswalk image size");
    size_ = {int(width), int(height)};
    const auto references = camera["references"];
    if (!references.isSeq() || references.size() < 2)
        throw std::runtime_error("Crosswalk calibration requires at least two measurements");
    for (const auto &reference : references) {
        const double y = numberValue(reference["near_edge_y_normalized"]);
        const double distance = numberValue(reference["distance_m"]);
        const double pixels = numberValue(reference["near_edge_y_px"]);
        if (!trueValue(reference["measured"]) || y <= 0 || y >= 1 || distance <= 0 ||
            std::abs(y * (size_.height - 1) - pixels) > 1e-6)
            throw std::runtime_error("Invalid crosswalk measured near-edge sample");
        samples_.push_back({y, distance});
    }
    std::sort(samples_.begin(), samples_.end(),
              [](const Sample &a, const Sample &b) { return a.y < b.y; });
    for (size_t i = 1; i < samples_.size(); ++i)
        if (samples_[i].y <= samples_[i - 1].y ||
            samples_[i].distance >= samples_[i - 1].distance)
            throw std::runtime_error("Crosswalk distance samples must decrease toward image bottom");
    const auto range = file["visible_measured_range_m"];
    if (!range.isSeq() || range.size() != 2 ||
        std::abs(numberValue(range[0]) - samples_.back().distance) > 1e-9 ||
        std::abs(numberValue(range[1]) - samples_.front().distance) > 1e-9)
        throw std::runtime_error("Crosswalk declared range disagrees with measured samples");
}

ZebraRange ZebraDistanceProfile::measure(double near_edge_px, cv::Size image_size) const {
    if (!identity_matches_)
        return {false, 0, "camera_mismatch"};
    if (image_size.width < 2 || image_size.height < 2 ||
        std::abs(double(image_size.width) / size_.width -
                 double(image_size.height) / size_.height) > 1e-6)
        return {false, 0, "unavailable"};
    if (!std::isfinite(near_edge_px) || near_edge_px <= 0 ||
        near_edge_px >= image_size.height - 1)
        return {false, 0, "clipped"};
    const double row = near_edge_px / (image_size.height - 1);
    // Only absorb arithmetic roundoff at an exact measured endpoint, not a
    // pixel tolerance that would silently extend the calibrated range.
    constexpr double roundoff = 1e-12;
    if (row < samples_.front().y - roundoff)
        return {false, 0, "beyond_far"};
    if (row > samples_.back().y + roundoff)
        return {false, 0, "too_near"};
    const double y = std::clamp(row, samples_.front().y, samples_.back().y);
    const auto upper = std::lower_bound(samples_.begin(), samples_.end(), y,
                                        [](const Sample &sample, double row) { return sample.y < row; });
    if (upper == samples_.begin() || upper->y == y)
        return {true, upper->distance, "in_range"};
    const auto lower = upper - 1;
    const double fraction = (y - lower->y) / (upper->y - lower->y);
    return {true, lower->distance + fraction * (upper->distance - lower->distance), "in_range"};
}
} // namespace smartcar
