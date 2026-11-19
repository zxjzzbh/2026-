#include "smartcar/zebra_distance.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

using namespace smartcar;
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
void near(const ZebraRange &range, double distance, const char *message) {
    require(range.valid && range.status == "in_range" &&
                std::abs(range.front_distance_m - distance) < 1e-9, message);
}
struct TemporaryProfile {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("smartcar-zebra-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         ".json");
    ~TemporaryProfile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};
} // namespace
int main() {
    try {
        const std::string path = std::filesystem::absolute(
            "config/calibration/crosswalk-distance-20261024.json").u8string();
        const std::string secondary = "/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0";
        const std::string primary = "/dev/v4l/by-id/usb-YT-FAY-220613_USB2.0_Camera-video-index0";
        ZebraDistanceProfile profile(path, "secondary", secondary);
        near(profile.measure(342, {480, 360}), .4, "measured 40 cm near edge");
        near(profile.measure(316, {480, 360}), .5, "measured 50 cm near edge; front origin unchanged");
        near(profile.measure(289, {480, 360}), .65, "measured 65 cm near edge");
        near(profile.measure(298, {480, 360}), .6, "linear interpolation inside measured interval");
        near(profile.measure(316.0 / 359 * 479, {640, 480}), .5, "uniform full-frame resize");
        near(profile.measure(342.0 / 359 * 719, {960, 720}), .4, "resized exact near endpoint");
        near(profile.measure(289.0 / 359 * 719, {960, 720}), .65, "resized exact far endpoint");
        require(profile.measure(288, {480, 360}).status == "beyond_far", "no far extrapolation");
        require(profile.measure(343, {480, 360}).status == "too_near", "no near extrapolation");
        for (double row : {-1., 0., 359., 360., std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
            const auto value = profile.measure(row, {480, 360});
            require(!value.valid && value.status == "clipped", "clipped/nonfinite edge rejected");
        }
        require(!profile.measure(200, {640, 360}).valid &&
                    profile.measure(200, {640, 360}).status == "unavailable", "aspect ratio mismatch");
        require(!profile.measure(1, {0, 0}).valid, "empty image size rejected");
        ZebraDistanceProfile wrong(path, "secondary", primary);
        require(wrong.measure(316, {480, 360}).status == "camera_mismatch", "wrong physical camera rejected");
        ZebraDistanceProfile ambiguous(path, "secondary", "H65 USB2.0 Camera");
        require(!ambiguous.measure(316, {480, 360}).valid, "ambiguous physical identity rejected");
        ZebraDistanceProfile unknown(path, "secondary", "/dev/video0");
        require(unknown.measure(316, {480, 360}).status == "camera_mismatch", "unresolved device identity rejected");
        ZebraDistanceProfile human(path, "secondary", "H65 USB CAMERA: H65 USB CAMERA | usb-xhci-hcd.0-2");
        near(human.measure(316, {480, 360}), .5, "recorded human-readable physical identity");
        ZebraDistanceProfile primaryProfile(path, "primary", primary);
        near(primaryProfile.measure(352, {480, 360}), .4, "primary has its own near-edge samples");
        near(primaryProfile.measure(308, {480, 360}), .5, "primary 50 cm table not secondary table");
        near(primaryProfile.measure(261, {480, 360}), .65, "primary 65 cm table");

        std::ifstream original(std::filesystem::u8path(path), std::ios::binary);
        std::ostringstream text;
        text << original.rdbuf();
        const std::string content = text.str();
        for (const auto &change : {
                 std::pair<std::string, std::string>{"vehicle_front_to_nearest_paper_edge_m", "rear_axle_y_m"},
                 {"paper_row_near_edge_y_normalized", "paper_far_edge_y_normalized"},
                 {"\"near_edge_y_px\": 342.0", "\"near_edge_y_px\": 300.0"}}) {
            TemporaryProfile changed;
            std::string invalid = content;
            const auto pos = invalid.find(change.first);
            require(pos != std::string::npos, "malformed-profile test fixture target missing");
            invalid.replace(pos, change.first.size(), change.second);
            { std::ofstream out(changed.path, std::ios::binary); out << invalid; }
            bool rejected = false;
            try {
                ZebraDistanceProfile bad(changed.path.u8string(), "secondary", secondary);
            } catch (const std::exception &) {
                rejected = true;
            }
            require(rejected, "inconsistent origin, feature or measured sample accepted");
        }
        std::cout << "zebra distance tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
