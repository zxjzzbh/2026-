#include "smartcar/controller.hpp"
#include "smartcar/perception.hpp"
#include "smartcar/zebra.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

using namespace smartcar;
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
std::shared_ptr<const ZebraDistanceProfile> profile(bool wrong_camera = false) {
    return std::make_shared<ZebraDistanceProfile>(
        "config/calibration/crosswalk-distance-20261024.json", "secondary",
        wrong_camera ? "/dev/v4l/by-id/usb-YT-FAY-220613_USB2.0_Camera-video-index0"
                     : "/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0");
}
GroundMap narrowMap() {
    return GroundMap(cv::Matx33d(.003, 0, -.96, 0, -.006, 2.64, 0, 0, 1), {640, 480},
                     {{200, 120}, {440, 120}, {440, 400}, {200, 400}});
}
cv::Point pixel(double x, double y) {
    return {int(std::lround(320 + x / .003)), int(std::lround(440 - y / .006))};
}
cv::Mat laneAndCone(double cone_y) {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(40, 90, 45));
    for (double x : {-.585, .585})
        cv::line(image, pixel(x, .15), pixel(x, 2.1), {235, 235, 235}, 11);
    const auto a = pixel(.41, cone_y), b = pixel(.49, cone_y);
    cv::fillConvexPoly(image, std::vector<cv::Point>{{(a.x + b.x) / 2, a.y - 35}, a, b},
                      cv::Scalar(230, 75, 20));
    return image;
}
cv::Mat measuredPaperRow() {
    cv::Mat image(360, 480, CV_8UC3, cv::Scalar(150, 145, 218));
    for (int x : {65, 135, 205, 275, 345})
        cv::rectangle(image, {x, 271, 30, 28}, {240, 240, 240}, cv::FILLED);
    return image; // Every complete paper's physical near edge is image row 298.
}
Observation stable(RoadPerception &perception, const cv::Mat &image, int count = 6) {
    Observation result;
    for (int i = 1; i <= count; ++i)
        result = perception.update(image, std::uint64_t(i), i * .05);
    return result;
}
} // namespace

int main() {
    try {
        Config config;
        GroundMap actual("config/calibration/ground_road.json");
        RoadPerception replay(config, actual);
        replay.setZebraDistanceProfile(profile());
        Controller controller(config, Task::Zebra);
        std::uint64_t id = 0;
        // These are recorded outdoor images, not twelve new camera captures.
        // Each is held for three *simulated* fresh observations to exercise the
        // complete confirmation/state interface. This does not measure FPS,
        // dynamic motion or the physical stopping distance of the real run.
        for (const std::string name : {"0401", "0410", "0420", "0430"}) {
            const auto image = cv::imread("tests/fixtures/zebra_outdoor_" + name + ".jpg");
            require(!image.empty(), "real outdoor regression image missing");
            const auto image_result = detectZebraImage(image);
            require(image_result.candidate, "recorded outdoor paper row must be a visual candidate");
            Observation observation;
            Command command;
            for (int repeat = 0; repeat < 3; ++repeat) {
                ++id;
                observation = replay.update(image, id, double(id) * .05);
                command = controller.update(observation);
            }
            require(observation.zebra_detected, "fresh observed outdoor row must confirm in the pipeline");
            require(!observation.zebra && observation.zebra_y == 0,
                    "outside-profile or hood-cut paper must not invent metre distance");
            require(observation.zebra_distance_reason.find("20261024 near-edge profile:") == 0,
                    "range diagnostics must identify the authoritative measured table");
            if (name == "0430") {
                require(std::isnan(image_result.near_edge_px), "hood-cut paper cannot define the physical near edge");
                require(observation.zebra_range_status == "clipped", "occluded row must carry unusable-edge status");
                require(command.state == "zebra_brake" && command.throttle_us == config.brake_us &&
                            command.brake_event_id != 0,
                        "confirmed hood-cut near row must brake rather than continue through the crossing");
            } else {
                require(observation.zebra_range_status == "beyond_far", "distant row must remain beyond measured range");
                require(command.state == "zebra_approach" && command.throttle_us != config.brake_us,
                        "far paper must not be interpreted as an immediate stopping distance");
            }
        }

        const auto paper = measuredPaperRow();
        require(std::abs(detectZebraImage(paper).near_edge_px - 298) < 1e-9,
                "synthetic integration scene must have the known physical near-edge pixel");
        RoadPerception measured(config, actual);
        measured.setZebraDistanceProfile(profile());
        auto observation = stable(measured, paper, 3);
        require(observation.zebra && observation.zebra_detected && observation.zebra_range_status == "in_range",
                "measured row must reach the controller as an in-range crosswalk");
        require(std::abs(observation.zebra_y - (.6 + config.front_extent)) < 1e-9,
                "0.60m front distance must receive exactly one rear-axle conversion");
        Controller threshold(config, Task::Zebra);
        const auto command = threshold.update(observation);
        if (command.state != "zebra_brake")
            std::cerr << std::setprecision(17) << "threshold integration: front="
                      << observation.zebra_y - config.front_extent << " threshold="
                      << config.zebra_stop_gap + config.braking_distance << " state=" << command.state << '\n';
        const bool threshold_brakes = command.state == "zebra_brake" &&
                                      command.throttle_us == config.brake_us && command.brake_event_id > 0;

        RoadPerception wrong_camera(config, actual);
        wrong_camera.setZebraDistanceProfile(profile(true));
        const auto wrong = stable(wrong_camera, paper, 3);
        require(wrong.zebra_detected && !wrong.zebra && wrong.zebra_y == 0 &&
                    wrong.zebra_range_status == "camera_mismatch",
                "wrong physical camera cannot fall back to an old ground-map distance");
        require(wrong.zebra_distance_reason.find("20261024 near-edge profile:") == 0,
                "camera mismatch must retain the selected source in diagnostics");

        RoadPerception duplicate(config, actual);
        duplicate.setZebraDistanceProfile(profile());
        require(!duplicate.update(paper, 1, .05).zebra_detected, "one source frame cannot confirm the row");
        require(!duplicate.update(paper, 1, .10).zebra_detected &&
                    !duplicate.update(paper, 1, .15).zebra_detected &&
                    !duplicate.update(paper, 2, .20).zebra_detected,
                "repeated source frame IDs must not accumulate paper confirmation");
        require(duplicate.update(paper, 3, .25).zebra, "third independent frame must confirm valid measured row");

        auto narrow = narrowMap();
        const auto original_domain = narrow.validPolygon();
        const auto outside_pixel = pixel(.45, 1.1);
        require(!narrow.project(outside_pixel, {640, 480}), "test cone must lie outside measured polygon");
        const auto model_point = narrow.projectModel(outside_pixel, {640, 480});
        require(model_point && std::isfinite(model_point->x) && std::isfinite(model_point->y),
                "bounded lateral model must produce a separate finite point");
        RoadPerception model_perception(config, narrow);
        const auto model_observation = stable(model_perception, laneAndCone(1.1));
        require(model_observation.lane.valid && model_observation.lane.both &&
                    !model_observation.lane.metric && model_observation.lane.model_projected,
                "two outside-domain observed boundaries must retain explicit model source");
        require(std::abs(model_observation.lane.right_x - model_observation.lane.left_x - 1.17) < .04 &&
                    model_observation.lane.lateral_uncertainty_m >= .03,
                "model lane must match known physical width and retain decision uncertainty");
        require(model_observation.cones.size() == 1 && model_observation.cones.front().model_projected &&
                    model_observation.cones.front().uncertainty_m >= .06 &&
                    std::abs(model_observation.cones.front().base.x - .45) < .03,
                "observed lateral cone needs tracked ID, model source, allowance and plausible position");
        require(model_observation.cones.front().id > 0 && model_perception.diagnostics().cone_model_candidates == 1,
                "model cone must survive normal temporal tracking and expose its source count");
        require(narrow.validPolygon() == original_domain && !narrow.project(outside_pixel, {640, 480}),
                "model fallback must never enlarge the measured calibration domain");
        for (double depth : {2.05, .12}) {
            require(!narrow.projectModel(pixel(.45, depth), {640, 480}),
                    "model must not extrapolate past measured far or near depth");
            RoadPerception beyond_depth(config, narrow);
            require(stable(beyond_depth, laneAndCone(depth)).cones.empty(),
                    "out-of-depth cone must not acquire a manufactured model distance");
        }
        require(threshold_brakes,
                "measured 0.60m must trigger the 0.15m goal plus 0.45m braking reserve");
        std::cout << "road integration tests passed (static-image simulated freshness; no physical stopping claim)\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
