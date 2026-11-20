#include "smartcar/perception.hpp"
#include "smartcar/controller.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace smartcar;
namespace {
void check(bool value, const char *reason) {
    if (!value)
        throw std::runtime_error(reason);
}
GroundMap map(bool narrow = false) {
    return GroundMap(cv::Matx33d(0.003, 0, -0.96, 0, -0.006, 2.64, 0, 0, 1), {640, 480},
                     narrow ? std::vector<cv::Point2d>{{200, 70}, {440, 70}, {440, 479}, {200, 479}}
                            : std::vector<cv::Point2d>{{0, 70}, {639, 70}, {639, 479}, {0, 479}});
}
cv::Point pixel(double x, double y) {
    return {int(std::lround(320 + x / .003)), int(std::lround(440 - y / .006))};
}
void tape(cv::Mat &image, double x1, double y1, double x2, double y2, cv::Scalar color = {230, 230, 230}) {
    const auto a = pixel(x1, y1), b = pixel(x2, y2);
    cv::rectangle(image, {std::min(a.x, b.x), std::min(a.y, b.y)}, {std::max(a.x, b.x), std::max(a.y, b.y)},
                  color, cv::FILLED);
}
cv::Mat road(bool left = true, bool right = true) {
    cv::Mat im(480, 640, CV_8UC3, cv::Scalar(35, 85, 35));
    if (left)
        tape(im, -.615, .25, -.555, 2.05);
    if (right)
        tape(im, .555, .25, .615, 2.05);
    return im;
}
void cone(cv::Mat &im, double x, double y, int height = 35, double width = .08) {
    const auto a = pixel(x - width / 2, y), b = pixel(x + width / 2, y);
    std::vector<cv::Point> points{{(a.x + b.x) / 2, a.y - height}, a, b};
    cv::fillConvexPoly(im, points, cv::Scalar(230, 75, 20));
}
Observation confirm(RoadPerception &p, const cv::Mat &im, std::uint64_t start = 1, double t = 0) {
    Observation o;
    for (int i = 0; i < 3; ++i)
        o = p.update(im, start + i, t + i * .1);
    return o;
}
cv::Mat calibratedRoad(const GroundMap &ground, double left, double right) {
    std::vector<cv::Point2f> pixels, points;
    for (auto p : ground.validPolygon()) {
        const auto q = ground.project(p, ground.imageSize());
        check(bool(q), "source anchor must remain inside actual map");
        pixels.emplace_back(float(p.x), float(p.y));
        points.emplace_back(float(q->x), float(q->y));
    }
    const cv::Mat inverse = cv::findHomography(points, pixels, 0);
    cv::Mat image(ground.imageSize(), CV_8UC3, cv::Scalar(45, 90, 45));
    for (double x : {left, right}) {
        std::vector<cv::Point> strip;
        for (auto p :
             std::vector<cv::Point2d>{{x - .02, .60}, {x + .02, .60}, {x + .02, 3.0}, {x - .02, 3.0}}) {
            const double w =
                inverse.at<double>(2, 0) * p.x + inverse.at<double>(2, 1) * p.y + inverse.at<double>(2, 2);
            strip.emplace_back(int(std::lround((inverse.at<double>(0, 0) * p.x +
                                                inverse.at<double>(0, 1) * p.y + inverse.at<double>(0, 2)) /
                                               w)),
                               int(std::lround((inverse.at<double>(1, 0) * p.x +
                                                inverse.at<double>(1, 1) * p.y + inverse.at<double>(1, 2)) /
                                               w)));
        }
        cv::fillConvexPoly(image, strip, cv::Scalar(230, 230, 230));
    }
    return image;
}
} // namespace
int main() {
    try {
        Config c;
        RoadPerception p(c, map());
        auto im = road();
        auto o = p.update(im, 1, 0);
        check(!o.lane.valid, "one frame must not confirm lane");
        p.update(im, 1, .1);
        p.update(im, 1, .2);
        check(!p.update(im, 2, .3).lane.valid, "duplicate frame must not accumulate confirmation");
        o = p.update(im, 3, .4);
        check(o.lane.valid && o.lane.both && o.lane.metric, "two metric boundaries missing");
        check(std::abs(o.lane.left_x + .585) < .025 && std::abs(o.lane.right_x - .585) < .025,
              "lane metres incorrect");
        p.reset();
        o = confirm(p, road(true, false));
        check(o.lane.valid && !o.lane.both && o.lane.metric, "single real boundary fallback missing");
        check(std::abs((o.lane.right_x - o.lane.left_x) - 1.17) < .01,
              "single-side completion must use metric width");
        RoadPerception narrow(c, map(true));
        o = confirm(narrow, im);
        check(o.lane.valid && o.lane.both && !o.lane.metric,
              "both image boundaries must support uncalibrated width-ratio fallback");
        check(o.lane.model_projected && o.lane.lateral_uncertainty_m >= .03,
              "bounded same-H model lane must expose projection source and decision margin");
        {
            GroundMap actual("config/calibration/ground_road.json");
            RoadPerception actual_perception(c, actual);
            auto measured_scene = calibratedRoad(actual, -.695, .475);
            auto measured = confirm(actual_perception, measured_scene);
            check(measured.lane.valid && !measured.lane.metric && measured.lane.model_projected,
                  "actual road calibration requires explicit model projection for outside-domain edges");
            check(std::abs(measured.lane.right_x - .475) < .02,
                  "47.5cm right boundary must not become 52cm through image-centre width normalization");
            check(measured.lane.lateral_uncertainty_m >= .03,
                  "outside-domain projection must not pretend exact centimetres");
            check(!actual.unproject({.475, .85}, actual.imageSize()),
                  "line model must not enlarge the GroundMap measured domain");
            check(!measured.board_observation_valid,
                  "initial empty track cannot establish a removed launch board");
            auto board_scene = measured_scene.clone();
            cv::rectangle(board_scene, {175, 180}, {305, 285}, {230, 75, 20}, cv::FILLED);
            measured = confirm(actual_perception, board_scene, 4, .3);
            check(measured.board_visible && measured.board_observation_valid,
                  "actual map must allow visible rectangular launch board despite unavailable fixed corridor "
                  "points");
            measured = confirm(actual_perception, measured_scene, 7, .6);
            check(!measured.board_visible && measured.board_observation_valid,
                  "removal must use actual old board ROI and current ground/lane evidence");
            auto blackout = cv::Mat(measured_scene.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            check(!actual_perception.update(blackout, 10, .9).board_observation_valid,
                  "blackout is not visible board removal");
            auto washed = cv::Mat(measured_scene.size(), CV_8UC3, cv::Scalar(255, 255, 255));
            check(!actual_perception.update(washed, 11, 1.0).board_observation_valid,
                  "washed-out image is not visible board removal");
            RoadPerception triangle_detector(c, actual);
            auto triangle_scene = measured_scene.clone();
            cv::fillConvexPoly(triangle_scene, std::vector<cv::Point>{{240, 180}, {175, 285}, {305, 285}},
                               cv::Scalar(230, 75, 20));
            check(!confirm(triangle_detector, triangle_scene).board_visible,
                  "large blue cone triangle cannot arm board removal");
        }
        {
            GroundMap actual("config/calibration/ground_road.json");
            const auto clear = cv::imread("tests/fixtures/start_outdoor_clear.jpg");
            const auto held = cv::imread("tests/fixtures/start_outdoor_board_hand.jpg");
            const auto sleeve = cv::imread("tests/fixtures/start_outdoor_board_sleeve.jpg");
            check(!clear.empty() && !held.empty() && !sleeve.empty(), "outdoor start regression images missing");
            RoadPerception outdoor(c, actual);
            auto obs = confirm(outdoor, clear);
            check(obs.lane.valid && obs.lane.both && !obs.lane.metric,
                  "outdoor sky/paint junction must retain both visible image lane boundaries");
            check(std::abs((obs.lane.left_x + obs.lane.right_x) / 2) < .15,
                  "outdoor image lane must stay in the current running lane");
            check(!obs.board_observation_valid, "empty road is not a removed start board");
            obs = confirm(outdoor, held, 4, .3);
            check(obs.board_visible && outdoor.diagnostics().board_candidate,
                  "hand notch and attached dark sleeve must not hide the bright rectangular panel");
            obs = confirm(outdoor, clear, 7, .6);
            check(!obs.board_visible && obs.board_observation_valid,
                  "real outdoor panel removal must be recognised");
            outdoor.reset();
            check(confirm(outdoor, sleeve).board_visible, "second held panel pose must also confirm");
            // The observation belongs to the board, not to the independent
            // lane detector. Controller still waits for a lane before motion.
            cv::Mat floor(clear.size(), CV_8UC3, cv::Scalar(55, 85, 150));
            obs = confirm(outdoor, floor, 4, .3);
            check(!obs.lane.valid && !obs.board_visible && obs.board_observation_valid,
                  "verified board removal must survive missing lane geometry");
            check(outdoor.diagnostics().board_clear_fraction > .9,
                  "board clearance diagnostics must explain the decision");
            outdoor.reset();
            confirm(outdoor, held);
            cv::Mat blackout(clear.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            check(!confirm(outdoor, blackout, 4, .3).board_observation_valid,
                  "camera blackout after a visible panel must not count as removal");
            cv::Mat washed(clear.size(), CV_8UC3, cv::Scalar(255, 255, 255));
            check(!confirm(outdoor, washed, 7, .6).board_observation_valid,
                  "overexposure after a visible panel must not count as removal");
            // Short distant pieces of the adjacent running lanes have smaller
            // pixel separation than our full near pair. All pairs must use the
            // same observed row before selecting the surrounding corridor.
            cv::Mat multiple(clear.size(), CV_8UC3, cv::Scalar(55, 85, 150));
            cv::rectangle(multiple, {0, 0}, {479, 140}, {210, 210, 210}, cv::FILLED);
            cv::line(multiple, {40, 300}, {218, 140}, {235, 235, 235}, 6);
            cv::line(multiple, {440, 300}, {262, 140}, {235, 235, 235}, 6);
            cv::line(multiple, {80, 190}, {183, 145}, {235, 235, 235}, 6);
            cv::line(multiple, {400, 190}, {297, 145}, {235, 235, 235}, 6);
            outdoor.reset();
            obs = confirm(outdoor, multiple);
            check(obs.lane.valid && std::abs((obs.lane.left_x + obs.lane.right_x) / 2) < .05,
                  "short adjacent-lane fragments must not replace the near current-lane pair");
            cv::Mat divergent(clear.size(), CV_8UC3, cv::Scalar(55, 85, 150));
            cv::line(divergent, {100, 240}, {220, 160}, {235, 235, 235}, 6);
            cv::line(divergent, {380, 240}, {260, 160}, {235, 235, 235}, 6);
            cv::line(divergent, {120, 340}, {60, 270}, {235, 235, 235}, 6);
            cv::line(divergent, {360, 340}, {420, 270}, {235, 235, 235}, 6);
            outdoor.reset();
            obs = confirm(outdoor, divergent);
            check(obs.lane.valid && std::abs((obs.lane.left_x + obs.lane.right_x) / 2) < .05,
                  "invalid divergent segments must not push the common row below genuine lane support");
            // Exercise the actual image-to-controller interface, including the
            // two independent confirmation counters, without any actuator.
            outdoor.reset();
            Controller start_controller(c, Task::Race);
            std::uint64_t frame_id = 0;
            auto feed = [&](const cv::Mat &image) {
                ++frame_id;
                return start_controller.update(outdoor.update(image, frame_id, frame_id * .05));
            };
            for (int i = 0; i < 6; ++i)
                check(feed(clear).throttle_us == c.neutral_us, "empty initial track must never launch");
            for (int i = 0; i < 5; ++i)
                check(feed(held).throttle_us == c.neutral_us, "visible held panel must keep neutral");
            for (int i = 0; i < 3; ++i) feed(clear);
            check(start_controller.stage() == Stage::ZebraApproach,
                  "real panel removal and real lane must advance the race state");
            check(feed(clear).throttle_us >= 1570,
                  "confirmed outdoor panel removal must produce a valid forward request");
        }
        p.reset();
        auto glare = road(false, false);
        cv::rectangle(glare, {70, 90}, {570, 440}, cv::Scalar(255, 255, 255), cv::FILLED);
        check(!confirm(p, glare).lane.valid, "broad glare patch must not be a lane");
        p.reset();
        auto shadow = road(false, false);
        cv::line(shadow, {125, 100}, {125, 420}, {8, 8, 8}, 8);
        cv::line(shadow, {515, 100}, {515, 420}, {8, 8, 8}, 8);
        check(!confirm(p, shadow).lane.valid, "dark shadow edges must not be white tape");
        p.reset();
        auto zebra = road();
        for (int i = -2; i <= 2; ++i)
            tape(zebra, i * .21 - .0525, .70, i * .21 + .0525, .997);
        o = confirm(p, zebra);
        check(o.zebra && std::abs(o.zebra_y - .70) < .025, "zebra bar dimensions/spacing not recognised");
        p.reset();
        auto bad_zebra = road();
        tape(bad_zebra, -.45, .7, -.345, .997);
        tape(bad_zebra, 0, .7, .105, .997);
        tape(bad_zebra, .45, .7, .555, .997);
        check(!confirm(p, bad_zebra).zebra, "unrelated white rectangles must not form zebra");
        p.reset();
        auto first = road();
        cone(first, 0, 1.3);
        o = confirm(p, first);
        check(!o.board_visible, "cone must not be classified as a launch board");
        p.reset();
        auto board = road();
        const auto base = pixel(0, 1.6);
        cv::rectangle(board, {base.x - 65, base.y - 105}, {base.x + 65, base.y}, {230, 75, 20}, cv::FILLED);
        o = confirm(p, board);
        check(o.board_observation_valid && o.board_visible,
              "rectangular board classification missing");
        p.reset();
        auto blank = cv::Mat(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
        for (int i = 0; i < 15; ++i)
            o = p.update(blank, i + 1, i * .1);
        check(!o.motion_valid && !o.stopped_confirmed, "textureless frame cannot prove stopped");
        cv::Mat texture(480, 640, CV_8UC3, cv::Scalar(35, 35, 35));
        cv::RNG rng(571);
        for (int i = 0; i < 250; ++i) {
            const int value = rng.uniform(70, 145);
            cv::circle(texture, {rng.uniform(20, 620), rng.uniform(80, 460)}, rng.uniform(2, 5),
                       {double(value), double(value), double(value)}, cv::FILLED);
        }
        p.reset();
        p.update(texture, 1, 0);
        cv::Mat moved;
        const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, -2, 0, 1, 3);
        cv::warpAffine(texture, moved, transform, texture.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
        o = p.update(moved, 2, .1);
        check(o.motion_valid, "distributed ground texture should provide motion");
        check(std::abs(o.forward_delta - .018) < .004 && std::abs(o.lateral_delta - .006) < .003,
              "ground motion sign/scale incorrect");
        check(!o.stopped_confirmed, "moving frame cannot prove stopped");
        p.reset();
        for (int i = 0; i < 14; ++i)
            o = p.update(texture, i + 1, i * .1);
        check(o.motion_valid && o.stopped_confirmed,
              "sustained stationary textured ground should confirm stopped");
        check(!p.update(cv::Mat{}, 99, 2).frame_ok, "empty frame must be invalid");
        std::cout << "Road perception checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
