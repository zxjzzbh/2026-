#include "smartcar/parking.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <limits>

using namespace smartcar;
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
cv::Point pixel(double x, double y) {
    return {int(std::round(320 + 200 * x)), int(std::round(450 - 100 * y))};
}
cv::Mat scene(double advance = 0, bool board_right = true, bool glare = false) {
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(45, 105, 35));
    auto tape = glare ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 220, 240);
    for (double x : {-.635, 0., .635})
        cv::rectangle(frame, pixel(x - .05, 2.60 - advance), pixel(x + .05, .90 - advance), tape, cv::FILLED);
    for (double y : {.95, 2.55})
        cv::rectangle(frame, pixel(-.685, y + .05 - advance), pixel(.685, y - .05 - advance), tape,
                      cv::FILLED);
    double bx = board_right ? .32 : -.32;
    cv::rectangle(frame, pixel(bx - .12, 2.10 - advance), pixel(bx + .12, 1.70 - advance),
                  cv::Scalar(210, 60, 20), cv::FILLED);
    return frame;
}
GroundMap map() {
    return GroundMap(cv::Matx33d(.005, 0, -1.6, 0, -.01, 4.5, 0, 0, 1), {640, 480},
                     {{0, 0}, {639, 0}, {639, 479}, {0, 479}});
}
Bay acquire(ParkingDetector &detector, const cv::Mat &image) {
    Lane lane;
    lane.valid = true;
    lane.both = true;
    lane.left_x = -.585;
    lane.right_x = .585;
    Bay bay;
    for (int i = 0; i < 4; ++i)
        bay = detector.detect(image, lane);
    return bay;
}
} // namespace
int main(int argc, char **argv) {
    try {
        GroundMap imported(std::filesystem::absolute("config/calibration/ground_parking.json").u8string());
        require(imported.imageSize() == cv::Size(480, 360),
                "imported calibration including Unicode absolute path");
        auto g = map();
        auto point = g.project({320, 250}, {640, 480});
        require(point && std::abs(point->x) < 1e-9 && std::abs(point->y - 2) < 1e-9, "ground projection");
        auto scaled = g.project({159.75, 124.75}, {320, 240});
        require(scaled && std::abs(scaled->y - 2) < 1e-9, "pixel-centre resize");
        require(!g.project({320, 250}, {640, 360}), "aspect mismatch must reject");
        require(!g.project({-1, 200}, {640, 480}), "outside domain must reject");
        auto round = g.unproject(*point, {640, 480});
        require(round && cv::norm(*round - cv::Point2d(320, 250)) < 1e-8, "projection round trip");
        bool invalid = false;
        try {
            GroundMap bad(cv::Matx33d::zeros(), {640, 480}, {{0, 0}, {639, 0}, {0, 479}});
        } catch (const std::exception &) {
            invalid = true;
        }
        require(invalid, "singular map rejected");
        Config cfg;
        ParkingDetector detector(cfg, g);
        cv::Mat image = scene(), unchanged = image.clone();
        auto bay = acquire(detector, image);
        require(cv::norm(image, unchanged, cv::NORM_INF) == 0, "detector modified original frame");
        require(bay.valid && bay.entry_visible && bay.end_visible, "complete bay acquisition");
        require(bay.current_geometry && bay.generation == detector.generation() && bay.generation > 0,
                "current image acquisition must expose its localization generation");
        require(std::abs(bay.entry_y - 1) < .15 && std::abs(bay.end_y - 2.5) < .15, "inner tape distances");
        require(bay.right_blocked && bay.left_clear && !bay.right_clear, "right board requires left slot");
        ParkingDetector other(cfg, g);
        auto other_bay = acquire(other, scene(0, false));
        require(other_bay.left_blocked && other_bay.right_clear, "left board requires right slot");
        cv::Mat outdoor = scene();
        for (double x : {-.635, .635})
            cv::rectangle(outdoor, pixel(x - .05, 2.60), pixel(x + .05, .90), cv::Scalar(240, 240, 240),
                          cv::FILLED);
        ParkingDetector outdoor_detector(cfg, g);
        auto outdoor_bay = acquire(outdoor_detector, outdoor);
        require(outdoor_bay.valid && outdoor_bay.left_clear,
                "white outer lane boundaries plus yellow divider form a bay");
        ParkingDetector reflection(cfg, g);
        require(!acquire(reflection, scene(0, true, true)).valid, "pure reflection is not yellow tape");
        // Image structure may establish the end identity without projecting its
        // out-of-domain pixels. The known bay length supplies an explicitly inferred end.
        GroundMap near_only(cv::Matx33d(.005, 0, -1.6, 0, -.01, 4.5, 0, 0, 1), {640, 480},
                            {{0, 220}, {639, 220}, {639, 479}, {0, 479}});
        ParkingDetector near_detector(cfg, near_only);
        auto inferred_end = acquire(near_detector, scene());
        require(inferred_end.valid && inferred_end.entry_visible && !inferred_end.end_visible,
                "partially mapped bay must not require all four corners in calibrated domain");
        require(std::abs(inferred_end.end_y - inferred_end.entry_y - cfg.parking_length) < 1e-8,
                "inferred end must preserve the configured inner length");
        GroundMap far_only(cv::Matx33d(.005, 0, -1.6, 0, -.01, 4.5, 0, 0, 1), {640, 480},
                           {{0, 0}, {639, 0}, {639, 310}, {0, 310}});
        ParkingDetector far_detector(cfg, far_only);
        auto inferred_entry = acquire(far_detector, scene());
        require(inferred_entry.valid && !inferred_entry.entry_visible && inferred_entry.end_visible,
                "measured end plus image identity must initialize without measured entrance");
        cv::Mat unknown = scene();
        cv::rectangle(unknown, pixel(-.56, 2.44), pixel(-.07, 1.06), cv::Scalar(255, 255, 255), cv::FILLED);
        ParkingDetector unknown_detector(cfg, g);
        auto unknown_bay = acquire(unknown_detector, unknown);
        require(!unknown_bay.left_clear, "overexposed empty-looking slot is not confirmed free");
        Lane lane;
        lane.valid = true;
        lane.both = true;
        for (int i = 1; i <= 5; ++i) {
            detector.predictMotion(.3, 0, 0);
            bay = detector.detect(scene(i * .3), lane);
        }
        require(bay.valid && bay.end_visible && !bay.entry_visible,
                "end identity persists after entrance leaves image");
        require(bay.entry_y < 0 && std::abs(bay.end_y - 1) < .2,
                "measured motion updates entry and end separately");
        ParkingDetector stationary(cfg, g);
        acquire(stationary, scene());
        cv::Mat no_entry = scene();
        cv::rectangle(no_entry, {0, 345}, {639, 365}, cv::Scalar(45, 105, 35), cv::FILLED);
        auto hidden = stationary.detect(no_entry, lane);
        require(hidden.entry_y > .7, "entry invisibility must not invent crossing");

        // Regression: confidence loss formerly left tracked_.valid latched forever,
        // so a clear bay 60 cm away could never pass old-position association.
        ParkingDetector recovery(cfg, g);
        require(acquire(recovery, scene()).valid, "recovery fixture initial acquisition");
        const auto old_generation = recovery.generation();
        cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(45, 105, 35));
        for (int i = 0; i < 20; ++i)
            bay = recovery.detect(blank, lane);
        require(!bay.valid, "unobserved bay must expire");
        for (int i = 1; i <= cfg.confirm_frames; ++i) {
            bay = recovery.detect(scene(.6), lane);
            require(bay.valid == (i == cfg.confirm_frames),
                    "expired bay must reacquire from consecutive current frames");
        }
        require(recovery.generation() > old_generation && std::abs(bay.entry_y - .4) < .15 &&
                    std::abs(bay.end_y - 1.9) < .15 && bay.current_geometry,
                "reacquisition must replace the stale position after 60 cm movement");

        // Missing optical flow is common on a stationary/glared floor. An entire
        // loss episode invalidates old geometry once, not every new image vote.
        ParkingDetector motion_gap(cfg, g);
        require(acquire(motion_gap, scene()).valid, "motion gap initial acquisition");
        auto before_gap = motion_gap.generation();
        for (int i = 1; i <= cfg.confirm_frames + 3; ++i) {
            motion_gap.invalidateMotion();
            bay = motion_gap.detect(scene(.6), lane);
            require(bay.valid == (i >= cfg.confirm_frames),
                    "repeated missing motion must allow visual relocalization");
        }
        require(motion_gap.generation() == before_gap + 1 && std::abs(bay.entry_y - .4) < .15,
                "one motion-loss episode must produce one fresh localization generation");
        require(motion_gap.predictMotion(0, 0, 0), "valid zero motion restores increment continuity");
        auto predicted = motion_gap.detect(blank, lane);
        require(predicted.valid && !predicted.current_geometry,
                "prediction-only bay must not claim current directly anchored geometry");
        motion_gap.invalidateMotion();
        require(!motion_gap.detect(scene(.6), lane).valid,
                "a new motion-loss episode must not reuse the previous generation");
        require(!motion_gap.predictMotion(std::numeric_limits<double>::quiet_NaN(), 0, 0),
                "nonfinite motion must be rejected");
        require(!motion_gap.predictMotion(.6, 0, 0), "out-of-range increment must be rejected");
        motion_gap.detect(scene(.6), lane);
        require(!motion_gap.detect(cv::Mat{}, lane).valid, "empty frame breaks candidate confirmation");
        for (int i = 1; i <= cfg.confirm_frames; ++i) {
            motion_gap.invalidateMotion();
            bay = motion_gap.detect(scene(.6), lane);
            require(bay.valid == (i == cfg.confirm_frames), "empty-frame recovery needs new image votes");
        }
        ParkingDetector recorded(cfg, imported);
        Bay recorded_bay;
        int fresh = 0;
        for (const char *path :
             {"tests/fixtures/parking_replay_001.png", "tests/fixtures/parking_replay_002.png",
              "tests/fixtures/parking_replay_003.png"}) {
            cv::Mat real = cv::imread(path);
            require(!real.empty(), "recorded parking fixture unavailable");
            recorded.invalidateMotion();
            recorded_bay = recorded.detect(real, Lane{});
            ++fresh;
            if (fresh < cfg.confirm_frames)
                require(!recorded_bay.valid, "real scene requires fresh repeated confirmation");
        }
        require(recorded_bay.valid,
                "actual left half-bay must initialize despite hidden opposite outer edge");
        require(recorded_bay.right_blocked && recorded_bay.left_clear && !recorded_bay.right_clear,
                "actual right blue board must select genuinely visible left floor");
        require(recorded_bay.entry_visible && !recorded_bay.end_visible,
                "out-of-domain end remains inferred, never falsely measured");
        // Optional full historical clip, kept outside the portable source package.
        if (argc == 2) {
            cv::VideoCapture replay(argv[1]);
            require(replay.isOpened(), "historical replay cannot open");
            ParkingDetector replay_detector(cfg, imported);
            cv::Mat replay_frame;
            int frames = 0, valid = 0, first_valid = 0;
            while (replay.read(replay_frame)) {
                ++frames;
                replay_detector.invalidateMotion();
                auto observed = replay_detector.detect(replay_frame, Lane{});
                if (observed.valid) {
                    ++valid;
                    if (!first_valid)
                        first_valid = frames;
                    require(observed.right_blocked && !observed.end_visible,
                            "historical bay identity or inferred end status changed");
                }
            }
            require(frames == 109 && valid == 107 && first_valid == 3,
                    "historical 109-frame clip must initialize at frame three and retain identity");
            std::cout << "Historical parking replay: " << valid << '/' << frames << " bay-valid frames\n";
        }
        std::cout << "parking mapping and scene tests passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
