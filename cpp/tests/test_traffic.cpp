#include "TrafficLight.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>

using namespace ImageProcess;
using A = TrafficLightAction;
using S = TrafficLightState;
namespace {
int checks = 0;
void check(bool ok, const char *message) {
    ++checks;
    if (!ok)
        throw std::runtime_error(message);
}
cv::Mat scene(bool red, bool green, bool vertical = false, int dx = 0) {
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(170, 170, 170));
    cv::Rect housing = vertical ? cv::Rect(250 + dx, 100, 36, 120) : cv::Rect(220 + dx, 160, 120, 36);
    cv::rectangle(frame, housing, cv::Scalar(25, 25, 25), cv::FILLED);
    const cv::Point r = vertical ? cv::Point(268 + dx, 120) : cv::Point(240 + dx, 178);
    const cv::Point g = vertical ? cv::Point(268 + dx, 200) : cv::Point(320 + dx, 178);
    if (red)
        cv::circle(frame, r, 10, cv::Scalar(0, 0, 255), cv::FILLED);
    if (green)
        cv::circle(frame, g, 10, cv::Scalar(0, 255, 0), cv::FILLED);
    return frame;
}
void redStop(TrafficLightDetector &d, bool vertical = false) {
    check(d.update(scene(true, false, vertical), 1, 1.0) == A::NONE, "one red frame must not trigger");
    check(d.update(scene(true, false, vertical), 2, 1.05) == A::RED_STOP, "second red must trigger");
    check(d.getState() == S::WAIT_GREEN, "red must latch hold");
}
} // namespace
int main() {
    try {
        {
            TrafficLightDetector d;
            redStop(d);
            d.update(scene(false, true), 3, 1.10);
            d.update(scene(false, true), 4, 1.15);
            check(d.update(scene(false, true), 5, 1.20) == A::GREEN_GO, "initial green confirmed");
            check(d.update(scene(true, false), 6, 1.25) == A::NONE, "single red cannot re-confirm new red");
            check(d.getState() == S::WAIT_GREEN, "single red withdraws green but preserves signal identity");
            check(d.update(scene(false, true), 7, 1.30) == A::NONE, "recovery green needs first fresh vote");
            check(d.update(scene(false, true), 8, 1.35) == A::NONE, "recovery green needs third fresh vote");
            check(d.update(scene(false, true), 9, 1.40) == A::GREEN_GO,
                  "same signal recovers after one-frame red reflection; no IDLE deadlock");
        }
        for (bool vertical : {false, true}) {
            TrafficLightDetector d;
            redStop(d, vertical);
            check(d.update(scene(false, true, vertical), 3, 1.10) == A::NONE, "first green cannot release");
            check(d.update(scene(false, true, vertical), 4, 1.15) == A::NONE, "second green cannot release");
            check(d.update(scene(false, true, vertical), 5, 1.20) == A::GREEN_GO,
                  "third same housing green releases");
            auto debug = d.getLastDebugInfo();
            check(debug.green_qualified && !debug.red_qualified,
                  "green release cannot simultaneously indicate red hold");
            check(d.update(scene(false, true, vertical), 6, 1.22) == A::NONE,
                  "confirmed green event is emitted once");
            check(d.getState() == S::GREEN_CONFIRMED,
                  "continuously observed associated green remains qualified");
            check(d.currentGreen(1.23), "current same-housing green authorizes control");
            check(d.update(scene(true, false, vertical), 7, 1.25) == A::NONE, "new red needs confirmation");
            check(d.getState() != S::GREEN_CONFIRMED, "first valid red immediately revokes green latch");
            check(d.update(scene(true, false, vertical), 8, 1.30) == A::RED_STOP,
                  "no blind cooldown after green");
        }
        for (bool wrong_housing : {false, true}) {
            TrafficLightDetector d;
            redStop(d);
            d.update(scene(false, true), 3, 1.10);
            d.update(scene(false, true), 4, 1.15);
            d.update(scene(false, true), 5, 1.20);
            check(d.currentGreen(1.21), "fresh confirmed green is usable");
            const auto next = wrong_housing ? scene(false, true, false, 180) : scene(false, false);
            d.update(next, 6, 1.25);
            check(!d.currentGreen(1.25), "dark or other-housing frame immediately revokes release");
            check(d.getState() == S::WAIT_GREEN, "missing current green retains recoverable red hold");
            check(d.update(scene(false, true), 7, 1.30) == A::NONE,
                  "green after visual absence must begin fresh confirmation");
            d.update(scene(false, true), 8, 1.35);
            check(!d.currentGreen(1.35), "two returning green frames do not reuse historical permission");
            check(d.update(scene(false, true), 9, 1.40) == A::GREEN_GO,
                  "new three-frame green confirmation restores permission");
        }
        {
            TrafficLightDetector d;
            redStop(d);
            d.update(scene(false, true), 3, 1.10);
            d.update(scene(false, true), 4, 1.15);
            d.update(scene(false, true), 5, 1.20);
            check(d.currentGreen(1.60), "bounded recent green can bridge a slower control consumer");
            check(!d.currentGreen(1.71), "latched state alone expires without fresh camera evidence");
            check(!d.currentGreen(1.19), "future-dated evidence cannot authorize an earlier control tick");
            d.update(scene(false, true), 5, 1.80);
            check(!d.currentGreen(1.80), "duplicate frame cannot refresh green evidence age");
            d.update(scene(false, true), 6, 1.85);
            check(!d.currentGreen(1.85), "fresh frame after long gap requires reconfirmation");
        }
        {
            TrafficLightDetector d;
            d.update(scene(true, false), 1, 1.0);
            check(d.update(scene(true, false), 1, 1.01) == A::NONE, "duplicate frame must not confirm red");
            check(d.update(scene(true, false), 2, 1.05) == A::RED_STOP, "fresh second frame must confirm");
            d.update(scene(false, true), 3, 1.10);
            check(d.update(scene(false, true), 3, 1.11) == A::NONE, "duplicate must not count towards green");
            check(d.update(scene(false, true), 4, 1.15) == A::NONE, "only two unique green frames");
            check(d.update(scene(false, true), 5, 1.20) == A::GREEN_GO, "third unique green frame releases");
        }
        {
            TrafficLightDetector d;
            d.update(scene(true, false), 1, 1.0);
            d.update(cv::Mat(), 2, 1.05);
            check(d.update(scene(true, false), 3, 1.10) == A::NONE, "empty frame breaks red streak");
            check(d.update(scene(true, false), 4, 1.15) == A::RED_STOP, "red can recover after empty");
            d.update(scene(false, true), 5, 1.20);
            d.update(scene(false, true), 6, 1.25);
            d.update(cv::Mat(), 7, 1.30);
            check(d.getState() == S::WAIT_GREEN, "missing frame must not clear red hold");
            check(d.update(scene(false, true), 8, 1.35) == A::NONE, "empty frame breaks green streak");
            check(d.update(scene(false, true), 9, 1.40) == A::NONE,
                  "second restarted green does not release");
            check(d.update(scene(false, true), 10, 1.45) == A::GREEN_GO, "third restarted green releases");
        }
        {
            TrafficLightDetector d;
            redStop(d);
            auto floor = scene(false, false);
            cv::rectangle(floor, {0, 260, 640, 220}, cv::Scalar(0, 180, 0), cv::FILLED);
            for (int i = 3; i <= 8; ++i)
                check(d.update(floor, i, 1.0 + i * .05) == A::NONE, "green floor cannot release");
            for (int i = 9; i <= 12; ++i)
                check(d.update(scene(false, true, false, 180), i, 1.0 + i * .05) == A::NONE,
                      "another green housing cannot release");
            for (int i = 13; i <= 16; ++i)
                check(d.update(scene(true, true), i, 1.0 + i * .05) == A::NONE,
                      "red has priority when both lit");
            check(d.getState() == S::WAIT_GREEN, "red-green conflict retains hold");
            // A green spot at the old RED lamp location must not pass association either.
            auto wrong = scene(false, false);
            cv::circle(wrong, {240, 178}, 10, cv::Scalar(0, 255, 0), cv::FILLED);
            for (int i = 17; i <= 20; ++i)
                check(d.update(wrong, i, 1.0 + i * .05) == A::NONE,
                      "same-position green artifact cannot release");
        }
        {
            TrafficLightDetector d;
            for (int i = 1; i <= 6; ++i)
                check(d.update(scene(false, true), i, i * .05) == A::NONE,
                      "green without earlier red never authorizes go");
            d.reset();
            d.update(scene(true, false), 1, 1.0);
            check(d.update(scene(true, false), 2, 2.0) == A::NONE, "large capture gap resets red streak");
            check(d.update(scene(true, false), 3, 2.05) == A::RED_STOP, "fresh red streak works after gap");
            d.update(scene(false, true), 4, 2.10);
            d.update(scene(false, true), 5, 2.15);
            check(d.update(scene(false, true), 6, 3.0) == A::NONE, "large capture gap resets green streak");
            for (int i = 7; i < 17; ++i)
                check(d.update(scene(false, false), i, 3.0 + i) == A::NONE, "no timed auto-release");
            check(d.getState() == S::WAIT_GREEN, "hold is not a fixed countdown");
        }
        for (int interrupted_by = 0; interrupted_by < 4; ++interrupted_by) {
            TrafficLightDetector d;
            redStop(d);
            d.update(scene(false, true), 3, 1.10);
            d.update(scene(false, true), 4, 1.15);
            check(d.update(scene(false, true), 5, 1.20) == A::GREEN_GO,
                  "prepare green before capture interruption");
            if (interrupted_by == 0)
                d.update(cv::Mat(), 5, 1.20); // stale camera repeats its last ID
            if (interrupted_by == 1)
                d.update(scene(false, true), 6, std::numeric_limits<double>::quiet_NaN());
            if (interrupted_by == 2)
                d.update(scene(false, true), 6, 1.10); // capture clock regressed
            if (interrupted_by == 3)
                d.update(scene(false, false), 6, 2.0); // long gap, no present green
            check(d.getState() == S::WAIT_GREEN, "capture interruption must withdraw unconsumed green latch");
            check(!d.getLastDebugInfo().green_qualified, "stale green must not authorize release");
            check(d.getLastDebugInfo().housing_found, "red housing identity survives interruption");
            const double restart = interrupted_by == 3 ? 2.05 : 1.25;
            check(d.update(scene(false, false), 7, restart) == A::NONE,
                  "blank recovered image cannot consume pre-outage green");
            check(d.getState() == S::WAIT_GREEN, "blank recovery remains stopped");
            check(d.update(scene(false, true), 8, restart + .05) == A::NONE,
                  "first post-outage green must requalify");
            check(d.update(scene(false, true), 9, restart + .10) == A::NONE,
                  "second post-outage green must requalify");
            check(d.update(scene(false, true), 10, restart + .15) == A::GREEN_GO,
                  "third same-housing post-outage green restores release");
        }
        {
            TrafficLightDetector d;
            d.update(scene(true, false), 1, 1.0);
            check(d.update(scene(true, false, false, 180), 2, 1.05) == A::NONE,
                  "different red bulb cannot confirm pending red");
            check(d.update(scene(true, false, false, 180), 3, 1.10) == A::RED_STOP,
                  "consistent relocated red can confirm");
            d.reset();
            check(d.update(scene(false, true), 1, 1.0) == A::NONE, "reset clears previous red identity");
            d.update(scene(true, false), 2, 1.05);
            d.update(scene(true, false), 3, std::numeric_limits<double>::quiet_NaN());
            check(d.update(scene(true, false), 4, 1.10) == A::NONE, "invalid timestamp breaks streak");
            auto config = d.getConfig();
            config.dual_roi_enable = 1;
            config.lower_roi_y_ratio_start = 0.05;
            config.lower_roi_y_ratio_end = 0.85;
            d.setConfig(config);
            d.reset();
            check(d.update(scene(true, false), 1, 1.0) == A::NONE, "overlapping ROIs do not duplicate votes");
            check(d.update(scene(true, false), 2, 1.05) == A::RED_STOP, "union ROI still sees red");
        }
        std::cout << checks << " traffic checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "traffic test failed: " << e.what() << '\n';
        return 1;
    }
}
