#include "smartcar/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>

using namespace smartcar;
namespace {
int checks = 0;
void check(bool ok, const char *message) {
    ++checks;
    if (!ok)
        throw std::runtime_error(message);
}
struct Run {
    Controller car;
    Observation o;
    std::uint64_t id = 0;
    explicit Run(Task t, Config config = {}) : car(config, t) {
        o.lane.valid = o.lane.both = true;
        o.lane.metric = true;
        o.lane.lateral_uncertainty_m = .03;
        o.lane.left_x = -.585;
        o.lane.right_x = .585;
        o.lane.confidence = 1;
    }
    Command at(double time) {
        o.time = time;
        o.frame_id = ++id;
        auto q = car.update(o);
        check((q.throttle_us == kZebraReverseUs && q.state == "zebra_brake" && q.brake_event_id) ||
                  q.throttle_us == 1400 || q.throttle_us == 1500 || q.throttle_us >= 1570,
              "forward output below motor movement threshold");
        check(q.steering_us >= 1350 && q.steering_us <= 1870, "steering outside measured endpoints");
        return q;
    }
};
Bay bay(double lo = -.28, double hi = .33, double heading = 0) {
    Bay b;
    b.valid = b.entry_visible = b.end_visible = true;
    b.left_clear = true;
    b.right_blocked = true;
    b.left_x = lo;
    b.divider_x = hi + .05;
    b.right_x = hi + .70;
    b.entry_y = -.665;
    b.end_y = .835;
    b.heading = heading;
    b.confidence = 1;
    return b;
}
} // namespace
int main() {
    try {
        {
            auto rejects = [](Config c) {
                try {
                    Controller controller(c);
                    return false;
                } catch (const std::invalid_argument &) {
                    return true;
                }
            };
            Config c;
            c.steering_gain = -1;
            check(rejects(c), "negative steering gain cannot reverse left/right control");
            c = Config{};
            c.max_steering_rate = 0;
            check(rejects(c), "zero steering slew rate cannot freeze turn output");
            c = Config{};
            c.clearance = -.01;
            check(rejects(c), "negative body clearance is invalid");
            c = Config{};
            c.body_width = 0;
            check(rejects(c), "zero body width cannot bypass boundary constraints");
            c = Config{};
            c.cone_loss_s = std::numeric_limits<double>::quiet_NaN();
            check(rejects(c), "NaN cannot disable prediction limit comparisons");
            c = Config{};
            c.lane_width = std::numeric_limits<double>::infinity();
            check(rejects(c), "infinite geometry is invalid");
            c = Config{};
            c.forward_min_us = 1590;
            c.forward_us = 1580;
            check(rejects(c), "forward command cannot be below configured motor minimum");
        }
        {
            Run r(Task::Race);
            r.o.board_observation_valid = true;
            for (int i = 0; i < 5; ++i)
                check(r.at(i * .05).throttle_us == 1500, "no board history must never auto-launch");
            r.o.board_visible = true;
            for (int i = 5; i < 8; ++i)
                r.at(i * .05);
            r.o.board_visible = false;
            r.at(.40);
            r.at(.45);
            check(r.car.stage() == Stage::WaitBoard, "one missing board observation is not a launch signal");
            r.o.board_observation_valid = false;
            r.at(.50);
            r.o.board_observation_valid = true;
            r.at(.55);
            r.at(.60);
            check(r.car.stage() == Stage::WaitBoard, "invalid board frame must interrupt removal streak");
            r.at(.65);
            check(r.car.stage() == Stage::ZebraApproach, "verified board removal starts race");
        }
        {
            Run r(Task::Race);
            r.o.lane.valid = false;
            r.o.board_observation_valid = r.o.board_visible = true;
            for (int i = 0; i < 3; ++i)
                r.at(i * .1);
            r.o.board_visible = false;
            r.at(.3);
            r.at(.4);
            auto removed = r.at(.5);
            check(removed.reason == "start board removed; waiting for visible lane",
                  "confirmed removal must be distinguished from lane unavailable");
            check(r.car.stage() == Stage::WaitBoard && removed.throttle_us == 1500,
                  "board removal alone cannot launch without visible steering guidance");
            r.o.board_visible = true;
            r.o.lane.valid = true;
            check(r.at(.6).throttle_us == 1500 && r.car.stage() == Stage::WaitBoard,
                  "board reappearance cancels pending removal even when lane recovers");
            r.o.board_visible = false;
            r.at(.7);
            r.at(.8);
            r.at(.9);
            check(r.car.stage() == Stage::ZebraApproach, "fresh removal and lane recovery allow race");
            check(r.at(1).throttle_us >= 1570, "race begins at motor-capable forward throttle");
        }
        {
            Run r(Task::Race);
            r.o.board_visible = r.o.board_observation_valid = true;
            r.at(0);
            r.at(.1);
            r.at(.2);
            r.o.board_visible = false;
            r.at(.3);
            r.at(.4);
            r.o.frame_ok = false;
            r.at(.5);
            r.o.frame_ok = true;
            r.at(.6);
            r.at(.7);
            check(r.car.stage() == Stage::WaitBoard,
                  "missing camera frame interrupts consecutive removal confirmation");
            r.at(.8);
            check(r.car.stage() == Stage::ZebraApproach,
                  "fresh removal can recover after a missing frame without losing board history");
        }
        {
            Run r(Task::Zebra);
            r.o.zebra = true;
            r.o.zebra_y = .80;
            check(r.at(0).throttle_us == 1400,
                  "zebra must start brake when measured distance reaches threshold");
            check(r.at(.34).throttle_us == 1400, "brake pulse should persist for its calibrated duration");
            check(r.at(.36).throttle_us == 1500, "brake pulse must return to neutral");
            r.o.stopped_confirmed = true;
            r.at(.40);
            check(!r.at(10.39).done, "must not finish before 10 full stopped seconds");
            check(r.at(10.41).done, "10 seconds of verified stationary hold completes zebra test");
        }
        {
            Run r(Task::Zebra);
            r.o.zebra = true;
            r.o.zebra_y = .8;
            r.at(0);
            r.at(.36);
            r.o.stopped_confirmed = true;
            r.at(.4);
            r.at(6);
            r.o.stopped_confirmed = false;
            r.o.motion_valid = true; // confirmed real motion, not just an unreadable frame
            r.o.forward_delta = .05;
            r.at(6.1);
            r.o.motion_valid = false;
            r.o.forward_delta = 0;
            r.o.stopped_confirmed = true;
            r.at(6.2);
            check(!r.at(10.5).done, "moving during hold invalidates earlier stopped time");
            check(r.at(16.21).done, "full renewed stationary hold can complete");
        }
        {
            Config c;
            c.zebra_brake_us = kZebraReverseUs;
            Run r(Task::Zebra, c);
            r.o.zebra = true;
            r.o.zebra_y = .70;
            r.o.brake_feedback_valid = true;
            auto first = r.at(0.);
            check(first.brake_event_id && first.throttle_us == 1300, "initial zebra reverse pulse");
            r.o.brake_completed_event_id = first.brake_event_id;
            r.at(.4);
            r.o.motion_valid = true;
            r.o.stopped_confirmed = false;
            r.o.forward_delta = .012;
            for (double time : {.61, .66})
                check(!r.at(time).brake_event_id, "a brief forward estimate must not re-brake");
            r.o.motion_valid = false;
            r.at(.71);
            r.o.motion_valid = true;
            for (double time : {.76, .86})
                check(!r.at(time).brake_event_id, "missing motion must restart roll confirmation");
            auto second = r.at(.97);
            check(second.throttle_us == 1300 && second.brake_event_id > first.brake_event_id,
                  "sustained forward roll after neutral needs a distinct brake transaction");
            check(r.at(1.5).brake_event_id == second.brake_event_id, "old ACK cannot complete re-braking");
            r.o.brake_completed_event_id = second.brake_event_id;
            r.at(1.55);
            for (int i = 0; i < 12; ++i)
                check(r.at(1.8+i*.10).throttle_us == 1500, "one supplementary pulse per crossing");
            r.o.forward_delta = 0;
            r.o.stopped_confirmed = true;
            r.at(3.1);
            check(!r.at(13.05).done && r.at(13.11).done, "ten stopped seconds after re-braking");
        }
        {
            Run r(Task::Traffic);
            r.o.red = true;
            check(r.at(0).throttle_us == 1400, "red must initiate brake");
            r.o.red = false;
            r.o.green = true; // main supplies only currently visible, qualified green evidence
            check(r.at(.1).throttle_us == 1400, "early green must not truncate brake pulse");
            check(r.at(.36).throttle_us == 1500, "green during brake still requires return to neutral");
            check(r.at(.40).done, "still-current qualified green can release after the brake stage");
            r.car.reset(Task::Traffic);
            r.o.red = true;
            r.o.green = false;
            r.at(1);
            r.at(1.36);
            r.o.green = true;
            check(!r.at(1.4).done, "red-green conflict cannot release");
            r.o.red = false;
            r.o.green = false;
            check(!r.at(30).done, "elapsed time alone cannot release traffic stop");
        }
        {
            Run r(Task::Obstacle);
            auto missing = r.at(0);
            check(missing.throttle_us == 1500 && missing.steering_us == 1610 && !missing.done,
                  "missing race-cone intent must never select a different obstacle planner");
            r.o.cone_drive = LegacyConeDrive{1570, 1510, 1, false, .1, "locked target"};
            check(r.at(.1).steering_us == 1510, "fresh race-cone intent resumes the same task");
            r.o.cone_drive.reset();
            auto lost = r.at(.2);
            check(lost.throttle_us == 1500 && lost.cones_passed == 1 && !lost.done,
                  "missing feedback retains confirmed progress without renewing motion");
            r.o.cone_drive = LegacyConeDrive{1500, 1610, 2, true, .3, "complete"};
            check(r.at(.3).done, "single-task completion requires the pinned controller's handoff");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            r.o.bay.entry_y = .25;
            r.o.bay.end_y = 1.75;
            check(r.at(0).throttle_us >= 1570, "parking approaches measured endpoint before brake threshold");
            r.o.bay = bay();
            r.o.bay.end_y = 1.28;
            r.o.bay.entry_y = -.22;
            check(r.at(.1).throttle_us == 1400, "parking endpoint includes braking-distance compensation");
            r.at(.46);
            r.o.bay = bay();
            r.o.stopped_confirmed = true;
            r.at(.5);
            check(!r.at(1.19).done, "parking settle is not instantaneous");
            check(r.at(1.21).done, "inside empty bay and stationary completes parking");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            r.o.bay.entry_y = -.02; // rear body / rear wheels still outside
            r.at(0);
            r.at(.36);
            r.o.stopped_confirmed = true;
            r.at(.4);
            check(!r.at(2).done, "front entering parking bay is not full vehicle entry");
            r.o.bay.valid = false;
            r.o.motion_valid = false;
            check(!r.at(3).done, "invisible entrance is not proof of entry");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay(-.1, .4, .2);
            r.at(0);
            r.at(.36);
            r.o.stopped_confirmed = true;
            r.at(.4);
            check(r.at(1.2).done, "tilted bay must use signed vehicle-to-boundary transform correctly");
            Run outside(Task::Parking);
            outside.o.bay = bay(-.4, .1, .2);
            outside.at(0);
            outside.at(.36);
            outside.o.stopped_confirmed = true;
            outside.at(.4);
            check(!outside.at(2).done, "tilted rear corner across boundary cannot be declared parked");
        }
        {
            // #3: an unreadable frame must neither reset nor credit the 10 s stationary hold.
            Run r(Task::Zebra);
            r.o.zebra = true;
            r.o.zebra_y = .8;
            r.at(0);
            r.at(.36);
            r.o.stopped_confirmed = true;
            r.at(.4);
            r.at(8.0);
            r.o.stopped_confirmed = false; // unknown frame: neither stopped nor measured-moving
            r.at(8.1);
            r.o.stopped_confirmed = true;
            r.at(8.2);
            check(!r.at(10.5).done, "unknown frames must not credit nor reset the hold window");
            check(r.at(10.61).done, "hold resumes after an unreadable frame without restarting");
        }
        {
            // #6: 10 cm remaining gap is an overshoot, not a completed ~50 cm park.
            Run over(Task::Parking);
            over.o.bay = bay();
            over.o.bay.entry_y = -.665;
            over.o.bay.end_y = .435; // end_gap = .10
            over.at(0);
            over.at(.36);
            over.o.stopped_confirmed = true;
            over.at(.4);
            check(!over.at(2).done, "10 cm remaining gap is not a completed 50 cm park");
        }
        {
            // A short correction cannot reuse a 45 cm steady-travel braking model.
            Run shortOf(Task::Parking);
            shortOf.o.bay = bay();
            shortOf.o.bay.entry_y = -.665;
            shortOf.o.bay.end_y = 1.035; // end_gap = .70
            shortOf.at(0);
            shortOf.at(.36);
            shortOf.o.stopped_confirmed = true;
            auto q = shortOf.at(.4);
            check(q.throttle_us == 1500 && !q.done,
                  "70 cm remaining gap requires a calibrated short correction instead of more forward drive");
        }
        {
            Run r(Task::Observe);
            r.o.red = true;
            r.o.zebra = true;
            r.o.zebra_y = .5;
            r.o.bay = bay();
            check(r.at(0).throttle_us == 1500, "observe task must never move despite detected elements");
            Run lane(Task::Zebra);
            lane.at(0);
            lane.o.lane.valid = false;
            check(lane.at(.5).throttle_us >= 1570, "brief missing line can use recent geometry");
            check(lane.at(1.1).throttle_us == 1500, "expired line memory must not invent geometry");
            lane.o.frame_ok = false;
            check(lane.at(1.2).throttle_us == 1500, "missing camera does not keep stale forward command");
        }
        {
            // A timeout is a diagnosis, not evidence that the traffic rule was completed.
            Run r(Task::Traffic);
            check(!r.at(0).done, "traffic approach must not finish immediately");
            for (int i = 1; i <= 520; ++i)
                r.at(i * 0.05); // 26 s > default traffic_max_approach_s (25 s)
            check(r.car.stage() != Stage::Finished,
                  "no-light traffic timeout must not be recorded as success");
        }
        {
            Run r(Task::Race);
            r.o.board_observation_valid = true;
            r.o.board_visible = true;
            r.at(0);
            r.at(.1);
            r.at(.2);
            r.o.board_visible = false;
            r.at(.3);
            r.at(.4);
            r.at(.5);
            r.o.zebra = true;
            r.o.zebra_y = .8;
            r.at(.6);
            r.at(.96);
            r.o.stopped_confirmed = true;
            r.at(1);
            r.at(11.01);
            check(r.car.stage() == Stage::TrafficApproach, "race enters traffic after full zebra hold");
            auto waiting = r.at(37);
            check(r.car.stage() == Stage::TrafficApproach && waiting.throttle_us == 1500,
                  "full race cannot skip traffic to obstacles after 25 seconds without evidence");
            r.o.red = true;
            check(r.at(37.1).throttle_us == 1400 && r.car.stage() == Stage::TrafficBrake,
                  "later red recovers timed-out approach without resetting the whole race");
        }
        for (Task task : {Task::Zebra, Task::Traffic, Task::Parking}) {
            Run r(task);
            r.o.brake_feedback_valid = true;
            r.o.zebra = true;
            r.o.zebra_y = .8;
            r.o.red = true;
            r.o.bay = bay();
            const auto first = r.at(0);
            const Stage brake = r.car.stage();
            check(first.brake_event_id != 0, "live brake starts a transaction");
            r.o.red = false;
            r.o.green = true;
            r.o.brake_completed_event_id = first.brake_event_id + 1;
            auto delayed = r.at(1);
            check(r.car.stage() == brake && !delayed.done &&
                      delayed.brake_event_id == first.brake_event_id,
                  "elapsed pulse time and an unrelated ACK cannot finish a live brake");
            r.o.brake_feedback_valid = false;
            check(!r.at(1.1).done && r.car.stage() == brake,
                  "lost feedback cannot silently fall back to an offline elapsed timer");
            r.o.brake_feedback_valid = true;
            r.o.brake_completed_event_id = first.brake_event_id;
            r.o.green = false;
            const auto completed = r.at(1.2);
            check(r.car.stage() != brake && completed.throttle_us == 1500 && !completed.done,
                  "matching actuator neutral-write ACK enters hold without forward output");
            if (task == Task::Traffic)
                check(!r.at(1.3).done, "green lost while braking cannot release on old evidence");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            const auto first = r.at(0);
            r.at(.36);
            r.o.bay.end_y = 1.435; // 1.10 m current front gap; enough correction braking room
            check(r.at(.4).throttle_us >= 1570, "110 cm gap permits correction with calibrated braking room");
            r.o.bay.end_y = 1.275; // 94 cm front gap crosses the 50 + 45 cm trigger
            const auto correction = r.at(.5);
            check(correction.throttle_us == 1400 && correction.brake_event_id > first.brake_event_id,
                  "correction crossing 95 cm creates a new brake before reaching the target");
            const auto repeat = r.at(.6);
            check(repeat.brake_event_id == correction.brake_event_id,
                  "correction brake does not create another event on every frame");
            r.at(.86);
            r.o.bay.end_y = .885; // current post-braking endpoint measurement
            r.o.stopped_confirmed = true;
            r.at(.9);
            check(!r.at(1.59).done && r.at(1.61).done,
                  "correction completes only after new braking and renewed stationary hold");
        }
        for (bool invalid_bay : {false, true}) {
            Run r(Task::Parking);
            r.o.bay = bay();
            r.at(0);
            r.at(.36);
            r.o.stopped_confirmed = true;
            if (invalid_bay)
                r.o.bay.valid = false;
            else {
                r.o.bay.end_visible = false;
                r.o.bay.current_geometry = true;
            }
            r.at(.4);
            check(!r.at(2).done, "cached or merely current bay geometry cannot replace a visible endpoint");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            auto first = r.at(0);
            r.at(.36);
            r.o.bay.end_y = 1.435;
            r.at(.4);
            r.o.bay.valid = false;
            auto stopped = r.at(.5);
            check(stopped.throttle_us == 1400 && stopped.brake_event_id > first.brake_event_id,
                  "losing current endpoint during final correction requests its own brake pulse");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            r.o.bay.end_y = 1.135; // 80 cm front gap
            r.at(0);
            r.at(.36);
            const auto q = r.at(.4);
            check(q.throttle_us == 1500 && !q.done && q.reason.find("shorter than calibrated") != std::string::npos,
                  "80 cm gap never starts a correction with a 45 cm stopping-distance assumption");
        }
        {
            Run r(Task::Parking);
            r.o.bay = bay();
            const auto initial = r.at(0);
            r.at(.36);
            r.o.bay.end_y = 1.435;
            r.at(.4);
            r.o.frame_ok = false;
            const auto stopped = r.at(.5);
            check(stopped.throttle_us == 1400 && stopped.brake_event_id > initial.brake_event_id,
                  "whole-frame loss during correction still starts the new brake transaction");
            r.at(.55);
            r.o.frame_ok = true;
            const auto recovered = r.at(.6);
            check(recovered.brake_event_id == stopped.brake_event_id,
                  "repeated frame loss and recovery do not create repeated correction brake events");
        }
        {
            // 制动事件编号：新制动事件必须携带非零、严格递增的事件号；
            // 同一脉冲内的重复帧复用同一事件号（执行器据此保证“只执行一次、重复帧不延长”）。
            Run r(Task::Zebra);
            r.o.zebra = true;
            r.o.zebra_y = .80;
            auto first = r.at(0);
            check(first.throttle_us == 1400, "zebra brake frame is a real brake command");
            check(first.brake_event_id != 0, "brake command must carry a nonzero event id");
            auto repeat = r.at(.10);
            check(repeat.throttle_us == 1400 && repeat.brake_event_id == first.brake_event_id,
                  "repeated brake frames reuse the same event id");
            auto after = r.at(.40);
            check(after.brake_event_id == 0, "non-brake frame clears the event id");
        }
        {
            // 斑马线在停车线前短暂丢失：凭最后一次确认位置加前进里程继续逼近并制动。
            Run r(Task::Zebra);
            r.o.motion_valid = true;
            r.o.zebra = true;
            r.o.zebra_y = 1.05;
            r.at(0);
            r.o.zebra = false;
            r.o.forward_delta = .04;
            bool braked = false;
            for (int i = 1; i <= 40 && !braked; ++i)
                braked = r.at(i * .05).throttle_us == 1400;
            check(braked, "latched crosswalk plus odometry must still trigger the stop");
        }
        {
            Run r(Task::Zebra);
            r.o.zebra_detected = true;
            r.o.zebra_range_status = "unavailable";
            r.o.zebra_distance_reason = "outside current distance calibration";
            auto waiting = r.at(0);
            check(waiting.throttle_us == 1500 && waiting.reason.find("distance mapping") != std::string::npos,
                  "confirmed image without a distance is not silently treated as an absent crosswalk");
            r.o.zebra_detected = false;
            check(r.at(.1).throttle_us == 1500, "detector dropout cannot forget a pending unmapped crosswalk");
            r.o.zebra_detected = true;
            r.o.zebra_range_status = "beyond_far";
            check(r.at(.2).throttle_us >= 1570,
                  "known beyond-far crosswalk can approach the dedicated calibration range");
            r.o.zebra = true;
            r.o.zebra_y = .85;
            r.o.zebra_range_status = "in_range";
            check(r.at(.3).throttle_us == 1400, "new measured crosswalk range releases mapping wait into brake");
        }
        for (const char *status : {"too_near", "clipped"}) {
            Run r(Task::Zebra);
            r.o.zebra_detected = true;
            r.o.zebra_range_status = status;
            const auto q = r.at(0);
            check(q.throttle_us == 1400 && q.reason.find("without a verified") != std::string::npos,
                  "confirmed near-limit crosswalk brakes without claiming verified metric stop distance");
        }
        {
            Run r(Task::Zebra);
            r.o.zebra = true;
            r.o.zebra_y = 1.1;
            r.at(0);
            r.o.zebra = false;
            r.o.motion_valid = false;
            check(r.at(.1).throttle_us == 1500, "lost motion invalidates remembered crosswalk range");
            r.o.motion_valid = true;
            r.o.forward_delta = .4;
            check(r.at(.2).throttle_us == 1500 && r.car.stage() == Stage::ZebraApproach,
                  "restored motion cannot reconstruct the unknown interval or trigger on stale range");
            r.o.zebra = true;
            r.o.zebra_y = .85;
            check(r.at(.3).throttle_us == 1400, "current measured range rebuilds crosswalk tracking");
        }
        {
            // 车位锁定后长期无观测（即使有微弱运动估计）不得无限前进：
            // 超出记忆时限应停下等待重新定位，而不是盲目前冲。
            Run r(Task::Parking);
            r.o.bay = bay();
            r.o.bay.end_y = 1.75; // 尚未到达终点，先锁定并向前接近
            r.o.bay.entry_y = .25;
            check(r.at(0).throttle_us >= 1570, "parking approaches a distant measured endpoint");
            r.o.bay = Bay{}; // 车位观测丢失（valid=false）
            r.o.motion_valid = true;
            r.o.forward_delta = .0001; // 微弱运动，几乎不前进
            check(r.at(1.0).throttle_us >= 1570,
                  "brief bay loss with motion may still creep toward the target");
            check(r.at(9.0).throttle_us == 1500,
                  "bay lost beyond memory limit must stop and wait for relocalization");
        }
        {
            Run r(Task::Race);
            r.o.board_visible = r.o.board_observation_valid = true;
            for (int i = 0; i < 3; ++i) r.at(.1 * i);
            r.o.board_visible = false;
            for (int i = 3; i < 6; ++i) r.at(.1 * i);
            r.o.zebra_front_trigger = .4;
            r.o.zebra = r.o.zebra_detected = true;
            r.o.zebra_y = .335 + .50;
            check(r.at(.6).throttle_us >= 1570, "legacy crosswalk measures from the vehicle front");
            r.o.zebra_y = .335 + .40;
            check(r.at(.7).throttle_us == 1400, "legacy front gap reaches calibrated trigger");
            r.at(1.1);
            r.o.stopped_confirmed = true;
            r.at(1.2);
            r.at(11.3);
            r.o.green = true;
            r.at(11.4);
            check(r.car.stage() == Stage::Obstacles, "qualified light hands off to old cone navigation");
            r.o.cone_drive = LegacyConeDrive{1570, 1530, 1, false, 11.5, "tracking second cone"};
            check(r.at(11.5).steering_us == 1530, "legacy cone correction reaches common actuator command");
            check(r.at(12.1).throttle_us == 1500, "expired cone steering cannot renew motion");
            r.o.cone_drive = LegacyConeDrive{1500, 1610, 2, false, 12.2, "recentering"};
            r.at(12.2);
            check(r.car.stage() == Stage::Obstacles, "two passes still require recentering");
            r.o.cone_drive = LegacyConeDrive{1570, 1610, 2, true, 12.3, "complete"};
            check(r.at(12.3).throttle_us == 1570, "handoff follows the current clear lane into parking");
            check(r.car.stage() == Stage::ParkingApproach, "second pass and recenter hand off to parking");
            r.o.bay = bay();
            r.at(12.4);
            r.at(12.8);
            r.at(12.9);
            check(r.at(15.).done, "new parking completes after endpoint and stationary evidence");
        }
        std::cout << checks << " controller checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "controller test failed: " << e.what() << '\n';
        return 1;
    }
}
