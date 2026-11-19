#pragma once
#include "types.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <deque>
#include <optional>

namespace smartcar {
std::vector<unsigned char> actuatorPacket(int steering_us, int throttle_us);
class OutputPolicy {
    Config config_;
    Command requested_;
    std::deque<Command> pending_brakes_;
    std::optional<Command> active_brake_;
    std::uint64_t last_event_id_ = 0, completed_event_id_ = 0;
    double command_time_ = 0;
    double brake_started_ = 0;

  public:
    explicit OutputPolicy(Config c) : config_(c) {
        requested_.throttle_us = config_.neutral_us;
        requested_.steering_us = config_.steering_center_us;
    }
    // Called synchronously by the producer, not only when the 20 ms writer wakes.
    void submit(Command requested, double command_time);
    Command resolve(double now);
    Command resolve(Command requested, double command_time, double now);
    Command stop(double now); // explicit operator stop, unlike a visual neutral request
    std::uint64_t activeBrakeEvent() const { return active_brake_ ? active_brake_->brake_event_id : 0; }
    std::uint64_t completedBrakeEvent() const { return completed_event_id_; }
};
class Actuator {
  public:
    Actuator(const Config &, bool drive, const std::string &port);
    ~Actuator();
    void submit(const Command &command);
    // capture_stamp is the wall-clock time the source image was captured (seconds, same
    // clock as the actuator's own timer). Pass a negative value to opt out (e.g. replay).
    void submit(const Command &command, double capture_stamp);
    void check() const;
    void close();
    int lastOutputUs() const { return last_output_us_; }
    unsigned gapStopCount() const { return gap_stops_; }
    std::uint64_t activeBrakeEvent() const { return active_brake_event_; }
    std::uint64_t completedBrakeEvent() const { return completed_brake_event_; }

  private:
    void loop();
    void write(int steering, int throttle);
    Config config_;
    bool drive_ = false;
    int fd_ = -1;
    std::atomic<bool> running_{false}, failed_{false};
    std::thread worker_;
    std::atomic<int> last_output_us_{1500};
    std::atomic<unsigned> gap_stops_{0};
    std::atomic<std::uint64_t> active_brake_event_{0}, completed_brake_event_{0};
    std::mutex mutex_;
    OutputPolicy policy_;
    std::chrono::steady_clock::time_point updated_;
};
} // namespace smartcar
