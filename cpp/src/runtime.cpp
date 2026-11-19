#include "smartcar/runtime.hpp"
#include <stdexcept>
#include <algorithm>
#include <cerrno>
#include <cmath>
#ifdef __linux__
#include <fcntl.h>
#include <sys/file.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {
// A command whose source image was captured more than this long before the actuator
// received it is treated as stale: forward is degraded to neutral instead of driving on
// an old picture. Must exceed the measured capture->decision latency of the real vehicle.
constexpr double kCaptureFreshnessS = 1.0;
} // namespace

namespace smartcar {
void OutputPolicy::submit(Command q, double stamp) {
    if (!std::isfinite(stamp))
        throw std::invalid_argument("invalid command timestamp");
    if (q.throttle_us == config_.brake_us) {
        if (q.brake_event_id == 0)
            throw std::invalid_argument("brake request requires a nonzero event ID");
        if (q.brake_event_id > last_event_id_) {
            pending_brakes_.push_back(q);
            last_event_id_ = q.brake_event_id;
        }
    }
    requested_ = std::move(q);
    command_time_ = stamp;
}

Command OutputPolicy::resolve(double now) {
    if (!std::isfinite(now))
        throw std::invalid_argument("invalid output timestamp");
    Command q = requested_;
    // A completed/cancelled event can never be re-armed by a repeated request.
    if (q.throttle_us == config_.brake_us)
        q.throttle_us = config_.neutral_us;
    if (active_brake_ && now - brake_started_ >= config_.brake_s) {
        completed_event_id_ = active_brake_->brake_event_id;
        active_brake_.reset();
        // A neutral write terminates this transaction before acknowledging it.
        q.throttle_us = config_.neutral_us;
        q.steering_us = config_.steering_center_us;
        q.brake_event_id = 0;
        return q;
    }
    if (!active_brake_ && !pending_brakes_.empty()) {
        active_brake_ = std::move(pending_brakes_.front());
        pending_brakes_.pop_front();
        brake_started_ = now;
    }
    // Vision loss, producer expiry and replacement neutral commands cannot cut
    // an accepted pulse short. Explicit stop()/Actuator::close() can cancel it.
    if (active_brake_)
        return *active_brake_;
    if (now - command_time_ > 1.0) {
        q.throttle_us = config_.neutral_us;
        q.steering_us = config_.steering_center_us;
    }
    q.brake_event_id = 0;
    return q;
}

Command OutputPolicy::resolve(Command q, double stamp, double now) {
    submit(std::move(q), stamp);
    return resolve(now);
}

Command OutputPolicy::stop(double now) {
    pending_brakes_.clear();
    active_brake_.reset();
    requested_ = Command{};
    requested_.throttle_us = config_.neutral_us;
    requested_.steering_us = config_.steering_center_us;
    command_time_ = now;
    return requested_;
}
std::vector<unsigned char> actuatorPacket(int steering, int throttle) {
    if (steering < 1350 || steering > 1870 ||
        (throttle != 1400 && throttle != 1500 && (throttle < 1570 || throttle > 1600)))
        throw std::invalid_argument("actuator command outside configured electrical envelope");
    // RasAdapter5A function 4, set PWM, 20 ms, two servos: S3 steering / S4 ESC.
    std::vector<unsigned char> p{0xaa,
                                 0x55,
                                 4,
                                 10,
                                 1,
                                 20,
                                 0,
                                 2,
                                 3,
                                 static_cast<unsigned char>(steering & 255),
                                 static_cast<unsigned char>(steering >> 8),
                                 4,
                                 static_cast<unsigned char>(throttle & 255),
                                 static_cast<unsigned char>(throttle >> 8)};
    unsigned char crc = 0;
    for (std::size_t i = 2; i < p.size(); ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
    }
    p.push_back(crc);
    return p;
}
Actuator::Actuator(const Config &c, bool drive, const std::string &port)
    : config_(c), drive_(drive), policy_(c) {
#ifndef __linux__
    (void)port;
#endif
    updated_ = std::chrono::steady_clock::now();
    if (drive_) {
#ifdef __linux__
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("cannot open actuator UART");
        if (flock(fd_, LOCK_EX | LOCK_NB) < 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("UART owned by another controller; handoff must release it first");
        }
        termios t{};
        if (tcgetattr(fd_, &t) < 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("UART configuration read failed");
        }
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~CSTOPB;
        t.c_cflag &= ~CRTSCTS;
        cfsetispeed(&t, B1000000);
        cfsetospeed(&t, B1000000);
        if (tcsetattr(fd_, TCSANOW, &t) < 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("UART 1000000 baud setup failed");
        }
        try {
            write(c.steering_center_us, c.neutral_us);
        } catch (...) {
            ::close(fd_);
            fd_ = -1;
            throw;
        }
#else
        throw std::runtime_error("real actuator mode is Linux/Raspberry Pi only");
#endif
    }
    running_ = true;
    worker_ = std::thread(&Actuator::loop, this);
}
Actuator::~Actuator() {
    close();
}
void Actuator::submit(const Command &q) {
    submit(q, -1.0);
}
void Actuator::submit(const Command &q, double captureStamp) {
    check();
    actuatorPacket(q.steering_us, q.throttle_us);
    Command cmd = q;
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = std::chrono::steady_clock::now();
    const double received = std::chrono::duration<double>(updated_.time_since_epoch()).count();
    // A picture captured long before it reached the actuator must not re-command forward.
    // Neutral and brake are always allowed (fail-safe); only stale forward is degraded.
    if (captureStamp >= 0.0 && cmd.throttle_us > config_.neutral_us &&
        received - captureStamp > kCaptureFreshnessS) {
        cmd.throttle_us = config_.neutral_us;
        cmd.steering_us = config_.steering_center_us;
    }
    policy_.submit(cmd, received);
}
void Actuator::check() const {
    if (failed_)
        throw std::runtime_error("actuator transport failed");
}
void Actuator::write(int s, int t) {
    if (!drive_)
        return;
#ifdef __linux__
    auto p = actuatorPacket(s, t);
    std::size_t written = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (written < p.size()) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("UART write incomplete");
        pollfd f{fd_, POLLOUT, 0};
        if (poll(&f, 1, 20) <= 0)
            continue;
        auto n = ::write(fd_, p.data() + written, p.size() - written);
        if (n > 0)
            written += static_cast<std::size_t>(n);
        else if (n < 0 && errno != EAGAIN && errno != EINTR)
            throw std::runtime_error("UART write failed");
    }
#else
    (void)s;
    (void)t;
#endif
}
void Actuator::loop() {
    bool was_expired = false;
    while (running_) {
        Command q;
        std::chrono::steady_clock::time_point stamp;
        std::uint64_t active_event, completed_event;
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stamp = updated_;
            q = policy_.resolve(std::chrono::duration<double>(now.time_since_epoch()).count());
            active_event = policy_.activeBrakeEvent();
            completed_event = policy_.completedBrakeEvent();
        }
        bool expired = std::chrono::duration<double>(now - stamp).count() > 1.0;
        if (expired && !was_expired)
            ++gap_stops_;
        was_expired = expired;
        // This is local software failure containment, not a browser heartbeat or a task timeout.
        if (!running_)
            break;
        try {
            write(q.steering_us, q.throttle_us);
            last_output_us_ = q.throttle_us;
            active_brake_event_ = active_event;
            completed_brake_event_ = completed_event; // only after the end-of-pulse neutral write succeeds
        } catch (...) {
            failed_ = true;
            running_ = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    try {
        write(config_.steering_center_us, config_.neutral_us);
        last_output_us_ = config_.neutral_us;
        active_brake_event_ = 0;
    } catch (...) {
        failed_ = true;
    }
}
void Actuator::close() {
    running_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        policy_.stop(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    if (worker_.joinable())
        worker_.join();
#ifdef __linux__
    if (fd_ >= 0) {
        try {
            write(config_.steering_center_us, config_.neutral_us);
            last_output_us_ = config_.neutral_us;
        } catch (...) {
            failed_ = true;
        }
        tcdrain(fd_);
        ::close(fd_);
        fd_ = -1;
    }
#endif
}
} // namespace smartcar
