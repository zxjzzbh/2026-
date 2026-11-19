#include "smartcar/controller.hpp"
#include "smartcar/ground.hpp"
#include "smartcar/perception.hpp"
#include "smartcar/parking.hpp"
#include "smartcar/runtime.hpp"
#include "TrafficLight.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <future>
#include <iomanip>
#include <cerrno>
#include <type_traits>
#include <condition_variable>
#include <chrono>
#include <thread>
#ifdef __linux__
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

namespace fs = std::filesystem;
using namespace smartcar;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signalStop(int) {
    interrupted = 1;
}
double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string escape(const std::string &s) {
    std::string r;
    for (char c : s) {
        if (c == '"' || c == '\\')
            r += '\\';
        if (c == '\n') {
            r += "\\n";
            continue;
        }
        r += c;
    }
    return r;
}
struct Settings {
    Config control;
    std::string secondary = "/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0";
    std::string primary = "/dev/v4l/by-id/usb-YT-FAY-220613_USB2.0_Camera-video-index0";
    std::string port = "/dev/ttyAMA0", speech = "我是山东大学启航队的无人车，请为我加油！";
    std::string ground = "calibration/ground_parking.json", road = "calibration/ground_road.json";
    std::string zebra_profile = "calibration/crosswalk-distance-20261024.json";
};
Settings load(const fs::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("cannot read config");
    std::string text((std::istreambuf_iterator<char>(f)), {});
    cv::FileStorage file(text,
                         cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    if (!file.isOpened())
        throw std::runtime_error("invalid config");
    Settings s;
    auto number = [&](const char *k, auto &v) {
        if (!file[k].empty()) {
            double a = (double)file[k];
            if (!std::isfinite(a))
                throw std::runtime_error("nonfinite setting");
            v = static_cast<std::decay_t<decltype(v)>>(a);
        }
    };
    auto string = [&](const char *k, std::string &v) {
        if (!file[k].empty())
            file[k] >> v;
    };
    number("lane_width_m", s.control.lane_width);
    number("parking_length_m", s.control.parking_length);
    number("body_width_m", s.control.body_width);
    number("wheelbase_m", s.control.wheelbase);
    number("front_extent_m", s.control.front_extent);
    number("rear_extent_m", s.control.rear_extent);
    number("clearance_m", s.control.clearance);
    number("braking_distance_m", s.control.braking_distance);
    number("brake_s", s.control.brake_s);
    number("zebra_hold_s", s.control.zebra_hold_s);
    number("parking_stop_gap_m", s.control.parking_stop_gap);
    number("forward_us", s.control.forward_us);
    number("steering_gain", s.control.steering_gain);
    number("obstacle_gain", s.control.obstacle_gain);
    number("traffic_max_approach_s", s.control.traffic_max_approach_s);
    string("secondary_camera", s.secondary);
    string("primary_camera", s.primary);
    string("uart", s.port);
    string("announcement_text", s.speech);
    string("parking_ground", s.ground);
    string("road_ground", s.road);
    string("zebra_distance_profile", s.zebra_profile);
    s.ground = (p.parent_path() / fs::u8path(s.ground)).u8string();
    s.road = (p.parent_path() / fs::u8path(s.road)).u8string();
    if (!s.zebra_profile.empty())
        s.zebra_profile = (p.parent_path() / fs::u8path(s.zebra_profile)).u8string();
    return s;
}
Task taskFrom(const std::string &n) {
    if (n == "race")
        return Task::Race;
    if (n == "zebra")
        return Task::Zebra;
    if (n == "traffic")
        return Task::Traffic;
    if (n == "obstacle")
        return Task::Obstacle;
    if (n == "parking")
        return Task::Parking;
    if (n == "observe")
        return Task::Observe;
    throw std::runtime_error("unknown task");
}
void speak(const std::string &text) {
#ifdef __linux__
    // 斑马线停车期间必须播报规则要求的话语。先尝试 espeak-ng，失败再降级到经典 espeak，
    // 避免因缺少单一合成器导致“未播报”而被加罚。
    auto play = [&](const char *program, bool mandarin) {
        pid_t pid;
        std::vector<char *> args;
        args.push_back(const_cast<char *>(program));
        if (mandarin) {
            args.push_back(const_cast<char *>("-v"));
            args.push_back(const_cast<char *>("cmn"));
        }
        args.push_back(const_cast<char *>(text.c_str()));
        args.push_back(nullptr);
        if (posix_spawnp(&pid, program, nullptr, nullptr, args.data(), environ) != 0)
            return false;
        int status = 0;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
        return status == 0;
    };
    if (play("espeak-ng", true) || play("espeak", true) || play("espeak", false))
        return;
    std::cerr << "announcement unavailable: install espeak-ng (sudo apt install espeak-ng)\n";
#else
    std::cout << "ANNOUNCEMENT: " << text << '\n';
#endif
}
class Recorder {
    std::mutex mutex_;
    cv::Mat latest_;
    std::thread worker_;
    std::atomic<bool> running_{true};
    cv::VideoWriter video_;
    std::uint64_t latest_id_ = 0;
    double latest_time_ = 0;
    std::ofstream frames_;
    fs::path preview_;

  public:
    explicit Recorder(const std::string &filename) {
        preview_ = fs::u8path(filename).parent_path() /
                   (fs::u8path(filename).stem().u8string() + ".jpg");
        video_.open(filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, cv::Size(480, 360));
        if (!video_.isOpened())
            throw std::runtime_error("cannot create recording");
        frames_.open(fs::u8path(filename + ".frames.jsonl"));
        if (!frames_)
            throw std::runtime_error("cannot create recording frame index");
        worker_ = std::thread([this]() {
            std::uint64_t index = 0;
            auto previewTime = std::chrono::steady_clock::time_point::min();
            while (running_) {
                cv::Mat frame;
                std::uint64_t id;
                double time;
                {
                    std::lock_guard<std::mutex> l(mutex_);
                    frame = latest_;
                    latest_.release();
                    id = latest_id_;
                    time = latest_time_;
                }
                if (!frame.empty())
                    try {
                        video_.write(frame);
                        frames_ << std::setprecision(14) << "{\"video_index\":" << index++
                                << ",\"source_frame\":" << id << ",\"time\":" << time << "}\n";
                        // Web observers read this snapshot; they never own a capture device
                        // during autonomy. Encoding stays off the control thread.
                        auto now = std::chrono::steady_clock::now();
                        if (previewTime == std::chrono::steady_clock::time_point::min() ||
                            now - previewTime >= std::chrono::milliseconds(250)) {
                            try {
                                auto temporary = preview_.parent_path() /
                                                 (preview_.stem().u8string() + ".tmp.jpg");
                                if (cv::imwrite(temporary.u8string(), frame,
                                                {cv::IMWRITE_JPEG_QUALITY, 78})) {
#ifdef _WIN32
                                    std::error_code ignored;
                                    fs::remove(preview_, ignored);
#endif
                                    fs::rename(temporary, preview_);
                                }
                            } catch (const std::exception &) {
                                // A failed preview cannot interrupt vehicle control or recording.
                            }
                            previewTime = now;
                        }
                    } catch (...) {
                        std::cerr << "recording failed\n";
                        break;
                    }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    void frame(const cv::Mat &m, std::uint64_t id, double time) {
        std::lock_guard<std::mutex> l(mutex_);
        latest_ = m.clone();
        latest_id_ = id;
        latest_time_ = time;
    }
    ~Recorder() {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
        video_.release();
    }
};
// Primary camera capture cannot block steering/parking. Latest frame IDs are kept
// separate from secondary IDs so reused images never count as fresh light votes.
class LatestCamera {
    struct Capture {
        cv::VideoCapture camera;
        std::atomic<bool> running{true};
        std::mutex mutex;
        std::condition_variable finished;
        cv::Mat frame;
        std::uint64_t id = 0;
        double timestamp = 0;
        bool done = false;
    };
    std::shared_ptr<Capture> capture_ = std::make_shared<Capture>();
    std::thread worker_;

  public:
    explicit LatestCamera(const std::string &source) {
        if (!capture_->camera.open(source))
            throw std::runtime_error("cannot open primary camera");
        capture_->camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture_->camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture_->camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
        capture_->camera.set(cv::CAP_PROP_FPS, 30);
        worker_ = std::thread([capture = capture_]() {
            try {
                while (capture->running) {
                    cv::Mat next;
                    if (!capture->camera.read(next) || next.empty())
                        break;
                    std::lock_guard<std::mutex> lock(capture->mutex);
                    capture->frame = next;
                    capture->timestamp = monotonic();
                    ++capture->id;
                }
            } catch (...) {
                capture->running = false;
            }
            capture->camera.release();
            {
                std::lock_guard<std::mutex> lock(capture->mutex);
                capture->done = true;
            }
            capture->finished.notify_all();
        });
    }
    bool latest(cv::Mat &frame, std::uint64_t &id, double &stamp) {
        std::lock_guard<std::mutex> lock(capture_->mutex);
        if (capture_->frame.empty())
            return false;
        frame = capture_->frame.clone();
        id = capture_->id;
        stamp = capture_->timestamp;
        return true;
    }
    ~LatestCamera() {
        capture_->running = false;
        if (!worker_.joinable())
            return;
        std::unique_lock<std::mutex> lock(capture_->mutex);
        const bool done = capture_->finished.wait_for(lock, std::chrono::milliseconds(500),
                                                      [&]() { return capture_->done; });
        lock.unlock();
        if (done)
            worker_.join();
        else {
            // Some camera drivers cannot cancel read(). The read-only worker owns its
            // entire state; it never captures this or any actuator. Do not hang exit.
            std::cerr
                << "primary capture did not close promptly; terminating with read-only worker pending\n";
            worker_.detach();
        }
    }
};
} // namespace
int main(int argc, char **argv) {
    try {
        std::string config = "config/race.json", taskName = "observe", video, trafficVideo, output = "runs",
                    secondaryOverride, primaryOverride;
        bool drive = false, supervised = false, handoff = false, show = false;
        int maxFrames = 0;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            auto value = [&]() {
                if (++i >= argc)
                    throw std::runtime_error("missing argument: " + a);
                return std::string(argv[i]);
            };
            if (a == "--config")
                config = value();
            else if (a == "--task")
                taskName = value();
            else if (a == "--video")
                video = value();
            else if (a == "--traffic-video")
                trafficVideo = value();
            else if (a == "--output")
                output = value();
            else if (a == "--secondary")
                secondaryOverride = value();
            else if (a == "--primary")
                primaryOverride = value();
            else if (a == "--max-frames")
                maxFrames = std::stoi(value());
            else if (a == "--drive")
                drive = true;
            else if (a == "--supervised")
                supervised = true;
            else if (a == "--handoff-complete")
                handoff = true;
            else if (a == "--show")
                show = true;
            else if (a == "--help") {
                std::cout << "smartcar --task observe|race|zebra|traffic|obstacle|parking [--config "
                             "config/race.json]\n"
                          << "Default: no actuator writes. Live --drive requires --supervised; race also "
                             "--handoff-complete.\n"
                          << "Offline: --video secondary.avi [--traffic-video primary.avi] [--max-frames N] "
                             "[--show]\n";
                return 0;
            } else
                throw std::runtime_error("unknown argument: " + a);
        }
        Task task = taskFrom(taskName);
        Settings settings = load(fs::u8path(config));
        if (drive && (!supervised || !video.empty() || !trafficVideo.empty() || task == Task::Observe))
            throw std::runtime_error("live drive requires supervised live camera task");
        if (drive && task == Task::Race && !handoff)
            throw std::runtime_error("release remote controller and complete stationary handoff before race");
        GroundMap ground(settings.ground), roadGround(settings.road);
        Controller controller(settings.control, task);
        RoadPerception perception(settings.control, roadGround);
        if (!settings.zebra_profile.empty())
            perception.setZebraDistanceProfile(std::make_shared<ZebraDistanceProfile>(
                settings.zebra_profile, "secondary",
                secondaryOverride.empty() ? settings.secondary : secondaryOverride));
        ParkingDetector parking(settings.control, ground);
        ImageProcess::TrafficDetector traffic;
        cv::VideoCapture roadCamera, trafficCamera;
        std::string roadSource =
            video.empty() ? (secondaryOverride.empty() ? settings.secondary : secondaryOverride) : video;
        if (!roadCamera.open(roadSource))
            throw std::runtime_error("cannot open secondary camera/video: " + roadSource);
        if (video.empty()) {
            roadCamera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            roadCamera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            roadCamera.set(cv::CAP_PROP_BUFFERSIZE, 1);
            roadCamera.set(cv::CAP_PROP_FPS, 30);
        }
        bool needsTraffic = task == Task::Race || task == Task::Traffic || task == Task::Observe;
        std::unique_ptr<LatestCamera> livePrimary;
        std::string trafficSource;
        if (needsTraffic) {
            std::string path = trafficVideo.empty()
                                   ? (primaryOverride.empty() ? settings.primary : primaryOverride)
                                   : trafficVideo;
            if (!video.empty() && trafficVideo.empty())
                path = video;
            trafficSource = path;
            if (video.empty() && trafficVideo.empty())
                livePrimary = std::make_unique<LatestCamera>(path);
            else if (!trafficCamera.open(path))
                throw std::runtime_error("cannot open primary traffic video");
        }
        cv::Mat frame;
        // A single missed frame on a live camera must not end the whole run. A replay file
        // legitimately ends when read() fails, so only the live path retries and recovers.
        const bool liveRoad = video.empty();
        int roadReadFailures = 0;
        auto nextRoadFrame = [&]() -> bool {
            for (;;) {
                if (interrupted)
                    return false;
                if (roadCamera.read(frame) && !frame.empty()) {
                    roadReadFailures = 0;
                    return true;
                }
                if (!liveRoad)
                    return false; // replay end is a normal termination
                if (++roadReadFailures > 250) { // ~5 s at 20 ms before giving up
                    std::cerr << "secondary camera read failed repeatedly; stopping\n";
                    return false;
                }
                if (roadReadFailures == 1)
                    std::cerr << "secondary camera read failed; retrying\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        };
        if (!nextRoadFrame())
            throw std::runtime_error("secondary camera has no frames");
        // Calibration covers full 4:3 frames. Never reshape a cropped/wide image to fit it.
        if (std::abs(double(frame.cols) / frame.rows - 4.0 / 3) > 0.01)
            throw std::runtime_error("camera aspect ratio differs from calibration; no crop/warp allowed");
        fs::path run =
            fs::u8path(output) / std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        fs::create_directories(run);
        // Persist the actual input identities so a future camera-swap diagnosis
        // does not have to infer them from a mutable configuration file.
        {
            auto resolved = [](const std::string &path) {
                std::error_code error;
                const auto value = fs::canonical(fs::u8path(path), error);
                return error ? std::string{} : value.u8string();
            };
            std::ofstream inputs(run / "inputs.json");
            inputs << "{\"secondary_source\":\"" << escape(roadSource)
                   << "\",\"secondary_resolved\":\"" << escape(resolved(roadSource))
                   << "\",\"primary_source\":\""
                   << escape(trafficSource)
                   << "\",\"primary_resolved\":\"" << escape(resolved(trafficSource))
                   << "\",\"road_ground\":\"" << escape(settings.road)
                   << "\",\"parking_ground\":\"" << escape(settings.ground)
                   << "\",\"zebra_distance_profile\":\"" << escape(settings.zebra_profile)
                   << "\",\"processed_image_size\":[480,360],\"live\":"
                   << (video.empty() ? "true" : "false") << "}\n";
        }
        std::ofstream log(run / "telemetry.jsonl");
        if (!log)
            throw std::runtime_error("cannot create run log");
        Recorder recorder((run / "secondary.avi").u8string());
        std::unique_ptr<Recorder> trafficRecorder;
        if (needsTraffic)
            trafficRecorder = std::make_unique<Recorder>((run / "primary.avi").u8string());
        std::signal(SIGINT, signalStop);
        std::signal(SIGTERM, signalStop);
        Actuator actuator(settings.control, drive, settings.port);
        std::future<void> speech;
        std::uint64_t frameId = 0;
        double replayFps = roadCamera.get(cv::CAP_PROP_FPS);
        if (replayFps < 1)
            replayFps = 30;
        std::string previousState;
        bool trafficActive = false;
        std::uint64_t lastRecordedLamp = 0;
        double raceStart = -1, lastControlTime = -1, redWaitSeconds = 0;
        bool completedTask = false;
        do {
            if (interrupted)
                break;
            if (frame.empty()) { // defensive: an empty frame must never terminate the run
                if (!nextRoadFrame())
                    break;
            }
            ++frameId;
            double timestamp = video.empty() ? monotonic() : double(frameId) / replayFps;
            cv::resize(frame, frame, cv::Size(480, 360)); // uniform full-frame scale only
            Observation o = perception.update(frame, frameId, timestamp);
            if (o.motion_valid)
                parking.predictMotion(o.forward_delta, o.lateral_delta, o.yaw_delta);
            o.bay = parking.detect(frame, o.lane);
            bool trafficStage = controller.stage() == Stage::TrafficApproach ||
                                controller.stage() == Stage::TrafficBrake ||
                                controller.stage() == Stage::TrafficWait;
            if (trafficStage && !trafficActive)
                traffic.reset(timestamp);
            trafficActive = trafficStage;
            // Consume offline paired frames together to preserve their recorded alignment.
            cv::Mat lamp;
            std::uint64_t lampId = frameId;
            double lampStamp = timestamp;
            bool lampOk = false;
            if (needsTraffic && !livePrimary)
                lampOk = trafficCamera.read(lamp) && !lamp.empty();
            if (needsTraffic && (trafficStage || task == Task::Observe)) {
                if (livePrimary)
                    lampOk = livePrimary->latest(lamp, lampId, lampStamp) && monotonic() - lampStamp < .5;
                if (!lampOk) {
                    lamp.release();
                    if (trafficStage)
                        o.frame_ok = false;
                }
                auto action = traffic.update(lamp, lampId, lampStamp);
                auto state = traffic.getState();
                o.red = action == ImageProcess::TrafficLightAction::RED_STOP ||
                        state == ImageProcess::TrafficLightState::WAIT_GREEN;
                // GREEN_CONFIRMED is history; permission uses fresh associated light evidence below.
                if (lampOk && lampId != lastRecordedLamp) {
                    cv::Mat recordedLamp;
                    cv::resize(lamp, recordedLamp, cv::Size(480, 360));
                    trafficRecorder->frame(recordedLamp, lampId, lampStamp);
                    lastRecordedLamp = lampId;
                }
            }
            // Motion/light evidence uses capture time; pulse/hold timing starts when the
            // command is actually decided, after image processing, not before it.
            const double controlTime = video.empty() ? monotonic() : timestamp;
            o.time = controlTime;
            if (trafficStage)
                o.green = traffic.currentGreen(controlTime);
            o.brake_feedback_valid = drive && video.empty();
            o.brake_completed_event_id = actuator.completedBrakeEvent();
            Command q = controller.update(o);
            const Stage nowStage = controller.stage();
            if (raceStart < 0 && q.throttle_us >= settings.control.forward_min_us)
                raceStart = controlTime;
            if (lastControlTime >= 0 && nowStage == Stage::TrafficWait) {
                const double dt = controlTime - lastControlTime;
                if (dt > 0 && dt < 1.0)
                    redWaitSeconds += dt;
            }
            lastControlTime = controlTime;
            if (q.done)
                completedTask = true;
            actuator.submit(q, video.empty() ? timestamp : -1.0);
            if (q.announce &&
                (!speech.valid() || speech.wait_for(std::chrono::seconds(0)) == std::future_status::ready))
                speech = std::async(std::launch::async, speak, settings.speech);
            const auto &visionDiagnostics = perception.diagnostics();
            log << std::setprecision(12) << "{\"frame\":" << frameId << ",\"time\":" << timestamp
                << ",\"control_time\":" << controlTime << ",\"state\":\"" << q.state << "\",\"reason\":\""
                << escape(q.reason) << "\",\"requested_throttle_us\":" << q.throttle_us
                << ",\"steering_us\":" << q.steering_us << ",\"last_output_us\":" << actuator.lastOutputUs()
                << ",\"local_gap_stop_count\":" << actuator.gapStopCount()
                << ",\"drive_enabled\":" << (drive ? "true" : "false") << ",\"lane_valid\":" << o.lane.valid
                << ",\"board_visible\":" << o.board_visible
                << ",\"board_observation_valid\":" << o.board_observation_valid
                << ",\"board_candidate\":" << visionDiagnostics.board_candidate
                << ",\"board_reference_valid\":" << visionDiagnostics.board_reference_valid
                << ",\"board_clear_fraction\":" << visionDiagnostics.board_clear_fraction
                << ",\"board_blue_fraction\":" << visionDiagnostics.board_blue_fraction
                << ",\"white_components\":" << visionDiagnostics.white_components
                << ",\"image_lane_candidates\":" << visionDiagnostics.image_lane_candidates
                << ",\"rejected_tapes\":" << visionDiagnostics.rejected_tapes
                << ",\"lane_metric\":" << o.lane.metric << ",\"lane_left\":" << o.lane.left_x
                << ",\"lane_model_projected\":" << o.lane.model_projected
                << ",\"lane_uncertainty_m\":" << o.lane.lateral_uncertainty_m
                << ",\"lane_right\":" << o.lane.right_x << ",\"target_x\":" << q.target_x
                << ",\"zebra_detected\":" << o.zebra_detected << ",\"zebra_distance_valid\":" << o.zebra
                << ",\"zebra_rear_axle_y\":" << o.zebra_y
                << ",\"zebra_image_stripes\":" << visionDiagnostics.zebra_image_stripes
                << ",\"zebra_near_edge_px\":" << visionDiagnostics.zebra_near_edge_px
                << ",\"zebra_image_reason\":\"" << escape(visionDiagnostics.zebra_image_reason) << "\""
                << ",\"zebra_range_status\":\"" << escape(o.zebra_range_status) << "\""
                << ",\"zebra_distance_reason\":\"" << escape(o.zebra_distance_reason) << "\""
                << ",\"cones_visible\":" << o.cones.size() << ",\"cones_passed\":" << q.cones_passed
                << ",\"cone_image_candidates\":" << visionDiagnostics.cone_image_candidates
                << ",\"unmapped_cones\":" << visionDiagnostics.unmapped_cones
                << ",\"cone_model_candidates\":" << visionDiagnostics.cone_model_candidates
                << ",\"brake_event_id\":" << q.brake_event_id
                << ",\"brake_completed_event_id\":" << o.brake_completed_event_id
                << ",\"bay_valid\":" << o.bay.valid << ",\"entry_y\":" << o.bay.entry_y
                << ",\"end_y\":" << o.bay.end_y << ",\"entry_visible\":" << o.bay.entry_visible
                << ",\"end_visible\":" << o.bay.end_visible << ",\"left_clear\":" << o.bay.left_clear
                << ",\"right_clear\":" << o.bay.right_clear << ",\"left_blocked\":" << o.bay.left_blocked
                << ",\"right_blocked\":" << o.bay.right_blocked << ",\"bay_confidence\":" << o.bay.confidence
                << ",\"motion_valid\":" << o.motion_valid << ",\"forward_delta\":" << o.forward_delta
                << ",\"stopped\":" << o.stopped_confirmed << ",\"red\":" << o.red << ",\"green\":" << o.green
                << "}\n";
            log.flush(); // bounded one-line telemetry, visible without waiting for a stage change
            recorder.frame(frame, frameId, timestamp);
            if (previousState != q.state) {
                std::cout << q.state << ": " << q.reason << '\n';
                log.flush();
                previousState = q.state;
            }
            if (show) {
                cv::Mat display = frame.clone();
                cv::putText(display, q.state, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .6,
                            cv::Scalar(0, 255, 255), 2);
                cv::imshow("smartcar observation", display);
                if (cv::waitKey(1) == 27)
                    break;
            }
            if (q.done || (maxFrames > 0 && frameId >= static_cast<unsigned>(maxFrames)))
                break;
        } while (nextRoadFrame());
        {
            // 有效时间 T = 运行时间 + 加罚时间 - 红绿灯停车时间（细则二.5）。加罚由裁判记录，
            // 本程序只负责测运行时间与红绿灯等待时间，便于现场核对成绩。
            const double runTime =
                (raceStart >= 0 && lastControlTime >= 0) ? lastControlTime - raceStart : 0.0;
            const double validTime = runTime - redWaitSeconds;
            std::cout << (completedTask ? "TASK COMPLETE" : "STOPPED") << ": run_time="
                      << std::fixed << std::setprecision(2) << runTime << "s red_wait=" << redWaitSeconds
                      << "s valid_time_estimate=" << validTime << "s\n";
            log << std::setprecision(12) << "{\"summary\":true,\"completed\":"
                << (completedTask ? "true" : "false") << ",\"run_time_s\":" << runTime
                << ",\"red_wait_s\":" << redWaitSeconds << ",\"valid_time_estimate_s\":" << validTime
                << "}\n";
        }
        actuator.close();
        log.flush();
        roadCamera.release();
        trafficCamera.release();
        cv::destroyAllWindows();
        std::cout << "Stopped; run evidence: " << run.u8string() << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
