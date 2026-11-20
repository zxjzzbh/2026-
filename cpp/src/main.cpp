#include "smartcar/controller.hpp"
#include "smartcar/announcement.hpp"
#include "smartcar/hybrid.hpp"
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
    std::string modules = "../algorithms";
    std::string announcement_wav = "audio/zebra_announcement.wav", announcement_device = "auto";
    std::string announcement_backend = "i2c_tts", announcement_helper;
    int announcement_volume = 8;
    std::string announcement_wav_text = "我是山东大学启航队的无人车，请为我加油！";
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
    number("zebra_brake_retry_enabled", s.control.zebra_brake_retry_enabled);
    number("parking_stop_gap_m", s.control.parking_stop_gap);
    number("forward_us", s.control.forward_us);
    number("steering_gain", s.control.steering_gain);
    number("traffic_max_approach_s", s.control.traffic_max_approach_s);
    string("secondary_camera", s.secondary);
    string("primary_camera", s.primary);
    string("uart", s.port);
    string("announcement_text", s.speech);
    string("announcement_backend", s.announcement_backend);
    number("announcement_volume", s.announcement_volume);
    s.announcement_helper = fs::absolute(p.parent_path()/"../tools/announce_i2c.py").u8string();
    string("announcement_wav", s.announcement_wav);
    string("announcement_device", s.announcement_device);
    string("announcement_wav_text", s.announcement_wav_text);
    s.announcement_wav = (p.parent_path() / fs::u8path(s.announcement_wav)).u8string();
    string("parking_ground", s.ground);
    string("road_ground", s.road);
    string("legacy_module_dir", s.modules);
    s.modules = fs::absolute(p.parent_path() / fs::u8path(s.modules)).u8string();
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
        worker_ = std::thread([capture = capture_, source]() {
            bool outage = false;
            const auto device = fs::u8path(source);
            const bool local = device.is_absolute();
            int backend = cv::CAP_ANY;
#ifdef __linux__
            if (source.compare(0, 5, "/dev/") == 0) backend = cv::CAP_V4L2;
#endif
            while (capture->running) {
                try {
                    std::error_code error;
                    if (!capture->camera.isOpened() && (!local || fs::exists(device, error)) &&
                        capture->camera.open(source, backend)) {
                        capture->camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
                        capture->camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                        capture->camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
                        capture->camera.set(cv::CAP_PROP_FPS, 30);
                    }
                    cv::Mat next;
                    if (capture->camera.isOpened() && capture->camera.read(next) && !next.empty()) {
                        std::lock_guard<std::mutex> lock(capture->mutex);
                        capture->frame = std::move(next);
                        capture->timestamp = monotonic();
                        ++capture->id;
                        capture->finished.notify_all();
                        outage = false;
                        continue;
                    }
                } catch (const cv::Exception &error) {
                    if (!outage) std::cerr << error.what() << '\n';
                }
                if (!outage) std::cerr << "camera unavailable; retrying " << source << '\n';
                outage = true;
                capture->camera.release();
                std::unique_lock<std::mutex> lock(capture->mutex);
                capture->finished.wait_for(lock, std::chrono::milliseconds(300),
                                           [&] { return !capture->running; });
            }
            capture->camera.release();
            {
                std::lock_guard<std::mutex> lock(capture->mutex);
                capture->done = true;
            }
            capture->finished.notify_all();
        });
    }
    bool latest(cv::Mat &frame, std::uint64_t &id, double &stamp, bool waitForNew = false) {
        std::unique_lock<std::mutex> lock(capture_->mutex);
        if (waitForNew)
            capture_->finished.wait_for(lock, std::chrono::milliseconds(50),
                                        [&] { return capture_->id > id || capture_->done; });
        if (capture_->frame.empty() || (waitForNew && capture_->id == id))
            return false;
        frame = capture_->frame; // worker replaces the Mat; this reference pins its pixel buffer
        id = capture_->id;
        stamp = capture_->timestamp;
        return true;
    }
    ~LatestCamera() {
        capture_->running = false;
        capture_->finished.notify_all();
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
                << "capture did not close promptly; exiting with read-only worker pending\n";
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
        if (drive && maxFrames > 0)
            throw std::runtime_error("--max-frames is for observation or replay only");
        std::signal(SIGINT, signalStop);
        std::signal(SIGTERM, signalStop);
#ifdef SIGBREAK
        std::signal(SIGBREAK, signalStop);
#endif
        GroundMap roadGround(settings.road);
        settings.control.outdoor_parking = true;
        Controller controller(settings.control, task);
        RoadPerception perception(settings.control, roadGround);
        if (!settings.zebra_profile.empty())
            perception.setZebraDistanceProfile(std::make_shared<ZebraDistanceProfile>(
                settings.zebra_profile, "secondary",
                secondaryOverride.empty() ? settings.secondary : secondaryOverride));
        HybridVision hybrid(settings.modules, fs::absolute(fs::u8path(config)).u8string(),
                            primaryOverride.empty() ? settings.primary : primaryOverride,
                            secondaryOverride.empty() ? settings.secondary : secondaryOverride);
        cv::VideoCapture roadCamera, trafficCamera;
        std::string roadSource =
            video.empty() ? (secondaryOverride.empty() ? settings.secondary : secondaryOverride) : video;
        const bool liveRoad = video.empty();
        std::unique_ptr<LatestCamera> liveSecondary;
        if (liveRoad)
            liveSecondary = std::make_unique<LatestCamera>(roadSource);
        else if (!roadCamera.open(roadSource))
            throw std::runtime_error("cannot open secondary replay: " + roadSource);
        bool needsTraffic = task != Task::Parking && (video.empty() || !trafficVideo.empty());
        std::unique_ptr<LatestCamera> livePrimary;
        std::string trafficSource;
        if (needsTraffic) {
            std::string path = trafficVideo.empty()
                                   ? (primaryOverride.empty() ? settings.primary : primaryOverride)
                                   : trafficVideo;
            trafficSource = path;
            if (video.empty() && trafficVideo.empty())
                livePrimary = std::make_unique<LatestCamera>(path);
            else if (!trafficCamera.open(path))
                throw std::runtime_error("cannot open primary traffic video");
        }
        cv::Mat frame;
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
        Actuator actuator(settings.control, drive, settings.port);
        std::atomic<bool> cancelSpeech{false};
        std::future<AnnouncementResult> speech;
        std::string announcementStatus = "not_requested", announcementReason;
        int announcementAttempts = 0;
        std::uint64_t frameId = 0, observationId = 0, roadCameraId = 0;
        double replayFps = liveRoad ? 30 : roadCamera.get(cv::CAP_PROP_FPS);
        if (replayFps < 1)
            replayFps = 30;
        std::string previousState;
        const double primaryFps = trafficCamera.isOpened() ? trafficCamera.get(cv::CAP_PROP_FPS) : 0.;
        if (trafficCamera.isOpened() && (!std::isfinite(primaryFps) || primaryFps <= 0))
            throw std::runtime_error("primary replay needs a valid frame rate");
        std::uint64_t replayPrimaryId = 0;
        cv::Mat replayPrimary;
        bool primaryEof = false;
        std::uint64_t lastRecordedLamp = 0;
        double raceStart = -1, lastControlTime = -1, redWaitSeconds = 0;
        bool completedTask = false, waitingForRoad = false;
        double lastRoadCapture = monotonic();
        while (!interrupted) {
            double roadCaptured = -1;
            if (liveRoad) {
                actuator.check();
                const bool fresh = liveSecondary->latest(frame, roadCameraId, roadCaptured, true);
                const double now = monotonic();
                if (!waitingForRoad && (!fresh || now - roadCaptured > .5) && now - lastRoadCapture > .5) {
                    waitingForRoad = true;
                    Observation missing;
                    missing.frame_id = ++observationId;
                    missing.time = now;
                    missing.frame_ok = false;
                    actuator.submit(controller.update(missing));
                    perception.reset();
                    log << "{\"event\":\"waiting_for_road_camera\",\"state\":\"" << Controller::name(controller.stage())
                        << "\",\"reason\":\"等待地面相机恢复\",\"time\":" << std::setprecision(12) << now << "}\n" << std::flush;
                }
                if (!fresh || now - roadCaptured > .5) {
                    if (show && cv::waitKey(1) == 27) break;
                    continue;
                }
                waitingForRoad = false;
                lastRoadCapture = roadCaptured;
            } else if (!roadCamera.read(frame) || frame.empty()) {
                if (!frameId) throw std::runtime_error("secondary replay has no frames");
                break;
            }
            if (frame.cols * 3 != frame.rows * 4)
                throw std::runtime_error("secondary input must have a full 4:3 frame");
            ++frameId;
            double timestamp = video.empty() ? roadCaptured : double(frameId) / replayFps;
            cv::resize(frame, frame, cv::Size(480, 360)); // uniform full-frame scale only
            const Stage stage = controller.stage();
            const bool watch = task == Task::Observe;
            const bool parkingStage = stage == Stage::ParkingApproach || stage == Stage::ParkingBrake || stage == Stage::ParkingHold;
            const double roadBegin = monotonic();
            PerceptionTasks roadTasks{watch || (stage != Stage::Obstacles && stage != Stage::ZebraHold && !parkingStage),
                                      watch, watch || stage == Stage::WaitBoard};
            Observation o = perception.update(frame, ++observationId, timestamp, roadTasks);
            const double roadMs = (monotonic()-roadBegin)*1000;
            const bool useHybrid = watch || stage == Stage::ZebraApproach || stage == Stage::TrafficApproach ||
                stage == Stage::TrafficBrake || stage == Stage::TrafficWait || stage == Stage::Obstacles || parkingStage;
            cv::Mat lamp;
            std::uint64_t lampId = 0;
            double lampStamp = -1;
            if (trafficCamera.isOpened()) {
                // Pair by media time; multiple road frames may reference one primary frame.
                while (!primaryEof && double(replayPrimaryId + 1)/primaryFps <= timestamp + 1e-8) {
                    cv::Mat next;
                    if (!trafficCamera.read(next)) { primaryEof = true; break; }
                    replayPrimary = std::move(next);
                    ++replayPrimaryId;
                }
                lampId = replayPrimaryId;
                lampStamp = double(lampId)/primaryFps;
                if (lampId && timestamp-lampStamp <= ((watch || stage == Stage::Obstacles) ? .5 : .25)) lamp = replayPrimary;
            } else if (livePrimary && useHybrid) {
                livePrimary->latest(lamp,lampId,lampStamp);
            }
            if (!lamp.empty()) {
                if (lamp.cols*3 != lamp.rows*4) throw std::runtime_error("primary input must have a full 4:3 frame");
                cv::resize(lamp,lamp,cv::Size(480,360));
                if (lampId != lastRecordedLamp && trafficRecorder && useHybrid) {
                    trafficRecorder->frame(lamp,lampId,lampStamp);
                    lastRecordedLamp = lampId;
                }
            }
            HybridEvidence legacy;
            const double hybridBegin = monotonic();
            if (useHybrid) {
                legacy = hybrid.update(watch ? "observe" : Controller::name(stage), frame, lamp, lampId,
                                        lampStamp, liveRoad ? monotonic() : timestamp, settings.control, o);
            }
            const double hybridMs = (monotonic()-hybridBegin)*1000;
            const double parkingMs = parkingStage ? hybridMs : 0.;
            const double controlTime = liveRoad ? monotonic() : timestamp;
            o.time = controlTime;
            if(liveRoad && controlTime-timestamp>.25) {
                o.motion_valid=false;
                o.stopped_confirmed=false;
            }
            if (controlTime > legacy.signal_stop_until) o.signal_stop = false;
            if (o.green && controlTime > legacy.green_until) o.green = false;
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
            if (!completedTask) lastControlTime = controlTime;
            if (q.done)
                completedTask = true;
            if (!q.keep_existing_output)
                actuator.submit(q, video.empty() ? timestamp : -1.0);
            if (speech.valid() && speech.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                const auto result = speech.get();
                announcementStatus = result.completed ? "completed" : result.command_sent ? "submitted" : "failed";
                announcementReason = result.reason;
                std::cerr << "announcement " << announcementStatus << ": " << announcementReason << '\n';
            }
            if (q.announce) {
                announcementStatus = "pending";
                announcementAttempts = 0;
            }
            if (nowStage == Stage::ZebraHold && q.zebra_hold_started &&
                announcementStatus != "completed" && announcementStatus != "submitted" && announcementStatus != "simulated" &&
                !speech.valid() && announcementAttempts < (settings.announcement_backend == "i2c_tts" ? 1 : 2) && q.zebra_hold_remaining_s > .5) {
                if (!drive) {
                    announcementStatus = "simulated";
                    announcementReason = "offline replay never opens audio hardware";
                } else if (settings.announcement_backend == "wav" && settings.speech != settings.announcement_wav_text) {
                    announcementStatus = "failed";
                    announcementAttempts = 2;
                    announcementReason = "configured text differs from recorded WAV text";
                } else {
                    cancelSpeech.store(false);
                    const double deadline = std::min(5.5, q.zebra_hold_remaining_s);
                    ++announcementAttempts;
                    announcementStatus = "playing";
                    if (settings.announcement_backend == "i2c_tts")
                        speech = std::async(std::launch::async,playExpansionTTS,
                            std::string(HYBRID_PYTHON_EXECUTABLE),settings.announcement_helper,
                            settings.speech,settings.announcement_volume,deadline,std::cref(cancelSpeech));
                    else if (settings.announcement_backend == "wav")
                        speech = std::async(std::launch::async, playAnnouncement,
                            settings.announcement_wav, settings.announcement_device, deadline,
                            std::cref(cancelSpeech), std::string("aplay"));
                    else {
                        announcementStatus="failed";announcementAttempts=2;
                        announcementReason="unknown announcement backend";
                    }
                }
            }
            if (nowStage != Stage::ZebraHold && speech.valid()) cancelSpeech.store(true);
            const auto &visionDiagnostics = perception.diagnostics();
            log << std::setprecision(12) << "{\"frame\":" << frameId << ",\"time\":" << timestamp
                << ",\"control_time\":" << controlTime << ",\"state\":\"" << q.state << "\",\"reason\":\""
                << escape(q.reason) << "\",\"requested_throttle_us\":" << q.throttle_us
                << ",\"steering_us\":" << q.steering_us << ",\"last_output_us\":" << actuator.lastOutputUs()
                << ",\"local_gap_stop_count\":" << actuator.gapStopCount()
                << ",\"drive_enabled\":" << (drive ? "true" : "false") << ",\"lane_valid\":" << o.lane.valid
                << ",\"road_processing_ms\":" << roadMs
                << ",\"lane_processing_enabled\":" << roadTasks.lane
                << ",\"parking_processing_ms\":" << parkingMs
                << ",\"hybrid_processing_ms\":" << hybridMs
                << ",\"capture_age_ms\":" << std::max(0.0,controlTime-timestamp)*1000
                << ",\"launch_beep_requested\":" << q.launch_beep
                << ",\"launch_beep_sent\":" << actuator.launchBeepSent()
                << ",\"board_visible\":" << o.board_visible
                << ",\"board_observation_valid\":" << o.board_observation_valid
                << ",\"board_candidate\":" << visionDiagnostics.board_candidate
                << ",\"board_reference_valid\":" << visionDiagnostics.board_reference_valid
                << ",\"board_clear_fraction\":" << visionDiagnostics.board_clear_fraction
                << ",\"board_blue_fraction\":" << visionDiagnostics.board_blue_fraction
                << ",\"board_path_blue_fraction\":" << visionDiagnostics.board_path_blue_fraction
                << ",\"white_components\":" << visionDiagnostics.white_components
                << ",\"image_lane_candidates\":" << visionDiagnostics.image_lane_candidates
                << ",\"rejected_tapes\":" << visionDiagnostics.rejected_tapes
                << ",\"lane_metric\":" << o.lane.metric << ",\"lane_left\":" << o.lane.left_x
                << ",\"lane_model_projected\":" << o.lane.model_projected
                << ",\"lane_uncertainty_m\":" << o.lane.lateral_uncertainty_m
                << ",\"lane_right\":" << o.lane.right_x << ",\"target_x\":" << q.target_x
                << ",\"zebra_detected\":" << o.zebra_detected << ",\"zebra_distance_valid\":" << o.zebra
                << ",\"zebra_rear_axle_y\":" << o.zebra_y
                << ",\"zebra_image_stripes\":" << (useHybrid ? legacy.stripes : 0)
                << ",\"zebra_near_edge_px\":" << (useHybrid ? legacy.near_px : -1.)
                << ",\"zebra_image_reason\":\"" << escape(o.zebra_distance_reason) << "\""
                << ",\"zebra_range_status\":\"" << escape(o.zebra_range_status) << "\""
                << ",\"zebra_distance_reason\":\"" << escape(o.zebra_distance_reason) << "\""
                << ",\"cone_raw_primary\":" << legacy.cone_raw_primary
                << ",\"cone_lane_valid\":" << legacy.cone_lane_valid
                << ",\"cone_raw_secondary\":" << legacy.cone_raw_secondary
                << ",\"cone_confirmed_primary\":" << legacy.cone_confirmed_primary
                << ",\"cone_confirmed_secondary\":" << legacy.cone_confirmed_secondary
                << ",\"cone_processing_ms\":" << legacy.cone_processing_ms
                << ",\"cones_visible\":" << legacy.cones_visible << ",\"cones_passed\":" << q.cones_passed
                << ",\"legacy_cone_phase\":\"" << escape(legacy.cone_phase) << "\",\"legacy_cone_camera\":\"" << escape(legacy.cone_camera)
                << "\",\"legacy_zebra_camera\":\"" << escape(legacy.zebra_camera) << "\",\"legacy_signal_color\":\"" << escape(legacy.signal_color) << "\""
                << ",\"brake_event_id\":" << q.brake_event_id
                << ",\"zebra_brake_retries\":" << q.zebra_brake_retries
                << ",\"brake_completed_event_id\":" << o.brake_completed_event_id
                << ",\"outdoor_parking_stage\":\"" << (o.outdoor_parking ? escape(o.outdoor_parking->stage) : "") << "\""
                << ",\"outdoor_parking_side\":\"" << (o.outdoor_parking ? escape(o.outdoor_parking->side) : "") << "\""
                << ",\"outdoor_parking_reason\":\"" << (o.outdoor_parking ? escape(o.outdoor_parking->reason) : "") << "\""
                << ",\"bay_valid\":" << o.bay.valid << ",\"entry_y\":" << o.bay.entry_y
                << ",\"end_y\":" << o.bay.end_y << ",\"entry_visible\":" << o.bay.entry_visible
                << ",\"end_visible\":" << o.bay.end_visible << ",\"left_clear\":" << o.bay.left_clear
                << ",\"right_clear\":" << o.bay.right_clear << ",\"left_blocked\":" << o.bay.left_blocked
                << ",\"right_blocked\":" << o.bay.right_blocked << ",\"bay_confidence\":" << o.bay.confidence
                << ",\"zebra_brake_retries\":" << q.zebra_brake_retries
                << ",\"zebra_hold_started\":" << q.zebra_hold_started
                << ",\"zebra_hold_elapsed_s\":" << q.zebra_hold_elapsed_s
                << ",\"zebra_hold_remaining_s\":" << q.zebra_hold_remaining_s
                << ",\"announcement_status\":\"" << escape(announcementStatus) << "\""
                << ",\"announcement_reason\":\"" << escape(announcementReason) << "\""
                << ",\"announcement_attempts\":" << announcementAttempts
                << ",\"motion_valid\":" << o.motion_valid << ",\"forward_delta\":" << o.forward_delta
                << ",\"stopped\":" << o.stopped_confirmed << ",\"red\":" << o.red
                << ",\"signal_span_px\":" << legacy.signal_span_px
                << ",\"signal_stop\":" << o.signal_stop << ",\"green\":" << o.green
                << ",\"signal_stop_reason\":\"" << escape(legacy.signal_stop_reason) << "\""
                << ",\"signal_observation\":\"" << escape(legacy.signal_observation) << "\""
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
            if (maxFrames > 0 && frameId >= static_cast<unsigned>(maxFrames))
                break;
        }
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
        cancelSpeech.store(true);
        if (speech.valid()) speech.get();
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
