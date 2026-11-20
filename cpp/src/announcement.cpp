#include "smartcar/announcement.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>
#ifdef __linux__
#include <cerrno>
#include <csignal>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

namespace smartcar {
#ifdef __linux__
namespace {
AnnouncementResult runPlayer(std::vector<std::string> strings, double timeout_s, const std::atomic<bool> &cancel) {
    if (strings.empty() || !std::isfinite(timeout_s) || timeout_s<=0 || timeout_s>10)
        return {false,"invalid audio subprocess deadline"};
    const std::string program=strings.front();
    std::vector<char *> args;
    for (auto &s : strings) args.push_back(s.data());
    args.push_back(nullptr);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attr, 0);
    pid_t pid = -1;
    const int error = posix_spawnp(&pid, program.c_str(), nullptr, &attr, args.data(), environ);
    posix_spawnattr_destroy(&attr);
    if (error) return {false, "cannot start audio player: " + std::to_string(error)};
    const auto started = std::chrono::steady_clock::now();
    int status = 0;
    while (true) {
        const pid_t waited = waitpid(pid, &status, WNOHANG);
        if (waited == pid)
            return {WIFEXITED(status) && WEXITSTATUS(status) == 0,
                    WIFEXITED(status) ? "audio player exit " + std::to_string(WEXITSTATUS(status)) :
                                       "audio player terminated"};
        if (waited < 0 && errno != EINTR) return {false, "audio waitpid failed"};
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        if (cancel.load() || elapsed >= timeout_s) {
            // Own process group only: a hung player and its children cannot
            // block camera/control progress or retain orphan audio processes.
            kill(-pid, SIGTERM);
            bool reaped = false;
            for (int i=0; i<10; ++i) {
                if (waitpid(pid, &status, WNOHANG) == pid) { reaped=true; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            kill(-pid, SIGKILL);
            if (!reaped) while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
            return {false, cancel.load() ? "playback cancelled" : "audio playback timed out"};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
}
#endif
AnnouncementResult playAnnouncement(const std::string &wav, const std::string &device,
                                   double timeout_s, const std::atomic<bool> &cancel,
                                   const std::string &program) {
    if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 10 || device.empty())
        return {false, "invalid playback deadline/device"};
    if (!std::filesystem::is_regular_file(std::filesystem::u8path(wav)))
        return {false, "announcement WAV missing"};
    if (cancel.load()) return {false, "playback cancelled"};
#ifdef __linux__
    std::string output=device;
    if (device=="auto") {
        // Cameras may expose capture-only USB sound cards. Only actual
        // playback endpoints count, and HDMI is never silently selected for
        // a vehicle's unconnected speaker.
        std::ifstream pcm("/proc/asound/pcm");
        std::vector<std::string> choices;
        std::string line;
        while (std::getline(pcm,line)) {
            if (line.find("playback")==std::string::npos || line.find("HDMI")!=std::string::npos ||
                line.find("hdmi")!=std::string::npos) continue;
            int card=-1,dev=-1;char dash=0;
            std::istringstream fields(line);
            if (!(fields>>card>>dash>>dev) || dash!='-' || card<0 || dev<0) continue;
            std::ifstream idfile("/proc/asound/card"+std::to_string(card)+"/id");
            std::string id;idfile>>id;
            if (!id.empty()) choices.push_back("plughw:CARD="+id+",DEV="+std::to_string(dev));
        }
        if (choices.size()!=1)
            return {false, choices.empty() ? "no non-HDMI speaker playback device detected" :
                                           "multiple speaker devices; configure announcement_device"};
        output=choices.front();
    }
    return runPlayer({program,"-q","-D",output,wav},timeout_s,cancel);
#else
    return {false, "ALSA playback requires Linux"};
#endif
}
AnnouncementResult playExpansionTTS(const std::string &python, const std::string &helper,
                                   const std::string &text, int volume, double timeout_s,
                                   const std::atomic<bool> &cancel) {
    if (cancel.load()) return {false,"playback cancelled"};
    std::error_code error;
    if (!std::filesystem::is_regular_file(helper,error) || volume<0 || volume>10 || text.empty())
        return {false,"invalid TTS helper/text/volume"};
#ifdef __linux__
    auto result=runPlayer({python,helper,"--text",text,"--volume",std::to_string(volume)},timeout_s,cancel);
    if (result.completed) {
        result.completed=false;
        result.command_sent=true;
        result.reason="TTS sentence delivered; audible completion not automatically verified";
    }
    return result;
#else
    return {false,"expansion TTS requires Linux I2C"};
#endif
}

}
