#pragma once
#include <atomic>
#include <string>

namespace smartcar {
struct AnnouncementResult {
    bool completed = false;
    std::string reason;
    bool command_sent = false;
};
// Playback completion means the player exited successfully, not proof that a
// physical speaker was audible. No actuator or controller access.
AnnouncementResult playAnnouncement(const std::string &wav, const std::string &device,
                                   double timeout_s, const std::atomic<bool> &cancel,
                                   const std::string &program = "aplay");
AnnouncementResult playExpansionTTS(const std::string &python, const std::string &helper,
                                   const std::string &text, int volume, double timeout_s,
                                   const std::atomic<bool> &cancel);
}
