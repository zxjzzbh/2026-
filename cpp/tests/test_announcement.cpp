#include "smartcar/announcement.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#ifdef __linux__
#include <unistd.h>
#endif
using namespace smartcar;
namespace fs=std::filesystem;
void check(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
int main() {
#ifdef __linux__
    const auto dir=fs::temp_directory_path()/ ("smartcar-audio-test-"+std::to_string(getpid()));
    fs::create_directory(dir);
    try {
        std::atomic<bool> cancel{false};
        const auto wav=dir/"sample.wav",player=dir/"player";
        std::ofstream(wav) << "fixture";
        auto script=[&](const std::string &text) {
            std::ofstream(player)<<"#!/bin/sh\n"<<text<<'\n';
            fs::permissions(player, fs::perms::owner_all);
        };
        script("exit 0");
        check(playAnnouncement(wav.string(),"default",1,cancel,player.string()).completed,
              "successful player must produce completed feedback");
        const auto tts=playExpansionTTS(player.string(),wav.string(),"test",8,1,cancel);
        check(tts.command_sent && !tts.completed,
              "TTS delivery must not falsely acknowledge physical playback completion");
        script("exit 2");
        check(!playAnnouncement(wav.string(),"default",1,cancel,player.string()).completed,
              "player failure cannot be recorded as completed speech");
        script("sleep 30");
        const auto start=std::chrono::steady_clock::now();
        const auto timeout=playAnnouncement(wav.string(),"default",.15,cancel,player.string());
        check(!timeout.completed && timeout.reason=="audio playback timed out",
              "hung audio needs a bounded timeout");
        check(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<1,
              "hung audio cannot block shutdown");
        cancel.store(true);
        check(!playAnnouncement(wav.string(),"default",1,cancel,player.string()).completed,
              "cancelled task cannot start new sound");
        cancel.store(false);
        check(!playAnnouncement((dir/"missing.wav").string(),"default",1,cancel).completed,
              "missing recording must report failure");
        fs::remove_all(dir);
        std::cout<<"audio completion/failure/deadline/cancellation checks passed; no sound hardware opened\n";
        return 0;
    } catch (const std::exception &e) {
        fs::remove_all(dir);std::cerr<<e.what()<<'\n';return 1;
    }
#else
    std::cout<<"ALSA process tests require Linux\n";
#endif
}
