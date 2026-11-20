#include "smartcar/runtime.hpp"
#include <cassert>
#include <iostream>
using namespace smartcar;
int main() {
    const std::vector<unsigned char> expectedBuzzer{0xaa,0x55,2,8,0xd0,7,150,0,100,0,1,0,0xbd};
    assert(launchBuzzerPacket()==expectedBuzzer);
    const std::vector<unsigned char> expectedStop{0xaa,0x55,2,8,0,0,100,0,100,0,1,0,0x9d};
    assert(launchBuzzerPacket(true)==expectedStop);
    auto p = actuatorPacket(1610, 1500);
    assert(p.size() == 15);
    assert(p[0] == 0xaa && p[1] == 0x55 && p[3] == 10);
    assert(p[8] == 3 && p[11] == 4 && p[9] == 0x4a && p[10] == 6 && p[12] == 0xdc && p[13] == 5);
    unsigned char crc = 0;
    for (std::size_t i = 2; i + 1 < p.size(); ++i) {
        crc ^= p[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
    }
    assert(p.back() == crc);
    bool rejected = false;
    try {
        actuatorPacket(1610, 1560);
    } catch (...) {
        rejected = true;
    }
    assert(rejected);
    for (int throttle : {1400, 1500, 1570, 1600})
        assert(actuatorPacket(1610, throttle).size() == 15);
    {
        OutputPolicy outdoor(Config{});
        Command cmd;
        cmd.outdoor_parking_command = true;
        cmd.throttle_us = 1588;
        assert(outdoor.resolve(cmd, 1., 1.20).throttle_us == 1588);
        assert(outdoor.resolve(1.26).throttle_us == 1500);
        cmd.throttle_us = 1400; cmd.brake_event_id = 1;
        assert(outdoor.resolve(cmd, 2., 2.).throttle_us == 1400);
        assert(outdoor.resolve(2.24).throttle_us == 1400);
        assert(outdoor.resolve(2.26).throttle_us == 1500);
    }
    Config policyConfig;
    OutputPolicy policy(policyConfig);
    Command pulse;
    pulse.throttle_us = 1400;
    pulse.brake_event_id = 1;
    assert(policy.resolve(pulse, 0, 0).throttle_us == 1400);
    assert(policy.activeBrakeEvent() == 1 && policy.completedBrakeEvent() == 0);
    Command neutral;
    neutral.throttle_us = 1500;
    assert(policy.resolve(neutral, .05, .05).throttle_us == 1400); // visual outage cannot cut the pulse
    assert(policy.resolve(pulse, .30, .30).throttle_us == 1400);
    assert(policy.resolve(pulse, .36, .36).throttle_us == 1500);
    assert(policy.completedBrakeEvent() == 1 && policy.activeBrakeEvent() == 0);
    assert(policy.resolve(pulse, .60, .60).throttle_us == 1500); // repeated frame cannot extend/re-arm brake
    assert(policy.resolve(neutral, .7, .7).throttle_us == 1500);
    assert(policy.resolve(pulse, .75, .75).throttle_us == 1500); // recovery must not replay event 1
    pulse.brake_event_id = 2;
    assert(policy.resolve(pulse, .8, .8).throttle_us == 1400);
    assert(policy.resolve(2).throttle_us == 1500); // completion still produces neutral before expiry handling
    assert(policy.completedBrakeEvent() == 2);
    pulse.throttle_us = 1570;
    pulse.brake_event_id = 0;
    assert(policy.resolve(pulse, 2, 2.9).throttle_us == 1570);
    assert(policy.resolve(pulse, 2, 3.01).throttle_us == 1500);
    {
        OutputPolicy pending(policyConfig);
        Command brake;
        brake.throttle_us = 1400;
        brake.brake_event_id = 1;
        pending.submit(brake, 0);
        pending.submit(neutral, .001);
        pending.submit(neutral, .002); // multiple producer updates before first writer tick
        assert(pending.resolve(.02).throttle_us == 1400);
        assert(pending.resolve(.36).throttle_us == 1400);
        assert(pending.resolve(.38).throttle_us == 1500);
        assert(pending.completedBrakeEvent() == 1);
        assert(pending.resolve(brake, 2, 2).throttle_us == 1500); // expired / resumed same ID
        brake.brake_event_id = 2;
        assert(pending.resolve(brake, 3, 3).throttle_us == 1400);
        assert(pending.stop(3.05).throttle_us == 1500);
        assert(pending.activeBrakeEvent() == 0);
        assert(pending.completedBrakeEvent() == 1); // cancellation is not claimed completion
        assert(pending.resolve(brake, 3.06, 3.06).throttle_us == 1500);
    }
    {
        OutputPolicy queued(policyConfig);
        Command brake;
        brake.throttle_us = 1400;
        brake.brake_event_id = 4;
        queued.submit(brake, 0);
        brake.brake_event_id = 5;
        queued.submit(brake, .001);
        queued.submit(neutral, .002);
        assert(queued.resolve(.02).brake_event_id == 4);
        assert(queued.resolve(.38).throttle_us == 1500);
        assert(queued.completedBrakeEvent() == 4);
        assert(queued.resolve(.40).brake_event_id == 5);
        queued.stop(.41); // explicit stop cancels active and all pending events
        assert(queued.resolve(.42).throttle_us == 1500);
        assert(queued.completedBrakeEvent() == 4);
    }
    Config c;
    Actuator a(c, false, "never-open-this");
    Command q;
    q.throttle_us = 1570;
    a.submit(q);
    a.check();
    q.throttle_us = 1400;
    q.brake_event_id = 1;
    a.submit(q);
    a.submit(neutral); // synchronous submission must retain brake despite immediate replacement
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (a.activeBrakeEvent() != 1 && std::chrono::steady_clock::now() < timeout)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(a.activeBrakeEvent() == 1 && a.lastOutputUs() == 1400);
    a.close();
    assert(a.activeBrakeEvent() == 0 && a.lastOutputUs() == 1500 && a.completedBrakeEvent() == 0);
    {
        Actuator beep(c,false,"never-open-for-beep");
        Command launch;
        launch.launch_beep=true;
        beep.submit(launch);
        beep.submit(neutral); // producer replacement cannot lose the one-shot cue
        const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(1);
        while (!beep.launchBeepEvents() && std::chrono::steady_clock::now()<end)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        assert(beep.launchBeepEvents()==1 && !beep.launchBeepSent());
        for(int i=0;i<10;++i) beep.submit(launch);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        assert(beep.launchBeepEvents()==1);
        const auto stopDeadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
        while (beep.buzzerActive() && std::chrono::steady_clock::now()<stopDeadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        assert(!beep.buzzerActive()); // no producer frames required to stop the buzzer
        beep.close();
        assert(beep.lastOutputUs()==1500 && !beep.buzzerActive());
    }
    std::cout << "runtime packet and dry-output checks passed\n";
}
