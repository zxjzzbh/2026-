#include "smartcar/perception.hpp"
#include "smartcar/controller.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>
using namespace smartcar;
int main() {
    Config c;
    GroundMap g("config/calibration/ground_road.json");
    for(const char* name:{"secondary-701","secondary-1007","secondary-1291","secondary-1678"}) {
        const auto image=cv::imread(std::string("tests/fixtures/field_20261120_")+name+".jpg");
        if(image.empty()) throw std::runtime_error("missing recorded field fixture");
        RoadPerception p(c,g);
        Observation o;
        for(int i=0;i<4;++i) o=p.update(image,i+1,i*.1);
        std::cout<<name<<" lane="<<o.lane.valid<<" both="<<o.lane.both
                 <<" metric="<<o.lane.metric<<" board="<<o.board_visible<<"\n";
        if(std::string(name)=="secondary-701") {
            if(!o.board_visible) throw std::runtime_error("sunlit board merged with sky was missed");
        } else if(!o.lane.valid || !o.lane.both)
            throw std::runtime_error("actual visible paired lane was missed");
    }
    RoadPerception p(c,g);
    Controller car(c,Task::Race);
    int beeps=0;Command q;std::uint64_t id=0;
    for(const char* n:{"secondary-701","secondary-1007"}) {
        auto image=cv::imread(std::string("tests/fixtures/field_20261120_")+n+".jpg");
        for(int i=0;i<9;++i) {
            auto o=p.update(image,++id,id*.1);
            q=car.update(o);beeps+=q.launch_beep;
            if(std::string(n)=="secondary-701" && q.throttle_us!=1500)
                throw std::runtime_error("visible board released motion");
        }
    }
    if(car.stage()!=Stage::ZebraApproach || q.throttle_us<1570 || beeps!=1)
        throw std::runtime_error("actual board/clear pair failed launch logic");
    RoadPerception guarded(c,g);Controller waiting(c,Task::Race);
    std::uint64_t frame=0;
    for(const auto sample:{std::pair<const char*,int>{"secondary-701",9},
                          {"secondary-660",2},{"secondary-666",6},{"secondary-700",9}}) {
        auto im=cv::imread(std::string("tests/fixtures/field_20261120_")+sample.first+".jpg");
        for(int i=0;i<sample.second;++i) {
            ++frame;auto o=guarded.update(im,frame,frame*.1);
            auto command=waiting.update(o);
            if(waiting.stage()!=Stage::WaitBoard || command.throttle_us!=1500 || command.launch_beep)
                throw std::runtime_error("partially returned board completed pending removal");
        }
    }
    auto clear=cv::imread("tests/fixtures/field_20261120_secondary-800.jpg");
    for(int i=0;i<9;++i) {
        ++frame;auto o=guarded.update(clear,frame,frame*.1);waiting.update(o);
    }
    if(waiting.stage()!=Stage::ZebraApproach)
        throw std::runtime_error("clear scene could not recover after returned board");
    auto image=cv::imread("tests/fixtures/field_20261120_secondary-1007.jpg");
    auto skipped=p.update(image,++id,id*.1,{false,false,false});
    if(skipped.lane.valid || skipped.board_visible || skipped.zebra_detected)
        throw std::runtime_error("disabled stage analysis returned stale evidence");
}
