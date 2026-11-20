#include "smartcar/controller.hpp"
#include <opencv2/core.hpp>
#include <cassert>
#include <iostream>
using namespace smartcar;
int main() {
    cv::FileStorage file("tests/fixtures/zebra_loss_20261120.json",cv::FileStorage::READ);
    assert(file.isOpened());
    for(const auto &trial:file["cases"]) {
        Config config;config.forward_us=1580;
        Controller car(config,Task::Zebra);
        bool braked=false;
        const double last_seen=double(trial["last_seen_time"]);
        for(const auto &r:trial["frames"]) {
            Observation o;o.frame_id=int(r["frame"]);o.time=double(r["control_time"]);
            o.zebra_detected=int(r["zebra_detected"]);o.zebra=int(r["zebra_distance_valid"]);
            o.zebra_y=double(r["zebra_rear_axle_y"]);o.zebra_range_status=std::string(r["zebra_range_status"]);
            o.zebra_near_px=double(r["zebra_near_edge_px"]);o.zebra_source=std::string(r["legacy_zebra_camera"]);
            o.lane.valid=true;o.lane.both=true;o.lane.left_x=-.585;o.lane.right_x=.585;
            auto q=car.update(o);
            if(q.throttle_us==1400) {
                assert(q.brake_event_id>0);
                assert(o.time-last_seen<=.35);
                assert(q.reason.find("approaching crosswalk lost")!=std::string::npos);
                std::cout<<"recorded run "<<int(trial["original_run"])<<" brakes at frame "<<o.frame_id<<"\n";
                braked=true;break;
            }
        }
        assert(braked);
    }
    for(bool isolated:{false,true}) {
        Controller car(Config{},Task::Zebra);
        for(int i=0;i<20;++i) {
            Observation o;o.frame_id=i+1;o.time=i*.1;
            o.zebra_detected=i<3 && (!isolated || i==0);
            o.zebra_near_px=190;o.zebra_source="secondary";o.zebra_range_status="beyond_far";
            assert(car.update(o).throttle_us!=1400);
        }
    }
}
