#include "smartcar/controller.hpp"
#include "smartcar/perception.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace smartcar;
static void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
struct Trial {
    Config config;
    GroundMap ground{"config/calibration/ground_road.json"};
    RoadPerception perception{config,ground};
    Controller controller{config,Task::Race};
    std::uint64_t id=0;
    bool visible=false, cleared=false, released=false, forward=false;
    int beeps=0;
    Command step(const cv::Mat &image,double stamp) {
        auto o=perception.update(image,++id,stamp);
        visible=visible||o.board_visible;
        cleared=cleared||(visible&&o.board_observation_valid&&!o.board_visible);
        auto q=controller.update(o);
        if(q.launch_beep) ++beeps;
        if(controller.stage()!=Stage::WaitBoard) {
            require(visible&&cleared,"launch without confirmed board appearance and removal");
            released=true;
        }
        if(q.throttle_us>=config.forward_min_us) {
            require(released&&o.lane.valid,"forward without released start and current lane");
            forward=true;
        }
        return q;
    }
};
int main(int argc,char**argv) {
    try {
        cv::Mat clear=cv::imread("tests/fixtures/start_sky_clear.jpg");
        require(!clear.empty(),"missing actual outdoor clear frame");
        Trial t;
        double stamp=0;
        for(int i=0;i<12;++i) {
            t.step(clear,stamp+=.1);
            require(!t.released&&!t.forward,"initial clear sky must not arm or launch the car");
        }
        for(const char *name:{"start_sky_board_100.jpg","start_sky_board_110.jpg",
                              "start_sky_board_120.jpg","start_sky_board_130.jpg"}) {
            cv::Mat panel=cv::imread(std::string("tests/fixtures/")+name);
            require(!panel.empty(),"missing actual outdoor board frame");
            for(int i=0;i<3;++i) {
                t.step(panel,stamp+=.1);
                require(!t.released&&!t.forward,"visible panel must keep vehicle stopped");
            }
        }
        require(t.visible,"sunlit panel merged with sky was not confirmed");
        for(int i=0;i<12;++i)t.step(clear,stamp+=.1);
        require(t.released&&t.forward,"confirmed removal with current lane did not release forward");
        require(t.beeps==1,"board launch cue must occur exactly once");
        std::cout<<"Actual outdoor frames: board confirmed, held stopped, removed, forward requested; no actuator code\n";
        if(argc==2) {
            cv::VideoCapture video(argv[1]);
            require(video.isOpened(),"cannot read actual recorded run");
            Trial replay;
            cv::Mat frame;
            int frames=0,release_frame=0,forward_frame=0;
            while(frames<450&&video.read(frame)) {
                ++frames;
                replay.step(frame,frames*.1);
                if(replay.released&&!release_frame)release_frame=frames;
                if(replay.forward) {forward_frame=frames;break;}
            }
            require(replay.visible&&replay.cleared&&replay.forward,
                    "actual failed run still cannot reach a forward request after board removal");
            std::cout<<"{\"recorded_run_release_video_frame\":"<<release_frame
                     <<",\"first_forward_video_frame\":"<<forward_frame
                     <<",\"drive_enabled\":false}\n";
        }
        return 0;
    }catch(const std::exception &e) {
        std::cerr<<e.what()<<'\n';return 1;
    }
}
