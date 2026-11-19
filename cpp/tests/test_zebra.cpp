#include "smartcar/zebra.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace smartcar;
namespace {
void check(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
cv::Mat background() { return cv::Mat(360, 480, CV_8UC3, cv::Scalar(150, 145, 218)); }
cv::Mat row(int count, cv::Scalar white = {240, 240, 240}) {
    auto image = background();
    for (int i = 0; i < count; ++i)
        cv::rectangle(image, {80 + i * 70, 190, 30, 28}, white, cv::FILLED);
    return image;
}
void absent(const cv::Mat &image, const char *message) {
    const auto r = detectZebraImage(image);
    check(!r.candidate && r.stripes.empty() && std::isnan(r.near_edge_px), message);
}
void replayCheck(const cv::Mat &image, bool present, bool known_edge, const std::string &label) {
    const auto result = detectZebraImage(image);
    if (result.candidate != present || (present && result.stripes.size() < 3) ||
        (!present && !result.stripes.empty()) || std::isfinite(result.near_edge_px) != known_edge)
        throw std::runtime_error("replay mismatch: " + label + ", candidate=" +
                                 std::to_string(result.candidate) + ", edge=" +
                                 std::to_string(result.near_edge_px));
}
} // namespace

int main() {
    std::filesystem::path temporary_video;
    try {
        absent({}, "empty image must not provide road evidence");
        absent(cv::Mat(360, 480, CV_8UC1, cv::Scalar(255)), "wrong image type must be rejected");
        absent(cv::Mat(360, 480, CV_8UC3, cv::Scalar(255, 255, 255)),
               "fully overexposed image must not become a zebra");
        absent(background(), "sunlit pink ground must not become a zebra");
        absent(row(1), "one white strip is not a crosswalk");
        absent(row(2), "two white strips are not a crosswalk");
        auto longitudinal = background();
        for (int x : {70, 170, 270, 370})
            cv::rectangle(longitudinal, {x, 150, 9, 200}, {240, 240, 240}, cv::FILLED);
        absent(longitudinal, "parallel longitudinal lane markings must not become a crosswalk");
        auto block = background();
        cv::rectangle(block, {70, 160, 330, 110}, {245, 245, 245}, cv::FILLED);
        absent(block, "a single large reflection must not become a crosswalk");
        auto thin_marks = background();
        for (int i = 0; i < 5; ++i)
            cv::rectangle(thin_marks, {65 + i * 65, 190, 40, 5}, {245, 245, 245}, cv::FILLED);
        absent(thin_marks, "aligned thin paint fragments are not substantial white paper strips");
        auto staggered = background();
        for (int i = 0; i < 4; ++i)
            cv::rectangle(staggered, {60 + i * 90, 150 + i * 45, 30, 25}, {240, 240, 240}, cv::FILLED);
        absent(staggered, "bright patches on different rows must not form a crosswalk");
        auto irregular = background();
        for (int x : {35, 80, 360})
            cv::rectangle(irregular, {x, 190, 30, 28}, {240, 240, 240}, cv::FILLED);
        absent(irregular, "inconsistent spacing must not connect unrelated objects");

        auto regular = detectZebraImage(row(5));
        check(regular.candidate && regular.stripes.size() == 5, "five separated bright strips must be found");
        check(std::abs(regular.near_edge_px - 217.0) < .01, "near edge must be the actual row edge");
        auto shadow = row(5, {160, 160, 160});
        // Apply the same shadow to the ground; paper remains locally brighter.
        for (int y = 0; y < shadow.rows; ++y)
            for (int x = 0; x < shadow.cols; ++x)
                if (shadow.at<cv::Vec3b>(y, x) == cv::Vec3b(150, 145, 218))
                    shadow.at<cv::Vec3b>(y, x) = cv::Vec3b(70, 68, 110);
        check(detectZebraImage(shadow).candidate,
              "moderately shadowed white paper must not need a 200+ brightness threshold");
        auto clipped = background();
        for (int i = 0; i < 4; ++i)
            cv::rectangle(clipped, {70 + i * 80, 320, 35, 40}, {245, 245, 245}, cv::FILLED);
        auto clipped_result = detectZebraImage(clipped);
        check(clipped_result.candidate && std::isnan(clipped_result.near_edge_px),
              "bottom clipping may support presence but must not fabricate a near-edge distance");

        auto occluded = background();
        for (int x : {90, 210, 330})
            cv::rectangle(occluded, {x, 220, 45, 90}, {245, 245, 245}, cv::FILLED);
        cv::rectangle(occluded, {175, 265, 130, 95}, {18, 18, 18}, cv::FILLED);
        auto occluded_result = detectZebraImage(occluded);
        check(occluded_result.candidate && std::isnan(occluded_result.near_edge_px),
              "hood-cut central paper leaves only two complete edges: presence must not imply distance");

        auto recoverable = background();
        for (int i = 0; i < 5; ++i)
            cv::rectangle(recoverable, {60 + i * 80, 220, 35, 90}, {245, 245, 245}, cv::FILLED);
        cv::rectangle(recoverable, {210, 265, 55, 95}, {18, 18, 18}, cv::FILLED);
        auto recoverable_result = detectZebraImage(recoverable);
        check(recoverable_result.candidate && std::abs(recoverable_result.near_edge_px - 309.0) < .01,
              "at least three complete consistent edges may measure the row after excluding an occluded one");

        const std::vector<std::string> names{"0401", "0410", "0420", "0430", "0460", "0470", "0780"};
        std::vector<cv::Mat> replay_images;
        for (const auto &name : names) {
            const auto image = cv::imread(std::string("tests/fixtures/zebra_outdoor_") + name + ".jpg");
            check(!image.empty(), "outdoor replay fixture must load");
            const bool present = name <= "0430";
            const bool known_edge = present && name != "0430";
            replayCheck(image, present, known_edge, name + " original");
            for (int quality : {95, 85, 70}) {
                std::vector<unsigned char> encoded;
                check(cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, quality}),
                      "JPEG regression sample must encode");
                replayCheck(cv::imdecode(encoded, cv::IMREAD_COLOR), present, known_edge,
                            name + " JPEG " + std::to_string(quality));
            }
            replay_images.push_back(image);
            const auto result = detectZebraImage(image);
            std::cout << "outdoor " << name << ": " << result.stripes.size()
                      << " strips, near-edge pixel " << result.near_edge_px << '\n';
        }
        // Exercise the actual MJPG encode/decode route used in saved vehicle
        // replays. This changes thin contours differently from the source JPEG.
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        temporary_video = std::filesystem::temp_directory_path() /
                          ("smartcar-zebra-replay-" + std::to_string(nonce) + ".avi");
        cv::VideoWriter writer(temporary_video.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               15.0, {480, 360});
        check(writer.isOpened(), "MJPG fixture writer must open");
        for (const auto &image : replay_images)
            writer.write(image);
        writer.release();
        cv::VideoCapture reader(temporary_video.string());
        check(reader.isOpened(), "MJPG fixture reader must open");
        for (const auto &name : names) {
            cv::Mat decoded;
            check(reader.read(decoded), "MJPG replay must preserve every image");
            const bool present = name <= "0430";
            replayCheck(decoded, present, present && name != "0430", name + " MJPG");
        }
        reader.release();
        std::filesystem::remove(temporary_video);
        temporary_video.clear();
        std::cout << "zebra image tests passed\n";
    } catch (const std::exception &e) {
        if (!temporary_video.empty()) {
            std::error_code ignored;
            std::filesystem::remove(temporary_video, ignored);
        }
        std::cerr << e.what() << '\n';
        return 1;
    }
}
