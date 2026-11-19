#include "TrafficLight.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ImageProcess {
namespace {
double extent(const cv::Rect &r) {
    return std::max(r.width, r.height);
}
double minor(const cv::Rect &r) {
    return std::min(r.width, r.height);
}
cv::Point2d centre(const cv::Rect &r) {
    return {r.x + r.width * 0.5, r.y + r.height * 0.5};
}
double ratio(double a, double b) {
    return b > 0 ? a / b : 0;
}
double longitudinal(const cv::Point2d &p, const cv::Rect &r, bool horizontal) {
    return horizontal ? (p.x - r.x) / r.width : (p.y - r.y) / r.height;
}
double transverse(const cv::Point2d &p, const cv::Rect &r, bool horizontal) {
    return horizontal ? (p.y - r.y) / r.height : (p.x - r.x) / r.width;
}
} // namespace

TrafficLightDetector::TrafficLightDetector() {
    reset();
}

bool TrafficLightDetector::currentGreen(double now) const {
    return state_ == TrafficLightState::GREEN_CONFIRMED && have_frame_ && std::isfinite(now) &&
           now >= last_time_ && now - last_time_ <= config_.confirmation_gap_s;
}

void TrafficLightDetector::setConfig(const TrafficLightConfig &config) {
    if (config.red_confirm_frames < 2 || config.green_confirm_frames < 3 ||
        !std::isfinite(config.confirmation_gap_s) || config.confirmation_gap_s <= 0 ||
        config.red_area_min <= 0 || config.red_circle_min <= 0 || config.red_circle_min > 1 ||
        config.housing_v_max < 1 || config.housing_v_max > 254 || config.morph_kernel_size < 0 ||
        config.morph_kernel_size > 15) {
        throw std::invalid_argument("invalid traffic confirmation or contour configuration");
    }
    config_ = config;
    interruptEvidence();
}

void TrafficLightDetector::breakConfirmation() {
    red_count_ = green_count_ = 0;
    have_pending_red_ = have_pending_green_ = false;
}

void TrafficLightDetector::interruptEvidence() {
    breakConfirmation();
    // Keep the previously measured red housing so a recovered camera can
    // reconfirm its green. A pre-outage release is not authority to drive now.
    if (state_ == TrafficLightState::GREEN_CONFIRMED)
        state_ = TrafficLightState::WAIT_GREEN;
}

void TrafficLightDetector::reset(double) {
    state_ = TrafficLightState::IDLE;
    have_track_ = have_frame_ = false;
    reconfirm_red_after_green_ = false;
    frame_size_ = cv::Size();
    last_frame_id_ = compatibility_frame_id_ = 0;
    last_time_ = -std::numeric_limits<double>::infinity();
    breakConfirmation();
    last_debug_info_ = TrafficLightDebugInfo();
}

cv::Mat TrafficLightDetector::roiMask(const cv::Size &size, int &roi_count) const {
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    auto add = [&](double y0, double y1) {
        if (!std::isfinite(y0) || !std::isfinite(y1) || !std::isfinite(config_.roi_x_ratio_start) ||
            !std::isfinite(config_.roi_x_ratio_end))
            return;
        const int x0 =
            std::max(0, std::min(size.width, static_cast<int>(size.width * config_.roi_x_ratio_start)));
        const int x1 =
            std::max(0, std::min(size.width, static_cast<int>(size.width * config_.roi_x_ratio_end)));
        const int top = std::max(0, std::min(size.height, static_cast<int>(size.height * y0)));
        const int bottom = std::max(0, std::min(size.height, static_cast<int>(size.height * y1)));
        if (x1 > x0 && bottom > top) {
            mask(cv::Rect(x0, top, x1 - x0, bottom - top)).setTo(255);
            ++roi_count;
        }
    };
    add(config_.roi_y_ratio_start, config_.roi_y_ratio_end);
    if (config_.dual_roi_enable)
        add(config_.lower_roi_y_ratio_start, config_.lower_roi_y_ratio_end);
    return mask; // union: overlapping ROIs never count a bulb twice
}

cv::Mat TrafficLightDetector::preprocessMask(const cv::Mat &mask) const {
    cv::Mat result = mask.clone();
    int k = config_.morph_kernel_size;
    if (k <= 1)
        return result;
    if (k % 2 == 0)
        ++k;
    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
    return result;
}

std::vector<TrafficLightDetector::Candidate>
TrafficLightDetector::candidates(const cv::Mat &mask, const std::vector<cv::Rect> &housings, bool red,
                                 TrafficLightDebugInfo &debug) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (red)
        debug.contour_count = static_cast<int>(contours.size());
    std::vector<Candidate> out;
    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < config_.red_area_min || area > mask.total() * config_.red_area_max_ratio)
            continue;
        if (red)
            ++debug.area_pass_count;
        auto bulb = cv::boundingRect(contour);
        const double aspect = ratio(bulb.width, bulb.height);
        if (aspect < config_.red_aspect_min || aspect > config_.red_aspect_max)
            continue;
        if (red)
            ++debug.aspect_pass_count;
        const double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 0)
            continue;
        const double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < config_.red_circle_min)
            continue;
        if (red) {
            ++debug.circle_pass_count;
            debug.best_circle = std::max(debug.best_circle, circularity);
            debug.best_area = std::max(debug.best_area, area);
        }
        const auto p = centre(bulb);
        for (const auto &box : housings) {
            if ((box & bulb).area() < bulb.area() * 0.95)
                continue;
            const double diameter = std::sqrt(bulb.width * bulb.height);
            if (extent(box) < 2.5 * diameter || extent(box) > 8 * diameter || minor(box) < 1.1 * diameter ||
                minor(box) > 3.5 * diameter)
                continue;
            const bool horizontal = box.width > box.height;
            const double position = longitudinal(p, box, horizontal);
            // In a three-lamp housing red and green occupy the end positions, never the centre.
            if (position > 0.38 && position < 0.62)
                continue;
            Candidate c;
            c.bulb = bulb;
            c.housing = box;
            c.centre = p;
            c.area = area;
            c.circle = circularity;
            c.horizontal = horizontal;
            out.push_back(c);
            break;
        }
    }
    std::sort(out.begin(), out.end(), [](const Candidate &a, const Candidate &b) { return a.area > b.area; });
    return out;
}

bool TrafficLightDetector::sameHousing(const Candidate &a, const Candidate &b) {
    if (a.horizontal != b.horizontal || a.housing.empty() || b.housing.empty())
        return false;
    const double large_ratio = ratio(extent(b.housing), extent(a.housing));
    const double minor_ratio = ratio(minor(b.housing), minor(a.housing));
    if (large_ratio < 0.65 || large_ratio > 1.55 || minor_ratio < 0.65 || minor_ratio > 1.55)
        return false;
    const double overlap =
        ratio((a.housing & b.housing).area(), std::min(a.housing.area(), b.housing.area()));
    const double displacement = cv::norm(centre(a.housing) - centre(b.housing));
    return overlap >= 0.55 && displacement <= std::max(12.0, extent(a.housing) * 0.25);
}

bool TrafficLightDetector::sameBulb(const Candidate &a, const Candidate &b) {
    return sameHousing(a, b) && std::abs(longitudinal(a.centre, a.housing, a.horizontal) -
                                         longitudinal(b.centre, b.housing, b.horizontal)) < 0.15;
}

bool TrafficLightDetector::greenMatchesRed(const Candidate &red, const Candidate &green) {
    if (!sameHousing(red, green))
        return false;
    const double scale = ratio(green.area, red.area);
    if (scale < 0.35 || scale > 2.8)
        return false;
    const double r = longitudinal(red.centre, red.housing, red.horizontal);
    const double g = longitudinal(green.centre, green.housing, green.horizontal);
    if ((r < 0.5) == (g < 0.5) || std::abs(r - g) < 0.35 || std::abs(r - g) > 0.85)
        return false;
    return std::abs(transverse(red.centre, red.housing, red.horizontal) -
                    transverse(green.centre, green.housing, green.horizontal)) < 0.25;
}

TrafficLightAction TrafficLightDetector::update(const cv::Mat &frame, std::uint64_t frame_id, double now) {
    TrafficLightDebugInfo debug;
    auto finish = [&](TrafficLightAction action, const char *reason) {
        debug.state = static_cast<int>(state_);
        debug.red_count = red_count_;
        debug.green_count = green_count_;
        debug.red_qualified = state_ == TrafficLightState::WAIT_GREEN;
        debug.green_qualified = currentGreen(now);
        debug.housing_found = have_track_;
        if (have_track_)
            debug.tracked_housing = track_.housing;
        debug.reason = reason;
        last_debug_info_ = debug;
        return action;
    };
    // Capture failure may carry the LAST frame ID/timestamp. Handle it before
    // duplicate suppression, otherwise a stalled camera retains a stale green.
    if (frame.empty() || frame.type() != CV_8UC3) {
        interruptEvidence();
        if (std::isfinite(now) && (!have_frame_ || (frame_id > last_frame_id_ && now > last_time_))) {
            have_frame_ = true;
            last_frame_id_ = frame_id;
            last_time_ = now;
            debug.fresh_frame = true;
        }
        return finish(TrafficLightAction::NONE,
                      "empty or invalid frame; green withdrawn and tracked red hold retained");
    }
    if (!std::isfinite(now) || (have_frame_ && now < last_time_)) {
        interruptEvidence();
        return finish(TrafficLightAction::NONE, "invalid capture timestamp; green withdrawn");
    }
    if (have_frame_ && frame_id <= last_frame_id_)
        return finish(TrafficLightAction::NONE, "duplicate or reordered frame ignored");
    if (have_frame_ && now == last_time_) {
        interruptEvidence();
        return finish(TrafficLightAction::NONE,
                      "new frame has non-increasing capture timestamp; green withdrawn");
    }
    if (have_frame_ && now - last_time_ > config_.confirmation_gap_s)
        interruptEvidence();
    have_frame_ = true;
    last_frame_id_ = frame_id;
    last_time_ = now;
    debug.fresh_frame = true;
    debug.frame_width = frame.cols;
    debug.frame_height = frame.rows;
    if (!frame_size_.empty() && frame_size_ != frame.size()) {
        interruptEvidence();
        have_track_ = false;
    }
    frame_size_ = frame.size();
    cv::Mat hsv, red1, red2, red, green, dark;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(config_.red1_h_min, config_.red_s_min, config_.red_v_min),
                cv::Scalar(config_.red1_h_max, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(config_.red2_h_min, config_.red_s_min, config_.red_v_min),
                cv::Scalar(config_.red2_h_max, 255, 255), red2);
    cv::bitwise_or(red1, red2, red);
    cv::inRange(hsv, cv::Scalar(config_.green_h_min, config_.green_s_min, config_.green_v_min),
                cv::Scalar(config_.green_h_max, 255, 255), green);
    auto roi = roiMask(frame.size(), debug.roi_count);
    cv::bitwise_and(red, roi, red);
    cv::bitwise_and(green, roi, green);
    red = preprocessMask(red);
    green = preprocessMask(green);
    debug.red_pixels = cv::countNonZero(red);
    debug.green_pixels = cv::countNonZero(green);
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, config_.housing_v_max), dark);
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> housings;
    for (const auto &contour : contours) {
        auto r = cv::boundingRect(contour);
        double aspect = ratio(extent(r), minor(r));
        if (aspect >= 1.8 && aspect <= 7.0 && r.area() < frame.total() * 0.25 &&
            ratio(cv::countNonZero(dark(r)), r.area()) >= 0.30)
            housings.push_back(r);
    }
    auto reds = candidates(red, housings, true, debug);
    auto greens = candidates(green, housings, false, debug);
    debug.red_found = !reds.empty();
    debug.green_found = !greens.empty();

    // Any valid current red suppresses green, including simultaneous colours or another signal.
    if (!reds.empty()) {
        green_count_ = 0;
        have_pending_green_ = false;
        // A newly visible red revokes a previously confirmed green immediately.
        // The red stop itself still needs its consecutive fresh-frame confirmation.
        if (state_ == TrafficLightState::GREEN_CONFIRMED) {
            // A single red candidate cancels permission to move, not the fact
            // that this is a previously confirmed signal. Keep green recovery
            // reachable if the red candidate was a one-frame reflection.
            state_ = TrafficLightState::WAIT_GREEN;
            reconfirm_red_after_green_ = true;
            red_count_ = 0;
            have_pending_red_ = false;
        }
        const Candidate *selected = &reds.front();
        if (have_track_)
            for (const auto &r : reds)
                if (sameBulb(track_, r)) {
                    selected = &r;
                    break;
                }
        if (state_ == TrafficLightState::WAIT_GREEN) {
            if (!have_track_ || sameBulb(track_, *selected)) {
                track_ = *selected;
                have_track_ = true;
            }
            if (reconfirm_red_after_green_) {
                red_count_ = (have_pending_red_ && sameBulb(pending_red_, *selected)) ? red_count_ + 1 : 1;
                pending_red_ = *selected;
                have_pending_red_ = true;
                if (red_count_ >= config_.red_confirm_frames) {
                    track_ = *selected;
                    have_track_ = true;
                    reconfirm_red_after_green_ = false;
                    return finish(TrafficLightAction::RED_STOP, "new red reconfirmed after green withdrawal");
                }
            }
            return finish(TrafficLightAction::NONE, "red present; green release suppressed");
        }
        red_count_ = (have_pending_red_ && sameBulb(pending_red_, *selected)) ? red_count_ + 1 : 1;
        pending_red_ = *selected;
        have_pending_red_ = true;
        if (red_count_ >= config_.red_confirm_frames) {
            track_ = *selected;
            have_track_ = true;
            state_ = TrafficLightState::WAIT_GREEN;
            return finish(TrafficLightAction::RED_STOP, "red confirmed on consecutive fresh frames");
        }
        return finish(TrafficLightAction::NONE, "red candidate awaiting next fresh frame");
    }
    red_count_ = 0;
    have_pending_red_ = false;
    if (state_ == TrafficLightState::GREEN_CONFIRMED) {
        const bool still_green = have_track_ &&
                                 std::any_of(greens.begin(), greens.end(), [&](const Candidate &g) {
                                     return greenMatchesRed(track_, g);
                                 });
        if (still_green)
            return finish(TrafficLightAction::NONE, "same housing green remains visible and current");
        interruptEvidence();
        return finish(TrafficLightAction::NONE, "current associated green absent; release withdrawn");
    }
    if (state_ != TrafficLightState::WAIT_GREEN || !have_track_) {
        green_count_ = 0;
        have_pending_green_ = false;
        return finish(TrafficLightAction::NONE, "no tracked red housing; green cannot authorize release");
    }
    for (const auto &g : greens) {
        if (!greenMatchesRed(track_, g))
            continue;
        green_count_ = (have_pending_green_ && sameBulb(pending_green_, g)) ? green_count_ + 1 : 1;
        pending_green_ = g;
        have_pending_green_ = true;
        if (green_count_ >= config_.green_confirm_frames) {
            state_ = TrafficLightState::GREEN_CONFIRMED;
            reconfirm_red_after_green_ = false;
            breakConfirmation();
            return finish(TrafficLightAction::GREEN_GO, "same housing green confirmed; no current red");
        }
        return finish(TrafficLightAction::NONE, "same housing green awaiting consecutive fresh frames");
    }
    green_count_ = 0;
    have_pending_green_ = false;
    return finish(TrafficLightAction::NONE, "waiting for round green bulb in tracked red housing");
}

TrafficLightAction TrafficLightDetector::update(const cv::Mat &frame) {
    const double now =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    compatibility_frame_id_ = std::max(compatibility_frame_id_, last_frame_id_) + 1;
    return update(frame, compatibility_frame_id_, now);
}
} // namespace ImageProcess
