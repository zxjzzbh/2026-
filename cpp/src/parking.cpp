#include "smartcar/parking.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace smartcar {
namespace {
struct Line {
    double intercept = 0, slope = 0, weight = 0;
    cv::Vec4i pixels;
    bool metric = false;
    bool yellow = true;
    double representative_weight = 0;
};
double length(cv::Vec4i s) {
    return std::hypot(double(s[2] - s[0]), double(s[3] - s[1]));
}
std::vector<Line> merge(std::vector<Line> lines, double tolerance) {
    std::sort(lines.begin(), lines.end(), [](auto a, auto b) { return a.intercept < b.intercept; });
    std::vector<Line> out;
    for (auto line : lines) {
        if (out.empty() || line.intercept - out.back().intercept > tolerance) {
            line.representative_weight = line.weight;
            out.push_back(line);
            continue;
        }
        auto &previous = out.back();
        double w = previous.weight + line.weight;
        previous.intercept = (previous.intercept * previous.weight + line.intercept * line.weight) / w;
        previous.slope = (previous.slope * previous.weight + line.slope * line.weight) / w;
        if (line.weight > previous.representative_weight) {
            previous.pixels = line.pixels;
            previous.representative_weight = line.weight;
        }
        previous.weight = w;
        previous.yellow = previous.yellow || line.yellow;
    }
    return out;
}
std::vector<Line> mergeImageLines(std::vector<Line> lines, double tolerance, bool horizontal,
                                  double reference) {
    std::sort(lines.begin(), lines.end(), [](auto a, auto b) { return a.intercept < b.intercept; });
    std::vector<std::vector<Line>> groups;
    for (auto line : lines) {
        if (groups.empty() || line.intercept - groups.back().front().intercept > tolerance)
            groups.push_back({line});
        else
            groups.back().push_back(line);
    }
    std::vector<Line> out;
    for (auto &group : groups) {
        std::vector<cv::Point2f> points;
        double weight = 0;
        for (auto l : group) {
            points.emplace_back(float(l.pixels[0]), float(l.pixels[1]));
            points.emplace_back(float(l.pixels[2]), float(l.pixels[3]));
            weight += l.weight;
        }
        cv::Vec4f f;
        cv::fitLine(points, f, cv::DIST_L2, 0, .01, .01);
        double divisor = horizontal ? f[0] : f[1];
        if (std::abs(divisor) < .01)
            continue;
        double slope = (horizontal ? f[1] : f[0]) / divisor;
        double low = std::numeric_limits<double>::infinity(), high = -low;
        for (auto p : points) {
            double v = horizontal ? p.x : p.y;
            low = std::min(low, v);
            high = std::max(high, v);
        }
        double intercept = horizontal ? f[3] - slope * f[2] : f[2] + slope * (reference - f[3]);
        cv::Vec4i pixels;
        if (horizontal)
            pixels = {int(std::round(low)), int(std::round(intercept + slope * low)), int(std::round(high)),
                      int(std::round(intercept + slope * high))};
        else
            pixels = {int(std::round(intercept + slope * (low - reference))), int(std::round(low)),
                      int(std::round(intercept + slope * (high - reference))), int(std::round(high))};
        out.push_back(
            {horizontal ? intercept + slope * reference : intercept, slope, weight, pixels, false, true});
    }
    return out;
}
double pixelY(Line l, double x) {
    auto p = l.pixels;
    return p[1] + (x - p[0]) * double(p[3] - p[1]) / (p[2] - p[0]);
}
double lineX(Line l, double y) {
    auto p = l.pixels;
    if (std::abs(p[3] - p[1]) < 1)
        return (p[0] + p[2]) * .5;
    return p[0] + (y - p[1]) * double(p[2] - p[0]) / (p[3] - p[1]);
}
bool spansBay(Line transverse, Line left, Line right) {
    double y = (transverse.pixels[1] + transverse.pixels[3]) * .5;
    double a = lineX(left, y), b = lineX(right, y);
    double lo = std::min(transverse.pixels[0], transverse.pixels[2]);
    double hi = std::max(transverse.pixels[0], transverse.pixels[2]);
    return b > a && hi - lo > (b - a) * .45 && hi > a + (b - a) * .25 && lo < b - (b - a) * .25;
}
bool junction(const cv::Mat &bands, Line horizontal, Line vertical) {
    auto a = horizontal.pixels, b = vertical.pixels;
    cv::Point2d p(a[0], a[1]), r(a[2] - a[0], a[3] - a[1]);
    cv::Point2d q(b[0], b[1]), s(b[2] - b[0], b[3] - b[1]);
    double cross = r.x * s.y - r.y * s.x;
    if (std::abs(cross) < 1 || cv::norm(s) < 1)
        return false;
    double t = ((q.x - p.x) * s.y - (q.y - p.y) * s.x) / cross;
    cv::Point2d intersection = p + r * t;
    auto present = [&](cv::Point2d pt) {
        int x = int(std::round(pt.x)), y = int(std::round(pt.y));
        if (x < 2 || y < 2 || x >= bands.cols - 2 || y >= bands.rows - 2)
            return false;
        return cv::countNonZero(bands(cv::Rect(x - 2, y - 2, 5, 5))) >= 5;
    };
    if (!present(intersection))
        return false;
    s *= 1 / cv::norm(s);
    // Require the longitudinal band to actually reach the transverse band.
    // An intersection of infinitely extended, disconnected lines is not a corner.
    for (double sign : {-1., 1.}) {
        int support = 0;
        for (double d : {10., 16., 22.})
            if (present(intersection + s * (sign * d)))
                ++support;
        if (support >= 2)
            return true;
    }
    return false;
}
std::optional<Line> metricLine(const GroundMap &ground, cv::Vec4i seg, cv::Size size, bool along,
                               double reference) {
    std::vector<cv::Point2f> points;
    for (int n = 0; n <= 60; ++n) {
        double t = n / 60.;
        auto p = ground.project({seg[0] + (seg[2] - seg[0]) * t, seg[1] + (seg[3] - seg[1]) * t}, size);
        if (p)
            points.emplace_back(float(p->x), float(p->y));
    }
    if (points.size() < 3 || cv::norm(points.back() - points.front()) < .07)
        return {};
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, .005, .005);
    double divisor = along ? line[1] : line[0];
    if (std::abs(divisor) < .1)
        return {};
    double slope = along ? line[0] / divisor : line[1] / divisor;
    double intercept = along ? line[2] + slope * (reference - line[3]) : line[3] - slope * line[2];
    return Line{intercept, slope, length(seg), seg, true, true};
}
std::vector<Point> boardFeet(const GroundMap &ground, cv::Rect board, cv::Size size) {
    std::vector<Point> result;
    // A partly mapped board must not require its entire base/centre to lie in the
    // calibration hull. Use only actually observed samples on its contact edge.
    for (int i = 1; i < 10; ++i) {
        auto p = ground.project({board.x + board.width * i / 10., double(board.y + board.height - 1)}, size);
        if (p)
            result.push_back(*p);
    }
    return result;
}
Bay visibleHalfBay(const cv::Mat &image, const cv::Mat &hsv, const cv::Mat &yellow, const GroundMap &ground,
                   const Config &config) {
    Bay result;
    cv::Mat blue, gray, edges;
    cv::inRange(hsv, cv::Scalar(95, 65, 35), cv::Scalar(135, 255, 255), blue);
    std::vector<std::vector<cv::Point>> boards;
    cv::findContours(blue, boards, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Rect board;
    for (auto &c : boards)
        if (cv::contourArea(c) > image.total() * .003 && cv::boundingRect(c).area() > board.area())
            board = cv::boundingRect(c);
    if (board.area() == 0)
        return result;
    cv::Canny(yellow, edges, 35, 90);
    std::vector<cv::Vec4i> color_lines, gray_lines;
    cv::HoughLinesP(edges, color_lines, 1, CV_PI / 360, 12, image.cols * .045, 5);
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 25, 70);
    cv::HoughLinesP(edges, gray_lines, 1, CV_PI / 360, 15, image.cols * .055, 5);
    std::vector<Line> sides, entries, ends;
    double reference = image.rows * .70;
    for (auto p : color_lines) {
        double dx = p[2] - p[0], dy = p[3] - p[1];
        if (std::abs(dy) > image.rows * .025 && std::abs(dx / dy) < 6) {
            double x = p[0] + (reference - p[1]) * dx / dy;
            sides.push_back({x, dx / dy, length(p), p, false, true});
        }
        if (std::abs(dx) > image.cols * .10 && std::abs(dy / dx) < .18)
            entries.push_back({(p[1] + p[3]) * .5, dy / dx, length(p), p, false, true});
    }
    for (auto p : gray_lines) {
        double dx = p[2] - p[0], dy = p[3] - p[1];
        if (std::abs(dx) > image.cols * .055 && std::abs(dy / dx) < .20)
            ends.push_back({(p[1] + p[3]) * .5, dy / dx, length(p), p, false, false});
    }
    sides = mergeImageLines(sides, image.cols * .025, false, reference);
    sides.erase(
        std::remove_if(sides.begin(), sides.end(),
                       [&](const Line &l) { return std::abs(l.pixels[3] - l.pixels[1]) < image.rows * .08; }),
        sides.end());
    entries = mergeImageLines(entries, image.rows * .025, true, image.cols * .5);
    ends = mergeImageLines(ends, image.rows * .012, true, image.cols * .5);
    double best = 0;
    for (std::size_t i = 0; i < sides.size(); ++i)
        for (std::size_t j = i + 1; j < sides.size(); ++j) {
            auto left = sides[i], right = sides[j];
            double width = right.intercept - left.intercept;
            if (width < image.cols * .15 || width > image.cols * .70)
                continue;
            double bx = board.x + board.width * .5;
            bool empty_left = bx > right.intercept + image.cols * .02;
            bool empty_right = bx < left.intercept - image.cols * .02;
            if (!empty_left && !empty_right)
                continue;
            // A genuine visible half-bay has two side bands joined by a near yellow
            // entrance and a real far image edge. A/B labels and blank glare are irrelevant.
            for (auto entry : entries)
                for (auto end : ends) {
                    if (entry.intercept - end.intercept < image.rows * .08)
                        continue;
                    if (!spansBay(entry, left, right) || !spansBay(end, left, right))
                        continue;
                    if (!junction(yellow, entry, left) || !junction(yellow, entry, right) ||
                        !junction(yellow, end, left) || !junction(yellow, end, right))
                        continue;
                    auto metric_entry =
                        metricLine(ground, entry.pixels, image.size(), false, config.lookahead);
                    auto metric_left = metricLine(ground, left.pixels, image.size(), true, config.lookahead);
                    auto metric_right =
                        metricLine(ground, right.pixels, image.size(), true, config.lookahead);
                    if (!metric_entry || (!metric_left && !metric_right))
                        continue;
                    // The visible side adjacent to the board is the centre-divider side.
                    auto divider = empty_left ? metric_right : metric_left;
                    auto outer = empty_left ? metric_left : metric_right;
                    double heading = divider ? std::atan(divider->slope) : std::atan(outer->slope);
                    if (std::abs(heading) > .6 || std::abs(std::atan(metric_entry->slope) + heading) > .35)
                        continue;
                    double divider_x =
                        divider ? divider->intercept
                                : outer->intercept + (empty_left ? 1 : -1) * config.lane_width * .5;
                    double entry_y = metric_entry->intercept + config.tape_width * .5;
                    if (entry_y < 0)
                        continue;
                    double end_y = entry_y + config.parking_length / std::cos(heading);
                    auto metric_end = metricLine(ground, end.pixels, image.size(), false, config.lookahead);
                    bool end_measured = false;
                    if (metric_end) {
                        double measured = metric_end->intercept - config.tape_width * .5;
                        if (std::abs(measured - end_y) > .25)
                            continue;
                        end_y = measured;
                        end_measured = true;
                    }
                    bool matching_board = false;
                    for (auto foot : boardFeet(ground, board, image.size())) {
                        if (foot.y < entry_y - .30 || foot.y > end_y + .25)
                            continue;
                        double foot_x = foot.x - std::tan(heading) * (foot.y - config.lookahead);
                        if ((empty_left && foot_x > divider_x) || (empty_right && foot_x < divider_x))
                            matching_board = true;
                    }
                    if (!matching_board)
                        continue;
                    double score = left.weight + right.weight + entry.weight + end.weight;
                    if (score < best)
                        continue;
                    best = score;
                    result.valid = true;
                    result.entry_visible = true;
                    result.end_visible = end_measured;
                    result.divider_x = divider_x;
                    result.left_x = divider_x - config.lane_width * .5;
                    result.right_x = divider_x + config.lane_width * .5;
                    if (outer) {
                        if (empty_left)
                            result.left_x = outer->intercept + config.tape_width * .5;
                        else
                            result.right_x = outer->intercept - config.tape_width * .5;
                    }
                    result.entry_y = entry_y;
                    result.end_y = end_y;
                    result.heading = heading;
                    result.confidence = metric_end ? .75 : .60;
                }
        }
    return result;
}
// Fit the entrance over its middle span. Endpoints alone give a noisy heading
// when a one-pixel change at a distant corner maps to several centimetres.
std::optional<Line> measuredEntrance(const cv::Mat &tape, cv::Point left, cv::Point right,
                                     const GroundMap &ground, double lookahead) {
    double width = right.x - left.x;
    int radius = std::max(2, cvRound(tape.rows / 90.));
    std::vector<cv::Point2f> edge;
    int begin = cvRound(left.x + .12 * width), end = cvRound(right.x - .12 * width);
    for (int x = begin; x <= end; ++x) {
        double expected = left.y + (right.y - left.y) * (x - left.x) / width;
        int lo = std::max(1, cvRound(expected) - radius);
        int hi = std::min(tape.rows - 1, cvRound(expected) + radius);
        for (int y = lo; y <= hi; ++y) {
            if (tape.at<uchar>(y, x) && !tape.at<uchar>(y - 1, x)) {
                edge.emplace_back(float(x), float(y - 1));
                break;
            }
        }
    }
    if (edge.size() < .55 * (end - begin + 1))
        return {};
    cv::Vec4f fit;
    cv::fitLine(edge, fit, cv::DIST_HUBER, 0, .01, .01);
    if (std::abs(fit[0]) < .9)
        return {};
    auto row = [&](int x) { return fit[3] + (x - fit[2]) * fit[1] / fit[0]; };
    int supported = 0;
    for (auto p : edge)
        if (std::abs(p.y - row(cvRound(p.x))) <= .5 * radius)
            ++supported;
    if (supported < .75 * edge.size() || std::abs(left.y - row(left.x)) > radius ||
        std::abs(right.y - row(right.x)) > radius)
        return {};
    return metricLine(ground, {left.x, cvRound(row(left.x)), right.x, cvRound(row(right.x))},
                      tape.size(), false, lookahead);
}

// A board can hide one half while glare erases the far stripe. Look for the
// floor bounded by both side stripes and the near stripe in the visible half.
Bay entranceFromFloor(const cv::Mat &image, const cv::Mat &hsv, const cv::Mat &blue,
                      const cv::Mat &tape, const std::vector<cv::Point> &floor, cv::Rect board,
                      const std::vector<cv::Rect> &boards, int top,
                      const GroundMap &ground, const Config &config) {
    std::vector<cv::Point> hull;
    cv::convexHull(floor, hull);
    double hull_area = cv::contourArea(hull);
    if (hull_area <= 0 || cv::contourArea(floor) < .85 * hull_area)
        return {};
    cv::Point far_left(image.cols, top), far_right(-1, top), left = hull.front(), right = left;
    for (auto p : hull) {
        if (p.y == top) {
            far_left.x = std::min(far_left.x, p.x);
            far_right.x = std::max(far_right.x, p.x);
        }
        if (p.y - .17 * p.x > left.y - .17 * left.x)
            left = p;
        if (p.y + .17 * p.x > right.y + .17 * right.x)
            right = p;
    }
    double width = right.x - left.x, far_width = far_right.x - far_left.x;
    double foot_y = board.br().y - 1;
    if (width < image.cols * .12 || width > image.cols * .72 ||
        far_width < .43 * width || far_width > 1.07 * width ||
        std::min(left.y, right.y) < top + .055 * board.height ||
        std::abs(left.y - right.y) > .14 * width ||
        std::abs((left.y + right.y) * .5 - foot_y) > .20 * board.height ||
        cv::contourArea(std::vector<cv::Point>{far_left, far_right, right, left}) < .86 * hull_area)
        return {};
    bool empty_left = board.x + board.width * .5 > right.x;
    auto inner = empty_left ? right : left;
    double gap = empty_left ? board.x - inner.x : inner.x - board.br().x;
    if (gap < -.20 * board.width || gap > .45 * board.width)
        return {};

    // Check the bounded interior against the apron at the same image column.
    // This allows a sun/shade gradient but requires visible, unclipped floor.
    int floor_samples = 0, warm_floor_samples = 0;
    for (int i = 1; i <= 7; ++i) {
        double across = i / 8.;
        cv::Point2d lower = cv::Point2d(left) * (1 - across) + cv::Point2d(right) * across;
        cv::Point2d upper = cv::Point2d(far_left) * (1 - across) + cv::Point2d(far_right) * across;
        int apron_y = cvRound(lower.y + .27 * board.height);
        if (apron_y >= image.rows)
            return {};
        for (double depth : {.25, .5, .75}) {
            cv::Point p = upper * (1 - depth) + lower * depth;
            if (blue.at<uchar>(p))
                return {};
            auto value = hsv.at<cv::Vec3b>(p);
            auto color = image.at<cv::Vec3b>(p), apron = image.at<cv::Vec3b>(apron_y, p.x);
            int difference = std::max({std::abs(color[0] - apron[0]), std::abs(color[1] - apron[1]),
                                       std::abs(color[2] - apron[2])});
            if (value[0] <= 43 && value[1] >= 16 && value[2] > 28 && value[2] < 244)
                ++warm_floor_samples;
            if (value[2] > 28 && value[2] < 244 && value[1] > 12 && difference < 65)
                ++floor_samples;
        }
    }
    if (floor_samples < 14)
        return {};

    auto joint = ground.project(inner, image.size());
    if (!joint)
        return {};
    auto outer = ground.project(empty_left ? left : right, image.size());
    // If the floor itself takes on the hue of yellow tape, corner positions
    // drift along the reflection. Use a robust fit across the stripe when its
    // whole span is calibrated. Otherwise use the measured part of the edge.
    auto entry = outer && warm_floor_samples >= 4
                     ? measuredEntrance(tape, left, right, ground, config.lookahead) : std::nullopt;
    if (!entry)
        entry = metricLine(ground, {left.x, left.y, right.x, right.y}, image.size(), false, config.lookahead);
    if (!entry || entry->intercept <= 0 || std::abs(std::atan(entry->slope)) > .60)
        return {};
    if (outer && std::abs(std::hypot(outer->x - joint->x, outer->y - joint->y) -
                          (config.lane_width * .5 - config.tape_width)) > .20)
        return {};
    double heading = -std::atan(entry->slope), direction = empty_left ? 1. : -1.;
    double divider = joint->x + direction * config.tape_width * .5 / std::cos(heading) +
                     std::tan(heading) * (config.lookahead - joint->y);
    bool adjacent_board = false;
    for (auto foot : boardFeet(ground, board, image.size())) {
        double across = direction * (foot.x - divider - std::tan(heading) * (foot.y - config.lookahead));
        if (across > .5 * config.tape_width && across < .5 * config.lane_width + .05)
            adjacent_board = true;
    }
    if (!adjacent_board)
        return {};
    Bay result;
    result.valid = result.entry_visible = true;
    result.left_clear = result.right_blocked = empty_left;
    result.right_clear = result.left_blocked = !empty_left;
    // A second board can sit farther back than the strip used to find the
    // entrance. Check its base between the two observed side stripes as well.
    for (auto other : boards) {
        if (other == board)
            continue;
        double y = other.br().y - 1, x = other.x + .5 * other.width;
        if (y < foot_y - .65 * board.height || y > foot_y + .20 * board.height)
            continue;
        double x_left = left.x + (far_left.x - left.x) * (left.y - y) / (left.y - top);
        double x_right = right.x + (far_right.x - right.x) * (right.y - y) / (right.y - top);
        if (x > x_left && x < x_right) {
            result.left_clear = result.right_clear = false;
            result.left_blocked = result.right_blocked = true;
        }
    }
    result.heading = heading;
    result.divider_x = divider;
    result.left_x = divider - config.lane_width * .5;
    result.right_x = divider + config.lane_width * .5;
    result.entry_y = entry->intercept;
    result.end_y = result.entry_y + config.parking_length / std::cos(heading);
    result.confidence = .60;
    return result;
}

Bay contrastEntrance(const cv::Mat &image, const cv::Mat &hsv, const GroundMap &ground,
                     const Config &config) {
    double scale = double(image.cols) / ground.imageSize().width;
    auto kernel = [&](int nominal, int shape) {
        int radius = std::max(1, cvRound((nominal / 2) * scale));
        return cv::getStructuringElement(shape, {2 * radius + 1, 2 * radius + 1});
    };
    cv::Mat blue;
    cv::inRange(hsv, cv::Scalar(102, 85, 45), cv::Scalar(135, 255, 255), blue);
    cv::morphologyEx(blue, blue, cv::MORPH_OPEN, kernel(5, cv::MORPH_RECT));
    std::vector<std::vector<cv::Point>> shapes;
    cv::findContours(blue, shapes, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> boards;
    for (const auto &shape : shapes) {
        auto r = cv::boundingRect(shape);
        double area = cv::contourArea(shape);
        if (area > .004 * image.total() && area > .65 * r.area() &&
            r.width > .65 * r.height && r.width < 2.8 * r.height && r.y > 0 &&
            r.br().y > .45 * image.rows && r.br().y < .98 * image.rows)
            boards.push_back(r);
    }
    if (boards.empty())
        return {};
    cv::Mat green, blue_channel, local, warm, tape;
    cv::extractChannel(image, green, 1);
    cv::extractChannel(image, blue_channel, 0);
    cv::morphologyEx(green, local, cv::MORPH_TOPHAT, kernel(17, cv::MORPH_ELLIPSE));
    cv::inRange(hsv, cv::Scalar(8, 8, 55), cv::Scalar(65, 255, 255), warm);
    tape = warm & (green - blue_channel > 5) & (local >= 9);
    cv::morphologyEx(tape, tape, cv::MORPH_CLOSE, kernel(3, cv::MORPH_RECT));
    for (auto board : boards) {
        // Try several depths: a reflected vertical streak may join a side
        // farther from the entrance while the floor near the board is visible.
        for (double depth : {.29, .22, .16, .39}) {
            int top = int(board.br().y - board.height * depth);
            int bottom = std::min(image.rows, int(board.br().y + .2 * board.height));
            cv::Mat floor_mask = ~tape.rowRange(top, bottom);
            std::vector<std::vector<cv::Point>> floors;
            cv::findContours(floor_mask, floors, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, {0, top});
            for (const auto &floor : floors) {
                auto r = cv::boundingRect(floor);
                if (r.y != top || r.x < 2 || r.br().x >= image.cols - 1 || r.br().y + 1 >= bottom ||
                    r.width < .1 * image.cols || r.width > .76 * image.cols || r.height < .06 * board.height)
                    continue;
                Bay bay = entranceFromFloor(image, hsv, blue, tape, floor, board, boards, top, ground, config);
                if (bay.valid)
                    return bay;
            }
        }
    }
    return {};
}


double surfaceFraction(const cv::Mat &hsv, const GroundMap &g, const Bay &bay, bool left, double tape,
                       double lookahead) {
    int measured = 0, clear = 0;
    double a = left ? bay.left_x : bay.divider_x + tape * .5;
    double b = left ? bay.divider_x - tape * .5 : bay.right_x;
    if (b - a < .15)
        return 0;
    for (int iy = 1; iy <= 8; ++iy)
        for (int ix = 1; ix <= 5; ++ix) {
            double y = bay.entry_y + (bay.end_y - bay.entry_y) * iy / 9;
            double x = a + (b - a) * ix / 6 + std::tan(bay.heading) * (y - lookahead);
            auto p = g.unproject({x, y}, hsv.size());
            if (!p)
                continue;
            int px = int(std::round(p->x)), py = int(std::round(p->y));
            if (px < 0 || py < 0 || px >= hsv.cols || py >= hsv.rows)
                continue;
            auto v = hsv.at<cv::Vec3b>(py, px);
            ++measured;
            // Overexposed/desaturated regions are unknown, never invented free floor.
            bool yellow = v[0] >= 10 && v[0] <= 45 && v[1] >= 8 && v[2] >= 60;
            bool blue = v[0] >= 95 && v[0] <= 135 && v[1] >= 65;
            if (v[2] > 25 && v[2] < 245 && v[1] > 25 && !yellow && !blue)
                ++clear;
        }
    return measured >= 8 ? double(clear) / measured : 0;
}
} // namespace
ParkingDetector::ParkingDetector(Config c, const GroundMap &g) : config_(c), ground_(g) {}
void ParkingDetector::clearTracking() {
    tracked_ = {};
    candidate_ = {};
    candidate_frames_ = 0;
    motion_updated_ = false;
}
void ParkingDetector::reset() {
    clearTracking();
    motion_lost_ = false;
}
void ParkingDetector::invalidateMotion() {
    if (!motion_lost_)
        clearTracking();
    motion_lost_ = true;
    motion_updated_ = false;
}
bool ParkingDetector::predictMotion(double forward, double lateral, double yaw) {
    if (!std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(yaw) || std::abs(forward) > .5 ||
        std::abs(lateral) > .3 || std::abs(yaw) > .35) {
        invalidateMotion();
        return false;
    }
    auto transform = [&](Bay &b) {
        if (!b.valid)
            return;
        const double c = std::cos(yaw), s = std::sin(yaw);
        auto point = [&](Point p) {
            p.x -= lateral;
            p.y -= forward;
            return Point{c * p.x - s * p.y, s * p.x + c * p.y};
        };
        // Transverse y values are y-intercepts at x=0, not distances along the side.
        Point entry = point({0, b.entry_y});
        Point end = point({0, b.end_y});
        b.heading -= yaw;
        auto shift = [&](double x) {
            Point p = point({x, config_.lookahead});
            return p.x + std::tan(b.heading) * (config_.lookahead - p.y);
        };
        b.left_x = shift(b.left_x);
        b.divider_x = shift(b.divider_x);
        b.right_x = shift(b.right_x);
        b.entry_y = entry.y + std::tan(b.heading) * entry.x;
        b.end_y = end.y + std::tan(b.heading) * end.x;
        b.entry_visible = false;
        b.end_visible = false;
        b.current_geometry = false;
    };
    transform(tracked_);
    transform(candidate_);
    motion_updated_ = true;
    motion_lost_ = false;
    return true;
}
Bay ParkingDetector::detect(const cv::Mat &image, const Lane &lane) {
    if (image.empty() || image.type() != CV_8UC3) {
        invalidateMotion();
        candidate_ = {};
        candidate_frames_ = 0;
        return {};
    }
    cv::Mat hsv, yellow, white, bands, edges, yellow_support;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // Sunlight desaturates real yellow tape. A warm chromatic hue is a candidate,
    // but only connected bay geometry can promote low-saturation candidates.
    cv::inRange(hsv, cv::Scalar(10, 8, 60), cv::Scalar(45, 255, 255), yellow);
    cv::morphologyEx(yellow, yellow, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 55, 255), white);
    cv::bitwise_or(yellow, white, bands);
    cv::dilate(yellow, yellow_support, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::Canny(bands, edges, 40, 100);
    std::vector<cv::Vec4i> raw;
    cv::HoughLinesP(edges, raw, 1, CV_PI / 360, 12, 12, 6);
    std::vector<Line> longitudinal, transverse, image_transverse;
    double heading = tracked_.valid ? tracked_.heading : (lane.valid ? lane.heading : 0);
    for (auto seg : raw) {
        double dx = seg[2] - seg[0], dy = seg[3] - seg[1];
        int yellow_pixels = 0;
        for (int n = 0; n <= 40; ++n) {
            int px = std::clamp(int(std::round(seg[0] + dx * n / 40.)), 0, image.cols - 1);
            int py = std::clamp(int(std::round(seg[1] + dy * n / 40.)), 0, image.rows - 1);
            if (yellow_support.at<unsigned char>(py, px))
                ++yellow_pixels;
        }
        bool is_yellow = yellow_pixels >= 22;
        if (is_yellow && std::abs(dx) > 10 && std::abs(dy / dx) < .5)
            image_transverse.push_back({(seg[1] + seg[3]) * .5, dy / dx, length(seg), seg, false, true});
        std::vector<cv::Point2f> mapped;
        for (int n = 0; n <= 40; ++n) {
            double f = n / 40.;
            auto p = ground_.project({seg[0] + dx * f, seg[1] + dy * f}, image.size());
            if (p)
                mapped.emplace_back(float(p->x), float(p->y));
        }
        if (mapped.size() < 3 || cv::norm(mapped.back() - mapped.front()) < .08)
            continue;
        cv::Vec4f fitted;
        cv::fitLine(mapped, fitted, cv::DIST_L2, 0, .005, .005);
        if (std::abs(fitted[1]) > .1) {
            double slope = fitted[0] / fitted[1];
            if (std::abs(std::atan(slope) - heading) < .30)
                longitudinal.push_back({fitted[2] + slope * (config_.lookahead - fitted[3]), slope,
                                        length(seg), seg, true, is_yellow});
        }
        if (is_yellow && std::abs(fitted[0]) > .1) {
            double slope = fitted[1] / fitted[0];
            if (std::abs(std::atan(slope) + heading) < .30)
                transverse.push_back({fitted[3] - slope * fitted[2], slope, length(seg), seg, true});
        }
    }
    longitudinal = merge(longitudinal, config_.tape_width + .02);
    transverse = merge(transverse, config_.tape_width + .02);
    image_transverse = merge(image_transverse, std::max(3., image.rows * .012));
    bool entrance_floor = false;
    Bay observed;
    if (!tracked_.valid) {
        // White track boundaries (or yellow perimeter tape) plus the actual yellow
        // divider identify the two half-bays. Outer boundaries need not be yellow.
        // Only measured in-domain parts determine metres; full image lines establish identity.
        for (std::size_t i = 0; i + 2 < longitudinal.size() && !observed.valid; ++i) {
            auto l = longitudinal[i], m = longitudinal[i + 1], r = longitudinal[i + 2];
            if (!m.yellow)
                continue;
            double width = r.intercept - l.intercept;
            if (std::abs(width - config_.lane_width) > .25 ||
                std::abs((m.intercept - l.intercept) - (r.intercept - m.intercept)) > .20)
                continue;
            if (lane.valid && lane.metric &&
                (std::abs(l.intercept + config_.tape_width * .5 - lane.left_x) > .25 ||
                 std::abs(r.intercept - config_.tape_width * .5 - lane.right_x) > .25))
                continue;
            std::vector<Line> horizontal;
            for (auto h : transverse)
                if (spansBay(h, l, r))
                    horizontal.push_back(h);
            if (horizontal.empty())
                continue;
            double near_y = 0, far_y = 0;
            bool entry_metric = false, end_metric = false, found = false;
            for (auto a : horizontal)
                for (auto b : horizontal) {
                    if (std::abs((b.intercept - a.intercept) -
                                 (config_.parking_length + config_.tape_width)) < .18) {
                        near_y = a.intercept + config_.tape_width * .5;
                        far_y = b.intercept - config_.tape_width * .5;
                        found = true;
                        entry_metric = end_metric = true;
                    }
                }
            if (!found) {
                // One measured transverse band is enough when the other is visibly connected
                // to these same side bands. The other endpoint remains explicitly inferred.
                for (auto a : horizontal) {
                    double ay = pixelY(a, image.cols * .5);
                    for (auto b : image_transverse) {
                        if (!spansBay(b, l, r))
                            continue;
                        double by = pixelY(b, image.cols * .5);
                        if (std::abs(ay - by) < image.rows * .035)
                            continue;
                        if (by < ay) {
                            near_y = a.intercept + config_.tape_width * .5;
                            far_y = near_y + config_.parking_length;
                            entry_metric = true;
                        } else {
                            far_y = a.intercept - config_.tape_width * .5;
                            near_y = far_y - config_.parking_length;
                            end_metric = true;
                        }
                        // A measured far end anchors the bay even when the entrance line has
                        // already passed behind the car; the entrance row is then inferred.
                        if (far_y > near_y && (near_y > 0 || (end_metric && far_y > 0))) {
                            found = true;
                            break;
                        }
                    }
                    if (found)
                        break;
                }
            }
            // Rebuild from the measured endpoint when the entrance is behind the car
            // (near_y < 0) instead of waiting forever for a no-longer-visible entrance.
            if (!found || far_y <= near_y || far_y <= 0 ||
                (near_y < 0 && !(end_metric && far_y > 0)))
                continue;
            observed.valid = true;
            observed.entry_visible = entry_metric;
            observed.end_visible = end_metric;
            observed.left_x = l.intercept + config_.tape_width * .5;
            observed.divider_x = m.intercept;
            observed.right_x = r.intercept - config_.tape_width * .5;
            observed.entry_y = near_y;
            observed.end_y = far_y;
            observed.heading = std::atan((l.slope + m.slope + r.slope) / 3);
            observed.confidence = entry_metric && end_metric ? .9 : .65;
        }
        if (!observed.valid)
            observed = visibleHalfBay(image, hsv, yellow, ground_, config_);
        if (!observed.valid) {
            observed = contrastEntrance(image, hsv, ground_, config_);
            entrance_floor = observed.valid;
        }
        if (observed.valid) {
            observed.current_geometry = true;
            bool match = candidate_.valid && !(candidate_.left_clear && observed.right_clear) &&
                         !(candidate_.right_clear && observed.left_clear) &&
                         std::abs(candidate_.entry_y - observed.entry_y) < .15 &&
                         std::abs(candidate_.divider_x - observed.divider_x) < .12 &&
                         std::abs(candidate_.end_y - observed.end_y) < .2;
            candidate_frames_ = match ? candidate_frames_ + 1 : 1;
            candidate_ = observed;
            if (candidate_frames_ >= config_.confirm_frames) {
                tracked_ = observed;
                ++generation_;
                tracked_.generation = generation_;
            }
        } else {
            candidate_frames_ = 0;
            candidate_ = {};
        }
    } else {
        // Preserve entry/end identity; an entry disappearing is not an entry crossing.
        observed = tracked_;
        observed.entry_visible = observed.end_visible = false;
        observed.current_geometry = false;
        int side_matches = 0;
        double correction = 0;
        double slope_sum = 0, minimum_heading = 10, maximum_heading = -10;
        for (auto expected : {tracked_.left_x - config_.tape_width * .5, tracked_.divider_x,
                              tracked_.right_x + config_.tape_width * .5}) {
            double best = .12, delta = 0, current_slope = 0;
            for (auto l : longitudinal)
                if (std::abs(l.intercept - expected) < best) {
                    best = std::abs(l.intercept - expected);
                    delta = l.intercept - expected;
                    current_slope = l.slope;
                }
            if (best < .12) {
                ++side_matches;
                correction += delta;
                slope_sum += current_slope;
                minimum_heading = std::min(minimum_heading, std::atan(current_slope));
                maximum_heading = std::max(maximum_heading, std::atan(current_slope));
            }
        }
        if (side_matches >= 1) {
            correction /= side_matches;
            observed.left_x += correction;
            observed.divider_x += correction;
            observed.right_x += correction;
        }
        const bool current_sides = side_matches >= 2 && maximum_heading - minimum_heading < .12;
        if (current_sides)
            observed.heading = std::atan(slope_sum / side_matches);
        for (auto t : transverse) {
            const double lo = std::min(t.pixels[0], t.pixels[2]), hi = std::max(t.pixels[0], t.pixels[2]);
            auto center = ground_.unproject({observed.divider_x, t.intercept}, image.size());
            if (!center || center->x < lo - 5 || center->x > hi + 5 || side_matches < 1)
                continue;
            if (std::abs(t.intercept + config_.tape_width * .5 - tracked_.entry_y) < .20) {
                observed.entry_y = t.intercept + config_.tape_width * .5;
                observed.entry_visible = true;
            } else if (std::abs(t.intercept - config_.tape_width * .5 - tracked_.end_y) < .20) {
                observed.end_y = t.intercept - config_.tape_width * .5;
                observed.end_visible = true;
            }
        }
        Bay visible_half = visibleHalfBay(image, hsv, yellow, ground_, config_);
        bool floor_candidate = false;
        if (!visible_half.valid && !observed.end_visible) {
            visible_half = contrastEntrance(image, hsv, ground_, config_);
            floor_candidate = visible_half.valid;
        }
        bool current_half = false;
        if (visible_half.valid && std::abs(visible_half.entry_y - tracked_.entry_y) < .22 &&
            std::abs(visible_half.end_y - tracked_.end_y) < .25 &&
            std::abs(visible_half.divider_x - tracked_.divider_x) < .15) {
            // Re-observing the same still-visible half-bay maintains identity even when
            // the opposite outer edge is hidden by the required blue board.
            observed.entry_y = visible_half.entry_y;
            observed.end_y = visible_half.end_y;
            observed.entry_visible = visible_half.entry_visible;
            observed.end_visible = visible_half.end_visible;
            observed.left_x = visible_half.left_x;
            observed.divider_x = visible_half.divider_x;
            observed.right_x = visible_half.right_x;
            observed.heading = visible_half.heading;
            if (floor_candidate) {
                observed.left_clear = visible_half.left_clear;
                observed.right_clear = visible_half.right_clear;
                observed.left_blocked = visible_half.left_blocked;
                observed.right_blocked = visible_half.right_blocked;
                entrance_floor = true;
            }
            current_half = true;
        }
        bool measured = observed.entry_visible || observed.end_visible;
        observed.current_geometry = current_half || (measured && current_sides);
        if (measured)
            observed.confidence = std::min(.95, tracked_.confidence + .08);
        else
            observed.confidence = std::max(0., tracked_.confidence - (motion_updated_ ? .025 : .08));
        tracked_ = observed;
    }
    motion_updated_ = false;
    // An expired track must not keep the detector inside the narrow old-position
    // association branch forever. Rebuild from current image evidence next time.
    if (tracked_.valid && tracked_.confidence < .25) {
        clearTracking();
        return {};
    }
    if (!tracked_.valid) {
        observed.valid = false;
        observed.left_clear = observed.right_clear = false;
        return observed;
    }
    if (entrance_floor)
        return tracked_;
    // Board assignment is based on its ground-contact position inside this bay.
    cv::Mat blue;
    cv::inRange(hsv, cv::Scalar(95, 65, 35), cv::Scalar(135, 255, 255), blue);
    std::vector<std::vector<cv::Point>> boards;
    cv::findContours(blue, boards, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    bool left_block = false, right_block = false;
    for (const auto &contour : boards) {
        if (cv::contourArea(contour) < 40)
            continue;
        auto r = cv::boundingRect(contour);
        for (auto p : boardFeet(ground_, r, image.size())) {
            if (p.y < tracked_.entry_y - .30 || p.y > tracked_.end_y + .25)
                continue;
            double x = p.x - std::tan(tracked_.heading) * (p.y - config_.lookahead);
            if (x > tracked_.left_x - .05 && x < tracked_.divider_x - config_.tape_width * .5)
                left_block = true;
            if (x > tracked_.divider_x + config_.tape_width * .5 && x < tracked_.right_x + .05)
                right_block = true;
        }
    }
    if (left_block || right_block) {
        tracked_.left_blocked = left_block;
        tracked_.right_blocked = right_block;
        tracked_.left_clear =
            !left_block && right_block &&
            surfaceFraction(hsv, ground_, tracked_, true, config_.tape_width, config_.lookahead) > .6;
        tracked_.right_clear =
            !right_block && left_block &&
            surfaceFraction(hsv, ground_, tracked_, false, config_.tape_width, config_.lookahead) > .6;
    }
    Bay result = tracked_;
    result.generation = generation_;
    result.valid = result.confidence >= .25;
    return result;
}
} // namespace smartcar
