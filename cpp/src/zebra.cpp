#include "smartcar/zebra.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace smartcar {
namespace {
double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const auto mid = values.size() / 2;
    return values.size() % 2 ? values[mid] : (values[mid - 1] + values[mid]) * .5;
}

double centerX(const cv::Rect &r) { return r.x + r.width * .5; }
double centerY(const cv::Rect &r) { return r.y + r.height * .5; }

// A white patch must differ from its immediate surroundings. A brightness-only
// threshold merges white paper and the sunlit pink track into one component.
bool localContrast(const std::vector<cv::Point> &contour, const cv::Rect &box,
                   const cv::Mat &gray, const cv::Mat &saturation) {
    const int radius = std::clamp(std::min(box.width, box.height) / 3, 3, 10);
    const cv::Rect region = cv::Rect(box.x - radius, box.y - radius,
                                    box.width + 2 * radius, box.height + 2 * radius) &
                            cv::Rect(0, 0, gray.cols, gray.rows);
    cv::Mat inside = cv::Mat::zeros(region.size(), CV_8UC1);
    std::vector<cv::Point> shifted;
    shifted.reserve(contour.size());
    for (auto p : contour)
        shifted.push_back(p - region.tl());
    cv::fillPoly(inside, std::vector<std::vector<cv::Point>>{shifted}, cv::Scalar(255));
    cv::Mat dilated, ring;
    cv::dilate(inside, dilated, cv::getStructuringElement(cv::MORPH_RECT,
                                                        {2 * radius + 1, 2 * radius + 1}));
    cv::subtract(dilated, inside, ring);
    if (cv::countNonZero(ring) < 8)
        return false;
    const double value_delta = cv::mean(gray(region), inside)[0] - cv::mean(gray(region), ring)[0];
    const double inner_saturation = cv::mean(saturation(region), inside)[0];
    const double saturation_delta = cv::mean(saturation(region), ring)[0] - inner_saturation;
    return value_delta >= 12.0 && (inner_saturation <= 55.0 || saturation_delta >= 12.0);
}

bool rowShape(const std::vector<cv::Rect> &row, int image_width) {
    if (row.size() < 3)
        return false;
    int left = row.front().x, right = row.front().x + row.front().width;
    double mean_x = 0, mean_y = 0;
    std::vector<double> heights;
    for (const auto &r : row) {
        left = std::min(left, r.x);
        right = std::max(right, r.x + r.width);
        mean_x += centerX(r);
        mean_y += centerY(r);
        heights.push_back(r.height);
    }
    if (right - left < image_width * .18)
        return false;
    mean_x /= row.size();
    mean_y /= row.size();
    double xx = 0, xy = 0;
    for (const auto &r : row) {
        const double dx = centerX(r) - mean_x;
        xx += dx * dx;
        xy += dx * (centerY(r) - mean_y);
    }
    if (xx <= 0)
        return false;
    const double slope = xy / xx;
    if (std::abs(slope) > .35)
        return false;
    const double residual_limit = std::max(4.0, median(heights) * .28);
    for (const auto &r : row)
        if (std::abs(centerY(r) - mean_y - slope * (centerX(r) - mean_x)) > residual_limit)
            return false;
    for (std::size_t i = 1; i < row.size(); ++i) {
        const double width = (row[i - 1].width + row[i].width) * .5;
        const double gap = centerX(row[i]) - centerX(row[i - 1]);
        if (gap < .45 * width || gap > 4.0 * width)
            return false;
    }
    return true;
}

void resolveNearEdge(ZebraImageResult &result, const cv::Size &image_size) {
    std::vector<cv::Rect> eligible;
    std::vector<double> heights;
    for (const auto &r : result.stripes) {
        // Side clipping can also remove the real corner of a slanted strip.
        if (r.x <= 0 || r.x + r.width >= image_size.width - 1 ||
            r.y + r.height >= image_size.height - 1)
            continue;
        eligible.push_back(r);
        heights.push_back(r.height);
    }
    result.near_edge_px = std::numeric_limits<double>::quiet_NaN();
    result.reason = "partial_or_occluded_paper_row_presence_only";
    if (eligible.size() < 3)
        return;

    // Both ends of one physical row should follow consistent image lines.
    // A hood leaves a plausible white quadrilateral, but its visible bottom
    // is not the paper's near edge. Fit the far and near edges together and
    // require at least three complete strips to support the same geometry.
    const double tolerance = std::clamp(median(heights) * .12, 2.5, 8.0);
    std::vector<double> best_bottoms;
    double best_error = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < eligible.size(); ++i) {
        for (std::size_t j = i + 1; j < eligible.size(); ++j) {
            const auto &a = eligible[i];
            const auto &b = eligible[j];
            const double dx = centerX(b) - centerX(a);
            if (std::abs(dx) < 1)
                continue;
            const double far_slope = (b.y - a.y) / dx;
            const double near_a = a.y + a.height - 1;
            const double near_slope = (b.y + b.height - 1 - near_a) / dx;
            if (std::abs(far_slope) > .35 || std::abs(near_slope) > .45 ||
                std::abs(near_slope - far_slope) > .25)
                continue;
            std::vector<double> bottoms;
            double error = 0;
            for (const auto &r : eligible) {
                const double offset = centerX(r) - centerX(a);
                const double far_error = std::abs(r.y - a.y - far_slope * offset);
                const double bottom = r.y + r.height - 1;
                const double near_error = std::abs(bottom - near_a - near_slope * offset);
                if (far_error <= tolerance && near_error <= tolerance) {
                    bottoms.push_back(bottom);
                    error += far_error + near_error;
                }
            }
            if (bottoms.size() > best_bottoms.size() ||
                (bottoms.size() == best_bottoms.size() && error < best_error)) {
                best_bottoms = std::move(bottoms);
                best_error = error;
            }
        }
    }
    if (best_bottoms.size() >= 3) {
        result.near_edge_px = median(best_bottoms);
        result.reason = best_bottoms.size() == result.stripes.size()
                            ? "image_paper_row"
                            : "image_paper_row_partial_edges_excluded";
    }
}
} // namespace

ZebraImageResult detectZebraImage(const cv::Mat &bgr) {
    ZebraImageResult result;
    if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols < 64 || bgr.rows < 48) {
        result.reason = "invalid_image";
        return result;
    }
    cv::Mat hsv, gray, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    // Keep moderately illuminated white paper; separate it from pastel road
    // by chroma and local contrast rather than raising the brightness floor.
    cv::inRange(hsv, cv::Scalar(0, 0, 105), cv::Scalar(179, 70, 255), mask);
    const int top = int(std::lround(bgr.rows * .35));
    mask.rowRange(0, top).setTo(0);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> patches;
    const double image_area = double(bgr.cols) * bgr.rows;
    for (const auto &contour : contours) {
        const auto box = cv::boundingRect(contour);
        const double area = cv::contourArea(contour);
        const double aspect = double(box.width) / box.height;
        const auto rotated_size = cv::minAreaRect(contour).size;
        const double rotated_area = rotated_size.area();
        const double short_side = std::min(rotated_size.width, rotated_size.height);
        const double long_side = std::max(rotated_size.width, rotated_size.height);
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, .035 * cv::arcLength(contour, true), true);
        if (box.width < 4 || box.height < 3 || box.width > bgr.cols * .30 ||
            box.height > bgr.rows * .38 || box.y <= top ||
            aspect < .22 || aspect > 8.0 || area < std::max(12.0, image_area * .00012) ||
            area > image_area * .07 || area < box.area() * .30 ||
            rotated_area <= 0 || area < rotated_area * .55 ||
            // Thin pieces of longitudinal/transverse lane paint can line up
            // after compression. Paper must retain a substantial short side
            // and a compact four-sided shape under perspective.
            short_side < std::max(3.0, bgr.rows * .01) || long_side > short_side * 4.0 ||
            polygon.size() < 4 || polygon.size() > 6)
            continue;
        if (localContrast(contour, box, gray, channels[1]))
            patches.push_back(box);
    }
    if (patches.size() > 80) {
        result.reason = "ambiguous_white_texture";
        return result;
    }
    double best_score = -1;
    for (const auto &base : patches) {
        std::vector<cv::Rect> group;
        for (const auto &patch : patches) {
            const int tall = std::max(base.height, patch.height);
            const int short_height = std::min(base.height, patch.height);
            if (double(tall) / short_height <= 3.25 &&
                std::abs(centerY(base) - centerY(patch)) <= tall * .65)
                group.push_back(patch);
        }
        std::sort(group.begin(), group.end(), [](const cv::Rect &a, const cv::Rect &b) {
            return centerX(a) < centerX(b);
        });
        // A separate bright object must not invalidate an otherwise consistent
        // run of strips. Check contiguous groups, preserving the gap checks.
        for (std::size_t begin = 0; begin + 3 <= group.size(); ++begin) {
            const auto maximum_end = std::min(group.size(), begin + 16);
            for (std::size_t end = begin + 3; end <= maximum_end; ++end) {
                std::vector<cv::Rect> row(group.begin() + begin, group.begin() + end);
                if (!rowShape(row, bgr.cols))
                    continue;
                std::vector<double> bottoms;
                for (const auto &r : row)
                    bottoms.push_back(r.y + r.height - 1);
                const double near_edge = median(bottoms);
                const double score = row.size() * bgr.rows + near_edge;
                if (score <= best_score)
                    continue;
                best_score = score;
                result.candidate = true;
                result.stripes = std::move(row);
                result.near_edge_px = near_edge;
                result.reason = "image_paper_row";
            }
        }
    }
    if (result.candidate)
        resolveNearEdge(result, bgr.size());
    return result;
}
} // namespace smartcar
