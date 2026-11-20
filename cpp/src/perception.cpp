#include "smartcar/perception.hpp"
#include "smartcar/zebra.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace smartcar {
namespace {
double distance(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
double median(std::vector<double> v) {
    if (v.empty())
        return 0;
    auto middle = v.begin() + v.size() / 2;
    std::nth_element(v.begin(), middle, v.end());
    return *middle;
}
struct Tape {
    std::vector<cv::Point2f> pts;
    double x = 0, heading = 0, length = 0, width = 0, residual = 0, min_y = 0, max_y = 0;
};
bool describeTape(const std::vector<cv::Point> &contour, const GroundMap &map, cv::Size size, double ref,
                  Tape &tape) {
    if (contour.size() < 6)
        return false;
    int total = 0;
    for (const auto &p : contour) {
        ++total;
        const auto q = map.project(cv::Point2d(p), size);
        if (q)
            tape.pts.emplace_back(float(q->x), float(q->y));
    }
    // An outline cut by the calibration boundary cannot supply a measured width.
    if (tape.pts.size() < 8 || tape.pts.size() < size_t(total * 0.55))
        return false;
    cv::Vec4f fit;
    cv::fitLine(tape.pts, fit, cv::DIST_HUBER, 0, 0.005, 0.005);
    double vx = fit[0], vy = fit[1];
    if (vy < 0) {
        vx = -vx;
        vy = -vy;
    }
    if (vy < 0.7)
        return false;
    const double slope = vx / vy;
    tape.heading = std::atan(slope);
    tape.x = fit[2] + (ref - fit[3]) * slope;
    tape.min_y = std::numeric_limits<double>::max();
    tape.max_y = -tape.min_y;
    std::vector<double> widths;
    double lo = 1e10, hi = -1e10;
    for (const auto &p : tape.pts) {
        const double n = (p.x - fit[2]) * vy - (p.y - fit[3]) * vx;
        lo = std::min(lo, n);
        hi = std::max(hi, n);
        widths.push_back(std::abs(n));
        tape.min_y = std::min(tape.min_y, double(p.y));
        tape.max_y = std::max(tape.max_y, double(p.y));
    }
    tape.width = hi - lo;
    tape.length = (tape.max_y - tape.min_y) / vy;
    tape.residual = median(widths);
    return std::isfinite(tape.x) && tape.width >= 0.012 && tape.width <= 0.16;
}
cv::Mat calibratedMask(const GroundMap &map, cv::Size size) {
    cv::Mat mask(size, CV_8U, cv::Scalar(0));
    const cv::Size reference = map.imageSize();
    const double scale = double(size.width) / reference.width;
    if (std::abs(scale - double(size.height) / reference.height) > 1e-6)
        return mask;
    std::vector<cv::Point> polygon;
    for (const auto &p : map.validPolygon())
        polygon.emplace_back(int(std::lround((p.x + .5) * scale - .5)),
                             int(std::lround((p.y + .5) * scale - .5)));
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
    // Rounding can add a boundary pixel; retain only the conservative polygon interior.
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    return mask;
}
Lane imageLane(const cv::Mat &white, cv::Size size, const Config &config,
               const GroundMap &map, PerceptionDiagnostics &diagnostics) {
    struct Edge {
        double x, dxdy, span, min_y, max_y;
    };
    std::vector<Edge> edges;
    // Skyline/railings can connect both lane stripes into one giant contour.
    // Work on the ground-facing image band; metric projection still uses only
    // the original measured polygon, unchanged below.
    cv::Mat ground_white = white.clone();
    ground_white.rowRange(0, int(size.height * .40)).setTo(0);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ground_white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    const auto reference = map.unproject({0, config.lookahead}, size);
    const double ref_y = reference ? reference->y : size.height * 0.70;
    for (const auto &points : contours) {
        if (points.size() < 12 || cv::contourArea(points) < 20)
            continue;
        cv::Vec4f f;
        cv::fitLine(points, f, cv::DIST_HUBER, 0, 0.1, 0.01);
        if (std::abs(f[1]) < 0.30)
            continue;
        double lo = 1e10, hi = -1e10, miny = 1e10, maxy = -1e10;
        for (const auto &p : points) {
            const double normal = (p.x - f[2]) * f[1] - (p.y - f[3]) * f[0];
            lo = std::min(lo, normal);
            hi = std::max(hi, normal);
            miny = std::min(miny, double(p.y));
            maxy = std::max(maxy, double(p.y));
        }
        if (hi - lo > size.width * 0.075 || hi - lo < 2 || maxy - miny < size.height * 0.12)
            continue;
        edges.push_back({f[2] + (ref_y - f[3]) * f[0] / f[1], f[0] / f[1], maxy - miny, miny, maxy});
    }
    // A transverse marking/highlight may join a genuine long stripe. Split
    // that junction using line segments, then require narrow white support
    // along the segment. Edges of a broad glare patch cannot pass this test.
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(ground_white, segments, 1, CV_PI / 180, std::max(20, size.width / 16),
                   size.height * .18, size.height * .035);
    for (const auto &s : segments) {
        const double dx = s[2] - s[0], dy = s[3] - s[1], length = std::hypot(dx, dy);
        if (length < 1 || std::abs(dy) < size.height * .12 || std::abs(dy) / length < .30)
            continue;
        const double nx = -dy / length, ny = dx / length;
        std::vector<double> widths;
        const int limit = std::max(4, int(size.width * .075));
        for (int k = 1; k < 20; ++k) {
            const double px = s[0] + dx * k / 20., py = s[1] + dy * k / 20.;
            auto is_white = [&](int offset) {
                int x = int(std::lround(px + nx * offset)), y = int(std::lround(py + ny * offset));
                return x >= 0 && x < size.width && y >= 0 && y < size.height &&
                       ground_white.at<uchar>(y, x) != 0;
            };
            if (!is_white(0))
                continue;
            int a = 0, b = 0;
            while (a < limit && is_white(-a - 1)) ++a;
            while (b < limit && is_white(b + 1)) ++b;
            widths.push_back(a == limit || b == limit ? limit * 2. : a + b + 1.);
        }
        if (widths.size() < 14 || median(widths) < 2 || median(widths) > limit)
            continue;
        Edge candidate{s[0] + (ref_y - s[1]) * dx / dy, dx / dy, std::abs(dy),
                       double(std::min(s[1], s[3])), double(std::max(s[1], s[3]))};
        bool duplicate = false;
        for (auto &edge : edges) {
            const double y = (std::max(edge.min_y, candidate.min_y) +
                              std::min(edge.max_y, candidate.max_y)) / 2;
            if (std::abs((edge.x + (y - ref_y) * edge.dxdy) -
                         (candidate.x + (y - ref_y) * candidate.dxdy)) < size.width * .02 &&
                std::abs(edge.dxdy - candidate.dxdy) < .15) {
                if (candidate.span > edge.span)
                    edge = candidate;
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            edges.push_back(candidate);
    }
    diagnostics.image_lane_candidates = int(edges.size());
    // Lane and cone extension share one bounded projection routine. This does
    // not change GroundMap's measured domain or promote image estimates.
    std::vector<cv::Point2f> anchors_m;
    for (auto p : map.validPolygon()) {
        auto q = map.project(p, map.imageSize());
        if (!q)
            continue;
        anchors_m.emplace_back(float(q->x), float(q->y));
    }
    struct ProjectedEdge {
        double x = 0, heading = 0, error = 0;
        bool valid = false;
    };
    auto project_edge = [&](const Edge &edge) {
        ProjectedEdge result;
        if (anchors_m.size() < 3)
            return result;
        std::vector<cv::Point2f> points;
        double extrapolation = 0;
        for (int i = 0; i <= 40; ++i) {
            const double py = edge.min_y + (edge.max_y - edge.min_y) * i / 40.0;
            const double px = edge.x + (py - ref_y) * edge.dxdy;
            const auto projected = map.projectModel({px, py}, size);
            if (!projected)
                continue;
            const cv::Point2f p(float(projected->x), float(projected->y));
            const double outside = std::max(0.0, -cv::pointPolygonTest(anchors_m, p, true));
            extrapolation = std::max(extrapolation, outside);
            points.push_back(p);
        }
        if (points.size() < 8)
            return result;
        cv::Vec4f fit;
        cv::fitLine(points, fit, cv::DIST_HUBER, 0, .005, .005);
        if (std::abs(fit[1]) < .75)
            return result;
        const double slope = fit[0] / fit[1];
        const double x = fit[2] + (config.lookahead - fit[3]) * slope;
        const double reference_gap = std::max(
            0.0, -cv::pointPolygonTest(anchors_m, cv::Point2f(float(x), float(config.lookahead)), true));
        if (reference_gap > .55)
            return result;
        double miny = 1e10, maxy = -1e10, residual = 0;
        for (auto p : points) {
            miny = std::min(miny, double(p.y));
            maxy = std::max(maxy, double(p.y));
            residual = std::max(residual, std::abs(p.x - (fit[2] + (p.y - fit[3]) * slope)));
        }
        if (maxy - miny < .45 || residual > .04)
            return result;
        result = {x, std::atan(slope), .03 + residual + .05 * std::max(extrapolation, reference_gap), true};
        return result;
    };
    Lane out;
    auto converges = [&](const Edge &l, const Edge &r, double width, double y, double first_y) {
        const double upper = width - (r.dxdy - l.dxdy) *
                                      std::min(size.height * .10, (y - first_y) * .5);
        return upper >= width * .3 && upper <= width * 1.08;
    };
    // Compare all pairs at one row. Comparing a short distant pair at its own
    // upper row with a near pair would favour an adjacent lane spuriously.
    // Prefer a fixed observed mid-ground row. A longer unrelated stripe must
    // not push the shared row below the actual current lane's visible ends.
    double common_ref = -1;
    const double desired_ref = std::min(ref_y, size.height * .65);
    for (size_t i = 0; i < edges.size(); ++i)
        for (size_t j = i + 1; j < edges.size(); ++j) {
            const double first = std::max(edges[i].min_y, edges[j].min_y);
            const double last = std::min(edges[i].max_y, edges[j].max_y);
            if (last - first < size.height * .10) continue;
            const double y = std::clamp(desired_ref, first, last);
            const double a = edges[i].x + (y - ref_y) * edges[i].dxdy;
            const double b = edges[j].x + (y - ref_y) * edges[j].dxdy;
            if (std::min(a, b) <= size.width * .5 && std::max(a, b) >= size.width * .5 &&
                std::abs(a - b) >= size.width * .28 && std::abs(a - b) <= size.width * 1.1 &&
                converges(a < b ? edges[i] : edges[j], a < b ? edges[j] : edges[i],
                          std::abs(a - b), y, first))
                common_ref = std::max(common_ref, std::min(y, desired_ref));
        }
    double best = 1e10;
    for (size_t i = 0; i < edges.size(); ++i)
        for (size_t j = i + 1; j < edges.size(); ++j) {
            const double first_y = std::max(edges[i].min_y, edges[j].min_y);
            const double last_y = std::min(edges[i].max_y, edges[j].max_y);
            if (last_y - first_y < size.height * .10 || common_ref < first_y || common_ref > last_y)
                continue;
            // Both sides must actually be visible at the steering reference.
            // A historical near-field reference below the visible stripes must
            // not reject them or extrapolate their pixels past the image sides.
            const double image_ref = common_ref;
            auto image_x = [&](const Edge &e) { return e.x + (image_ref - ref_y) * e.dxdy; };
            const Edge &l = image_x(edges[i]) < image_x(edges[j]) ? edges[i] : edges[j],
                       &r = image_x(edges[i]) < image_x(edges[j]) ? edges[j] : edges[i];
            const double lx = image_x(l), rx = image_x(r), width = rx - lx;
            if (width < size.width * 0.28 || width > size.width * 1.1 || lx > size.width * 0.50 ||
                rx < size.width * 0.50)
                continue;
            // Both genuine tape strips must converge, rather than diverge, toward the horizon.
            if (!converges(l, r, width, image_ref, first_y))
                continue;
            bool intervening=false;
            for (const auto &edge:edges) {
                if (image_ref<edge.min_y || image_ref>edge.max_y) continue;
                const double x=image_x(edge);
                if (x>lx+size.width*.035 && x<rx-size.width*.035) {intervening=true;break;}
            }
            if (intervening) continue;
            const double center = (lx + rx) / 2;
            // The nearest pair bracketing the car wins over a more symmetric
            // but wider pair from adjacent running lanes.
            const double score = width + std::abs(center - size.width * 0.5) * .10;
            if (score >= best)
                continue;
            best = score;
            const double offset = (center - size.width * 0.5) / width * config.lane_width;
            out = {true,
                   true,
                   offset - config.lane_width / 2,
                   offset + config.lane_width / 2,
                   0,
                   0.40,
                   config.lookahead};
            out.metric = false;
            const auto pl = project_edge(l), pr = project_edge(r);
            if (pl.valid && pr.valid && std::abs(pl.heading - pr.heading) < .15) {
                const double heading = (pl.heading + pr.heading) / 2;
                const double model_width = (pr.x - pl.x) * std::cos(heading);
                if (std::abs(model_width - config.lane_width) < .12) {
                    out.left_x = pl.x;
                    out.right_x = pr.x;
                    out.heading = heading;
                    out.model_projected = true;
                    out.lateral_uncertainty_m = std::max(pl.error, pr.error);
                }
            }
        }
    return out;
}
} // namespace

RoadPerception::RoadPerception(const Config &c, const GroundMap &g) : config_(c), ground_(g) {
    if (c.lane_width <= c.body_width || c.confirm_frames < 1)
        throw std::invalid_argument("Invalid road geometry/confirmation count");
}
void RoadPerception::setZebraDistanceProfile(std::shared_ptr<const ZebraDistanceProfile> profile) {
    zebra_profile_ = std::move(profile);
    zebra_streak_ = zebra_image_streak_ = 0;
    previous_zebra_edge_ = -1;
    cached_.zebra = cached_.zebra_detected = false;
}
void RoadPerception::reset() {
    cached_ = Observation{};
    diagnostics_ = {};
    previous_gray_.release();
    previous_feature_mask_.release();
    have_frame_ = false;
    last_frame_id_ = 0;
    previous_time_ = 0;
    last_lane_seen_s_ = -1;
    stopped_since_ = -1;
    stopped_frames_ = lane_streak_ = zebra_streak_ = board_streak_ = 0;
    previous_lane_ = {};
    board_candidate_ = board_reference_ = {};
    previous_zebra_y_ = 0;
    zebra_image_streak_ = 0;
    previous_zebra_edge_ = -1;
}

void RoadPerception::motion(const cv::Mat &gray, const cv::Mat &mask, Observation &out) {
    const double dt = out.time - previous_time_;
    auto fail = [&] {
        stopped_since_ = -1;
        stopped_frames_ = 0;
    };
    if (previous_gray_.empty() || previous_gray_.size() != gray.size() || dt <= 0 || dt > 0.75) {
        fail();
        return;
    }
    std::vector<cv::Point2f> old, newp, back;
    cv::goodFeaturesToTrack(previous_gray_, old, 200, 0.015, 7, previous_feature_mask_, 5);
    diagnostics_.motion_features = int(old.size());
    if (old.size() < 12) {
        fail();
        return;
    }
    std::vector<uchar> ok, back_ok;
    std::vector<float> err, back_err;
    cv::calcOpticalFlowPyrLK(previous_gray_, gray, old, newp, ok, err, {21, 21}, 3);
    cv::calcOpticalFlowPyrLK(gray, previous_gray_, newp, back, back_ok, back_err, {21, 21}, 3);
    std::vector<cv::Point2f> before, after;
    for (size_t i = 0; i < old.size(); ++i) {
        if (!ok[i] || !back_ok[i] || err[i] > 30 || cv::norm(old[i] - back[i]) > 0.7)
            continue;
        const int x = int(std::lround(newp[i].x)), y = int(std::lround(newp[i].y));
        if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows || mask.at<uchar>(y, x) == 0)
            continue;
        const auto a = ground_.project(old[i], gray.size()), b = ground_.project(newp[i], gray.size());
        if (a && b) {
            before.emplace_back(float(a->x), float(a->y));
            after.emplace_back(float(b->x), float(b->y));
        }
    }
    if (before.size() < 10) {
        fail();
        return;
    }
    cv::Mat inliers;
    cv::Mat affine = cv::estimateAffinePartial2D(before, after, inliers, cv::RANSAC, 0.018, 1500, 0.995, 10);
    if (affine.empty()) {
        fail();
        return;
    }
    const double a = affine.at<double>(0, 0), b = affine.at<double>(1, 0);
    const double scale = std::hypot(a, b), angle = std::atan2(b, a);
    const double tx = affine.at<double>(0, 2), ty = affine.at<double>(1, 2);
    std::vector<double> residuals;
    std::vector<cv::Point2f> retained;
    for (size_t i = 0; i < before.size(); ++i)
        if (inliers.at<uchar>(int(i))) {
            retained.push_back(before[i]);
            residuals.push_back(std::hypot(a * before[i].x - b * before[i].y + tx - after[i].x,
                                           b * before[i].x + a * before[i].y + ty - after[i].y));
        }
    diagnostics_.motion_inliers = int(retained.size());
    diagnostics_.motion_residual_m = median(residuals);
    if (retained.size() < 9 || retained.size() < before.size() * 0.65 || std::abs(scale - 1) > 0.04 ||
        diagnostics_.motion_residual_m > 0.012 || std::abs(angle) / dt > 2.5 ||
        std::hypot(tx, ty) / dt > 3.0) {
        fail();
        return;
    }
    float minx = retained[0].x, maxx = minx, miny = retained[0].y, maxy = miny;
    for (const auto &p : retained) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }
    if (maxx - minx < 0.12 || maxy - miny < 0.18) {
        fail();
        return;
    }
    const double c = std::cos(angle), s = std::sin(angle);
    out.motion_valid = true;
    out.lateral_delta = -c * tx - s * ty;
    out.forward_delta = s * tx - c * ty;
    out.yaw_delta = angle;
    if (std::hypot(out.lateral_delta, out.forward_delta) / dt < 0.015 && std::abs(angle) / dt < 0.025) {
        if (stopped_since_ < 0)
            stopped_since_ = out.time;
        ++stopped_frames_;
        out.stopped_confirmed = stopped_frames_ >= 5 && out.time - stopped_since_ >= config_.settle_s;
    } else
        fail();
}

Observation RoadPerception::update(const cv::Mat &bgr, std::uint64_t id, double time, PerceptionTasks tasks) {
    diagnostics_ = {};
    if (have_frame_ && id == last_frame_id_)
        return cached_;
    Observation out;
    out.frame_id = id;
    out.time = time;
    if ((have_frame_ && (id < last_frame_id_ || time <= previous_time_)) || !std::isfinite(time) ||
        bgr.empty() || bgr.type() != CV_8UC3) {
        out.frame_ok = false;
        return out;
    }
    const auto calibrated_size = ground_.imageSize();
    if (std::abs(double(bgr.cols) / calibrated_size.width - double(bgr.rows) / calibrated_size.height) >
        1e-6) {
        out.frame_ok = false;
        return out;
    }
    if (have_frame_ && time - previous_time_ > 0.75) {
        lane_streak_ = zebra_streak_ = board_streak_ = 0;
        zebra_image_streak_ = 0;
        previous_zebra_edge_ = -1;
        previous_lane_ = {};
        board_candidate_ = board_reference_ = {};
    }
    diagnostics_.fresh = true;
    cv::Mat hsv, gray, white, blue;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    // Colour evidence is required: arbitrary Canny edges/shadows are not lane tape.
    cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(179, 80, 255), white);
    cv::inRange(hsv, cv::Scalar(88, 70, 45), cv::Scalar(137, 255, 255), blue);
    if (calibrated_mask_.size() != bgr.size())
        calibrated_mask_ = calibratedMask(ground_, bgr.size());
    const cv::Mat &valid = calibrated_mask_;
    cv::morphologyEx(white, white, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::Mat feature_mask = valid.clone(), expanded_blue;
    cv::dilate(blue, expanded_blue, cv::getStructuringElement(cv::MORPH_RECT, {9, 9}));
    feature_mask.setTo(0, expanded_blue); // vertical blue objects cannot supply ground odometry
    motion(gray, feature_mask, out);
    std::vector<std::vector<cv::Point>> contours;
    if (tasks.lane || tasks.zebra) cv::findContours(white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    diagnostics_.white_components = int(contours.size());
    std::vector<Tape> lanes, zebra_bars;
    if (tasks.lane || tasks.zebra) {
    for (const auto &contour : contours) {
        if (cv::contourArea(contour) < 12)
            continue;
        Tape t;
        if (!describeTape(contour, ground_, bgr.size(), config_.lookahead, t)) {
            ++diagnostics_.rejected_tapes;
            continue;
        }
        if (t.length >= 0.21 && t.length <= 0.40 && t.width >= 0.065 && t.width <= 0.15 &&
            std::abs(t.heading) < 0.55)
            zebra_bars.push_back(t);
        if (t.length < 0.45 || t.residual > 0.075 || t.min_y > config_.lookahead + 0.25 ||
            t.max_y < config_.lookahead - 0.25) {
            ++diagnostics_.rejected_tapes;
            continue;
        }
        // Overexposed broad patches have no measured thin-tape footprint and failed above.
        lanes.push_back(t);
    }
    }
    if (tasks.lane) {
    double best = 1e10;
    for (size_t i = 0; i < lanes.size(); ++i)
        for (size_t j = i + 1; j < lanes.size(); ++j) {
            const Tape &l = lanes[i].x < lanes[j].x ? lanes[i] : lanes[j],
                       &r = lanes[i].x < lanes[j].x ? lanes[j] : lanes[i];
            const double heading = (l.heading + r.heading) / 2;
            const double width = (r.x - l.x) * std::cos(heading);
            if (std::abs(width - config_.lane_width) > 0.14 || std::abs(l.heading - r.heading) > 0.18)
                continue;
            const double center = (l.x + r.x) / 2;
            // A valid pair brackets the rear axle's forward corridor; prefer the prior lane.
            if (l.x > config_.body_width / 2 || r.x < -config_.body_width / 2)
                continue;
            const double score =
                std::abs(width - config_.lane_width) + 0.12 * std::abs(center) +
                (previous_lane_.valid
                     ? 0.5 * std::abs(center - (previous_lane_.left_x + previous_lane_.right_x) / 2)
                     : 0);
            if (score < best) {
                best = score;
                out.lane = {true, true, l.x, r.x, heading, 0.9, config_.lookahead};
            }
        }
    if (!out.lane.valid && !lanes.empty()) {
        // One real boundary plus the known metric width, never a constant pixel width.
        const Tape *chosen = nullptr;
        bool is_left = false;
        double score = 1e10;
        for (const auto &t : lanes) {
            const bool left = t.x < 0;
            const double expected = previous_lane_.valid && previous_lane_.metric
                                        ? (left ? previous_lane_.left_x : previous_lane_.right_x)
                                        : (left ? -config_.lane_width / 2 : config_.lane_width / 2);
            const double d = std::abs(t.x - expected);
            if (d < 0.28 && d < score) {
                score = d;
                chosen = &t;
                is_left = left;
            }
        }
        if (chosen) {
            const double width = config_.lane_width / std::cos(chosen->heading);
            out.lane = {true,
                        false,
                        is_left ? chosen->x : chosen->x - width,
                        is_left ? chosen->x + width : chosen->x,
                        chosen->heading,
                        0.55,
                        config_.lookahead};
        }
    }
    if (out.lane.valid) {
        out.lane.metric = true;
        out.lane.lateral_uncertainty_m = .03;
    } else {
        // Sunlit red ground can enter the broad low-light white mask and
        // absorb a genuine stripe. Fit stricter current paint first, then
        // retain the original shadow mask as a bounded fallback.
        cv::Mat bright_white;
        cv::inRange(hsv, cv::Scalar(0,0,170), cv::Scalar(179,60,255), bright_white);
        cv::morphologyEx(bright_white,bright_white,cv::MORPH_CLOSE,
                        cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
        out.lane=imageLane(bright_white,bgr.size(),config_,ground_,diagnostics_);
        if(!out.lane.valid) out.lane=imageLane(white,bgr.size(),config_,ground_,diagnostics_);
    }
    if (out.lane.valid) {
        const bool consistent =
            lane_streak_ == 0 || (std::abs((out.lane.left_x + out.lane.right_x) -
                                           (previous_lane_.left_x + previous_lane_.right_x)) < 0.5 &&
                                  std::abs(out.lane.heading - previous_lane_.heading) < 0.25);
        lane_streak_ = consistent ? lane_streak_ + 1 : 1;
        previous_lane_ = out.lane;
        last_lane_seen_s_ = time;
        if (lane_streak_ < config_.confirm_frames)
            out.lane.valid = false;
    } else if (last_lane_seen_s_ < 0 || time-last_lane_seen_s_ > .30) {
        lane_streak_ = 0;
        previous_lane_ = {};
    }
    }

    if (tasks.zebra) {
    std::sort(zebra_bars.begin(), zebra_bars.end(), [](const Tape &a, const Tape &b) { return a.x < b.x; });
    std::vector<double> zebra_near;
    for (size_t i = 0; i < zebra_bars.size(); ++i) {
        std::vector<double> row{zebra_bars[i].min_y};
        double last = zebra_bars[i].x;
        for (size_t j = i + 1; j < zebra_bars.size(); ++j) {
            const auto &t = zebra_bars[j];
            const double gap = t.x - last;
            if (gap > 0.29)
                break;
            if (gap >= 0.14 && std::abs(t.min_y - zebra_bars[i].min_y) < 0.08 &&
                std::abs(t.heading - zebra_bars[i].heading) < 0.18) {
                row.push_back(t.min_y);
                last = t.x;
            }
        }
        if (row.size() >= 3 && row.size() > zebra_near.size())
            zebra_near = row;
    }
    if (!zebra_near.empty()) {
        out.zebra_y = median(zebra_near);
        const double predicted = previous_zebra_y_ - (out.motion_valid ? out.forward_delta : 0);
        zebra_streak_ =
            (zebra_streak_ == 0 || std::abs(out.zebra_y - predicted) < 0.18) ? zebra_streak_ + 1 : 1;
        previous_zebra_y_ = out.zebra_y;
        out.zebra = zebra_streak_ >= config_.confirm_frames;
    } else
        zebra_streak_ = 0;

    // Presence is image evidence; absence of a metric projection must not erase it.
    const auto zebra_image = detectZebraImage(bgr);
    diagnostics_.zebra_image_stripes = int(zebra_image.stripes.size());
    diagnostics_.zebra_image_reason = zebra_image.reason;
    diagnostics_.zebra_near_edge_px = std::isfinite(zebra_image.near_edge_px) ? zebra_image.near_edge_px : -1;
    if (zebra_image.candidate) {
        const bool compatible_edge = previous_zebra_edge_ < 0 || !std::isfinite(zebra_image.near_edge_px) ||
                                     std::abs(zebra_image.near_edge_px - previous_zebra_edge_) < bgr.rows * .16;
        zebra_image_streak_ = compatible_edge ? zebra_image_streak_ + 1 : 1;
        previous_zebra_edge_ = diagnostics_.zebra_near_edge_px;
    } else {
        zebra_image_streak_ = 0;
        previous_zebra_edge_ = -1;
    }
    out.zebra_detected = zebra_image_streak_ >= config_.confirm_frames;
    if (zebra_profile_) {
        // The selected 24 October profile is authoritative for zebra distance.
        // Never silently fall back to the older road homography outside its range.
        out.zebra = false;
        out.zebra_y = 0;
        if (zebra_image.candidate) {
            const auto range = zebra_profile_->measure(zebra_image.near_edge_px, bgr.size());
            out.zebra_range_status = range.status;
            out.zebra_distance_reason = "20261024 near-edge profile: " + range.status;
            if (out.zebra_detected && range.valid) {
                out.zebra = true;
                out.zebra_y = range.front_distance_m + config_.front_extent;
            }
        }
    } else if (out.zebra) {
        out.zebra_detected = true;
        out.zebra_range_status = "in_range";
        out.zebra_distance_reason = "measured road projection";
    } else if (out.zebra_detected) {
        out.zebra_range_status = "unavailable";
        out.zebra_distance_reason = "paper row visible; no compatible distance profile";
    }

    }
    if (tasks.board) {
    cv::morphologyEx(blue, blue, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    bool board = false;
    cv::Rect observed_board;
    // Hands and a blue-tinted dark sleeve can join the panel contour. Keep the
    // full-colour candidate for shade, and separately extract the bright blue
    // panel body. Convex-hull corners tolerate a small hand notch, while actual
    // filled area still rejects triangles, thin poles and mostly empty boxes.
    // In direct sun a pale blue panel and the sky can be one low-threshold
    // component touching the image border. Brighter layers separate the panel
    // without weakening the shape, position or removal-evidence checks below.
    for (const auto &layer : {std::pair<int,int>{70,45},{70,120},{70,180},{70,210},{70,235},{100,120}}) {
        cv::Mat board_mask;
        cv::inRange(hsv, cv::Scalar(88, layer.first, layer.second), cv::Scalar(137, 255, 255), board_mask);
        cv::morphologyEx(board_mask, board_mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
        cv::morphologyEx(board_mask, board_mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
        cv::findContours(board_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours) {
            const auto r = cv::boundingRect(contour);
            if (r.area() == 0) continue;
            const double fill = cv::contourArea(contour) / r.area(), aspect = double(r.height) / r.width;
            std::vector<cv::Point> hull, corners;
            cv::convexHull(contour, hull);
            cv::approxPolyDP(hull, corners, cv::arcLength(hull, true) * .025, true);
            if (corners.size() != 4 || fill <= .78 || r.width < bgr.cols * .12 ||
                r.height < bgr.rows * .10 || aspect < .5 || aspect > 2.5 || r.x <= 1 || r.y <= 1 ||
                r.br().x >= bgr.cols - 1 || r.br().y >= bgr.rows - 1 ||
                r.x + r.width * .5 <= bgr.cols * .25 || r.x + r.width * .5 >= bgr.cols * .75)
                continue;
            const auto base = ground_.project({r.x + r.width * .5, double(r.br().y - 1)}, bgr.size());
            const auto left = ground_.project({double(r.x), double(r.br().y - 1)}, bgr.size());
            const auto right = ground_.project({double(r.br().x - 1), double(r.br().y - 1)}, bgr.size());
            const bool measured = base && left && right;
            const bool metric_board = measured && distance(*left, *right) >= .25 &&
                                      distance(*left, *right) <= .65 && base->y > config_.front_extent &&
                                      base->y < 4.0;
            if (!metric_board && (measured || r.br().y <= bgr.rows * .55)) continue;
            if (!board || r.area() > observed_board.area()) observed_board = r;
            board = true;
        }
    }
    diagnostics_.board_candidate = board;
    // Track the actual visible board rectangle, not three fixed ground points
    // outside this camera's measured field. Initial absence never arms a launch.
    if (board) {
        const double overlap = board_candidate_.area() > 0
                                   ? double((board_candidate_ & observed_board).area()) /
                                         std::max(1, (board_candidate_ | observed_board).area())
                                   : 0;
        board_streak_ = overlap > .6 ? board_streak_ + 1 : 1;
        board_candidate_ = observed_board;
    } else {
        board_streak_ = 0;
        board_candidate_ = {};
    }
    out.board_visible = board && board_streak_ >= config_.confirm_frames;
    if (out.board_visible)
        board_reference_ = observed_board;
    bool cleared = false;
    diagnostics_.board_reference_valid = board_reference_.area() > 0;
    if (!board && board_reference_.area() > 0) {
        // A moving/tilting panel can vacate its old rectangle while still
        // blocking the forward path. Blue sky is above this ground-facing
        // band; the bonnet is below it. Never count old-ROI floor alone as
        // removal when a substantial current blue obstruction remains.
        const cv::Rect path(int(bgr.cols*.18),int(bgr.rows*.45),
                            int(bgr.cols*.64),int(bgr.rows*.33));
        diagnostics_.board_path_blue_fraction=double(cv::countNonZero(blue(path)))/path.area();
        const cv::Rect lower(board_reference_.x, board_reference_.y + board_reference_.height * 2 / 3,
                             board_reference_.width, std::max(1, board_reference_.height / 3));
        const cv::Rect roi = lower & cv::Rect(0, 0, bgr.cols, bgr.rows);
        int floor = 0, blue_pixels = 0;
        for (int y = roi.y; y < roi.br().y; ++y)
            for (int x = roi.x; x < roi.br().x; ++x) {
                const auto v = hsv.at<cv::Vec3b>(y, x);
                if (blue.at<uchar>(y, x))
                    ++blue_pixels;
                if (v[1] >= 25 && v[2] >= 30 && v[2] < 235 && !(v[0] >= 88 && v[0] <= 137))
                    ++floor;
            }
        if (roi.area() > 0) {
            diagnostics_.board_clear_fraction = double(floor) / roi.area();
            diagnostics_.board_blue_fraction = double(blue_pixels) / roi.area();
            cleared = diagnostics_.board_clear_fraction > .50 && diagnostics_.board_blue_fraction < .02 &&
                      diagnostics_.board_path_blue_fraction < .02;
        }
    }
    out.board_observation_valid = out.board_visible || cleared;
    }

    previous_gray_ = gray.clone();
    previous_feature_mask_ = feature_mask;
    previous_time_ = time;
    last_frame_id_ = id;
    have_frame_ = true;
    cached_ = out;
    return out;
}
} // namespace smartcar
