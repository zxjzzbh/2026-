#pragma once

#include <opencv2/core.hpp>
#include <limits>
#include <string>
#include <vector>

namespace smartcar {

// Image evidence only: these pixel coordinates are not a ground calibration.
struct ZebraImageResult {
    bool candidate = false;
    std::vector<cv::Rect> stripes;
    // NaN if fewer than three consistent, unclipped physical near edges are
    // visible. Presence can remain true when a vehicle hood cuts paper short.
    double near_edge_px = std::numeric_limits<double>::quiet_NaN();
    std::string reason = "no_paper_row";
};

ZebraImageResult detectZebraImage(const cv::Mat &bgr);

} // namespace smartcar
