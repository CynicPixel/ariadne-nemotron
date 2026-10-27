#include "planning/route/egress.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ariadne::planning {

cv::Mat BodySupport(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body, double band_lo, double band_hi) {
    const double vox = map.Voxel();
    if (std::abs(g.cell - vox) > 1e-12) throw std::invalid_argument("BodySupport: grid cell must equal the voxel");
    const int cx0 = int(std::lround(g.ox / vox)), cy0 = int(std::lround(g.oy / vox));
    const int R = int(std::ceil(body.rho / vox));
    cv::Mat known_sum = cv::Mat::zeros(g.rows, g.cols, CV_32F);
    double total = 0;   // voxels in the envelope: the same for every column
    const int z0 = int(std::floor((body.axis_lo - body.rho) / vox)), z1 = int(std::floor((body.axis_hi + body.rho) / vox));
    cv::Mat known(g.rows, g.cols, CV_32F), conv;
    for (int z = z0; z <= z1; ++z) {
        const double zc = (z + 0.5) * vox;
        if (zc < band_lo || zc > band_hi) continue;
        // Squared distance from the layer centre to the axis segment, in z only (the axis is vertical).
        const double dz = zc < body.axis_lo ? body.axis_lo - zc : (zc > body.axis_hi ? zc - body.axis_hi : 0.0);
        const double r2 = body.rho * body.rho - dz * dz;
        if (r2 < 0) continue;
        cv::Mat disc = cv::Mat::zeros(2 * R + 1, 2 * R + 1, CV_32F);
        for (int i = -R; i <= R; ++i)
            for (int j = -R; j <= R; ++j)
                if ((i * i + j * j) * vox * vox <= r2 + 1e-12) disc.at<float>(i + R, j + R) = 1.f;
        const double n = cv::sum(disc)[0];
        if (n == 0) continue;
        cv::parallel_for_(cv::Range(0, g.rows), [&](const cv::Range& rows) {   // const reads, one row each
            for (int r = rows.start; r < rows.end; ++r) {
                float* k = known.ptr<float>(r);
                for (int c = 0; c < g.cols; ++c) k[c] = map.State({cx0 + c, cy0 + r, z}) != kUnknown ? 1.f : 0.f;
            }
        });
        // A symmetric kernel, so correlation = convolution; outside the grid is unobserved (constant 0).
        cv::filter2D(known, conv, CV_32F, disc, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
        known_sum += conv;
        total += n;
    }
    if (total == 0) return cv::Mat::zeros(g.rows, g.cols, CV_32F);
    cv::Mat s = known_sum / total;
    cv::min(cv::max(s, 0.0), 1.0, s);   // filter2D's float rounding can land a hair outside [0, 1]
    return s;
}

cv::Mat EgressPenalty(const cv::Mat& clear_r, const cv::Mat& support, const cv::Mat& risk, const EgressWeights& w) {
    if (clear_r.type() != CV_32F || support.type() != CV_32F || support.size() != clear_r.size()) {
        throw std::invalid_argument("EgressPenalty: clear_r and support CV_32F of one size");
    }
    cv::Mat p(clear_r.size(), CV_32F);
    for (int r = 0; r < p.rows; ++r)
        for (int c = 0; c < p.cols; ++c) {
            double m = w.w_u * (1.0 - double(support.at<float>(r, c)));
            m += w.w_c * std::max(0.0, w.rho_safe - double(clear_r.at<float>(r, c))) / w.rho_safe;
            if (w.w_r != 0 && !risk.empty()) m += w.w_r * double(risk.at<float>(r, c));
            p.at<float>(r, c) = float(std::max(0.0, m));
        }
    return p;
}

double RouteConfidence(const cv::Mat& support, const std::vector<Cell>& cells) {
    if (cells.empty()) return 0;
    std::vector<float> v;
    v.reserve(cells.size());
    for (const auto& q : cells) v.push_back(support.at<float>(q.r, q.c));
    std::sort(v.begin(), v.end());
    return v[size_t(std::floor(0.05 * double(v.size() - 1)))];
}

EgressReplan EgressPlanner::Replan(const cv::Mat& trav_r, const cv::Mat& penalty, const cv::Mat& support, const cv::Mat& risk) {
    EgressReplan out;
    const size_t e0 = ds_ ? ds_->Expanded() : 0;
    if (!ds_) {
        ds_ = std::make_unique<DStarLite>(trav_r, penalty, target_, exits_);
        out.changed_cells = size_t(trav_r.rows) * size_t(trav_r.cols);
        out.triggers.push_back("first");
    } else {
        const std::vector<Cell> ch = ds_->Update(trav_r, penalty);
        out.changed_cells = ch.size();
        bool obstacle = false;
        for (const auto& q : ch) obstacle |= prev_trav_.at<uint8_t>(q.r, q.c) != 0 && trav_r.at<uint8_t>(q.r, q.c) == 0;   // left trav
        if (obstacle) out.triggers.push_back("new_obstacle");
        for (const auto& q : prev_route_) {
            if (!trav_r.at<uint8_t>(q.r, q.c)) {
                char buf[64];
                std::snprintf(buf, sizeof buf, "reroute: obstacle at (%d, %d)", q.r, q.c);
                out.reroute = buf;
                break;
            }
        }
    }
    out.expanded = ds_->Expanded() - e0;
    out.found = ds_->Found();
    if (out.found) {
        out.cells = ds_->Path();
        out.found = !out.cells.empty();
    }
    if (out.found) {
        out.cost_cells = ds_->Cost();
        out.exit = ds_->GoalIndex(out.cells.back());
        for (size_t i = 1; i < out.cells.size(); ++i) {
            const bool diag = out.cells[i].r != out.cells[i - 1].r && out.cells[i].c != out.cells[i - 1].c;
            out.length_m += (diag ? std::sqrt(2.0) : 1.0) * cell_m_;
        }
        out.confidence = RouteConfidence(support, out.cells);
    }
    if (prev_conf_ >= 0 && out.found && out.confidence < prev_conf_) out.triggers.push_back("confidence_fell");
    if (!risk.empty()) {
        if (!prev_risk_.empty() && !prev_route_.empty()) {
            for (const auto& q : prev_route_) {
                if (risk.at<float>(q.r, q.c) != prev_risk_.at<float>(q.r, q.c)) {
                    out.triggers.push_back("risk_changed");
                    break;
                }
            }
        }
        prev_risk_ = risk.clone();
    }
    prev_route_ = out.cells;
    prev_trav_ = trav_r.clone();
    prev_conf_ = out.found ? out.confidence : -1;
    return out;
}

}  // namespace ariadne::planning
