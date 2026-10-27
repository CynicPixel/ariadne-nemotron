#include "planning/route/egress.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ariadne::planning {

int BodySupportReach(const BodyCapsule& body, double voxel) { return int(std::ceil(body.rho / voxel)); }

void UpdateSupportCounts(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body, double band_lo,
                         double band_hi, const cv::Mat& mask, SupportCounts* out) {
    const double vox = map.Voxel();
    if (std::abs(g.cell - vox) > 1e-12) throw std::invalid_argument("BodySupport: grid cell must equal the voxel");
    const int cx0 = int(std::lround(g.ox / vox)), cy0 = int(std::lround(g.oy / vox));
    const int R = BodySupportReach(body, vox);
    // Per envelope layer: the disc as row spans, w[i + R] = half-width of disc row i (-1 = empty row).
    struct Layer {
        int z;
        std::vector<int> w;
    };
    std::vector<Layer> layers;
    int total = 0;
    const int z0 = int(std::floor((body.axis_lo - body.rho) / vox)), z1 = int(std::floor((body.axis_hi + body.rho) / vox));
    for (int z = z0; z <= z1; ++z) {
        const double zc = (z + 0.5) * vox;
        if (zc < band_lo || zc > band_hi) continue;
        // Squared distance from the layer centre to the axis segment, in z only (the axis is vertical).
        const double dz = zc < body.axis_lo ? body.axis_lo - zc : (zc > body.axis_hi ? zc - body.axis_hi : 0.0);
        const double r2 = body.rho * body.rho - dz * dz;
        if (r2 < 0) continue;
        Layer L{z, std::vector<int>(size_t(2 * R + 1), -1)};
        int n = 0;
        for (int i = -R; i <= R; ++i)
            for (int j = 0; j <= R; ++j)   // the disc is symmetric in j and convex along a row
                if ((i * i + j * j) * vox * vox <= r2 + 1e-12) L.w[size_t(i + R)] = j;
        for (int x : L.w) n += x >= 0 ? 2 * x + 1 : 0;
        if (n == 0) continue;
        total += n;
        layers.push_back(std::move(L));
    }
    const bool all = mask.empty() || out->Empty();
    if (all) out->count = cv::Mat::zeros(g.rows, g.cols, CV_32S);
    out->total = total;
    // The columns to recount and the box around them that their discs read (outside the grid = unobserved).
    cv::Rect box(0, 0, g.cols, g.rows);
    if (!all) {
        std::vector<cv::Point> nz;
        cv::findNonZero(mask, nz);
        if (nz.empty()) return;
        int x0 = nz[0].x, x1 = x0, y0 = nz[0].y, y1 = y0;
        for (const auto& q : nz) x0 = std::min(x0, q.x), x1 = std::max(x1, q.x), y0 = std::min(y0, q.y), y1 = std::max(y1, q.y);
        box = cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    }
    const cv::Rect src = (box + cv::Size(2 * R, 2 * R)) - cv::Point(R, R);   // box dilated by R, may leave the grid
    const int L = int(layers.size()), W = src.width, H = src.height;
    // prefix[l][y][x + 1] = known voxels of layer l in src row y, columns [0, x].
    std::vector<int> prefix(size_t(L) * size_t(H) * size_t(W + 1), 0);
    cv::parallel_for_(cv::Range(0, H), [&](const cv::Range& rows) {   // const map reads, one source row each
        for (int y = rows.start; y < rows.end; ++y) {
            const int r = src.y + y;
            for (int l = 0; l < L; ++l) {
                int* p = &prefix[(size_t(l) * size_t(H) + size_t(y)) * size_t(W + 1)];
                for (int x = 0; x < W; ++x) {
                    const int c = src.x + x;
                    const bool in = r >= 0 && r < g.rows && c >= 0 && c < g.cols;
                    p[x + 1] = p[x] + (in && map.State({cx0 + c, cy0 + r, layers[size_t(l)].z}) != kUnknown ? 1 : 0);
                }
            }
        }
    });
    cv::parallel_for_(cv::Range(box.y, box.y + box.height), [&](const cv::Range& rows) {   // one output row each
        for (int r = rows.start; r < rows.end; ++r) {
            int* cnt = out->count.ptr<int>(r);
            const uint8_t* m = all ? nullptr : mask.ptr<uint8_t>(r);
            for (int c = box.x; c < box.x + box.width; ++c) {
                if (m && !m[c]) continue;
                int s = 0;
                for (int l = 0; l < L; ++l)
                    for (int i = -R; i <= R; ++i) {
                        const int w = layers[size_t(l)].w[size_t(i + R)];
                        if (w < 0) continue;
                        const int y = r + i - src.y, x = c - src.x;   // src row of disc row i; this column in src
                        const int* p = &prefix[(size_t(l) * size_t(H) + size_t(y)) * size_t(W + 1)];
                        s += p[x + w + 1] - p[x - w];
                    }
                cnt[c] = s;
            }
        }
    });
}

cv::Mat SupportFromCounts(const SupportCounts& c) {
    cv::Mat s = cv::Mat::zeros(c.count.size(), CV_32F);
    if (c.total == 0) return s;
    for (int r = 0; r < s.rows; ++r) {   // one division per column: a fully observed envelope is exactly 1
        const int* n = c.count.ptr<int>(r);
        float* o = s.ptr<float>(r);
        for (int k = 0; k < s.cols; ++k) o[k] = float(double(n[k]) / double(c.total));
    }
    return s;
}

cv::Mat BodySupport(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body, double band_lo, double band_hi) {
    SupportCounts c;
    UpdateSupportCounts(map, g, body, band_lo, band_hi, cv::Mat(), &c);
    return SupportFromCounts(c);
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
