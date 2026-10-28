#include "planning/volumetric/traversability.hpp"

#include <opencv2/core.hpp>

#include "planning/volumetric/clearance.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ariadne::planning {

bool SdfNormal(const TsdfMap& map, const Idx3& v, core::Vec3* n) {
    const double w_min = map.Params().w_min;
    double d[6];
    const int o[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int i = 0; i < 6; ++i) {
        const TsdfVoxel* x = map.Find({v.x + o[i][0], v.y + o[i][1], v.z + o[i][2]});
        if (!x || x->w <= w_min) return false;
        d[i] = x->sdf;
    }
    const core::Vec3 g(d[0] - d[1], d[2] - d[3], d[4] - d[5]);
    if (g.norm() < 1e-9) return false;
    *n = g / g.norm();   // SDF increases away from the surface: the gradient is the outward normal
    return true;
}

int TravColumnReach(const Esdf& esdf, double voxel) { return int(std::ceil(esdf.DMax() / voxel)) + 1 + 2; }

void UpdateTravColumns(const TsdfMap& map, const Esdf& esdf, const GridFrame& g, const TravParams& p,
                       const cv::Mat& mask, TravColumns* out) {
    const double vox = map.Voxel();
    if (std::abs(g.cell - vox) > 1e-12) throw std::invalid_argument("traversability: grid cell must equal the voxel");
    const int cx0 = int(std::lround(g.ox / vox)), cy0 = int(std::lround(g.oy / vox));
    auto Zv = [&](double z) { return int(std::floor(z / vox)); };
    const bool all = out->Empty() || mask.empty();
    if (out->Empty()) {
        out->trav_a = cv::Mat::zeros(g.rows, g.cols, CV_8U), out->trav_r_strict = cv::Mat::zeros(g.rows, g.cols, CV_8U);
        out->slab = cv::Mat::zeros(g.rows, g.cols, CV_8U);
        out->cap_occ = cv::Mat::zeros(g.rows, g.cols, CV_32F), out->clear_a = cv::Mat::zeros(g.rows, g.cols, CV_32F);
        out->n_down = cv::Mat::zeros(g.rows, g.cols, CV_32F), out->n_hang = cv::Mat::zeros(g.rows, g.cols, CV_32F);
    }
    const int sl = Zv(p.slab_z_lo), sh = Zv(p.slab_z_hi);
    const int rl = Zv(p.responder.axis_lo), rh = Zv(p.responder.axis_hi), al = Zv(p.agent.axis_lo), ah = Zv(p.agent.axis_hi);
    const int kl = Zv(p.risk_z_lo), kh = Zv(p.risk_z_hi), ol = Zv(p.open_below_z_lo);
    // Columns are independent (const map/ESDF reads, one output pixel each): rows in parallel, same bits.
    cv::parallel_for_(cv::Range(0, g.rows), [&](const cv::Range& rows) {
    for (int r = rows.start; r < rows.end; ++r) {
        const uint8_t* mk = all ? nullptr : mask.ptr<uint8_t>(r);
        for (int c = 0; c < g.cols; ++c) {
            if (mk && !mk[c]) continue;
            const int x = cx0 + c, y = cy0 + r;
            double cr = esdf.DMax(), co = esdf.DMax(), ca = esdf.DMax();
            for (int z = rl; z <= rh; ++z) {
                cr = std::min(cr, esdf.Egress({x, y, z}));
                co = std::min(co, esdf.Agent({x, y, z}));
            }
            for (int z = al; z <= ah; ++z) ca = std::min(ca, esdf.Agent({x, y, z}));
            out->cap_occ.at<float>(r, c) = float(co), out->clear_a.at<float>(r, c) = float(ca);
            out->trav_r_strict.at<uint8_t>(r, c) = cr >= p.responder.rho - kTravEpsM;
            out->trav_a.at<uint8_t>(r, c) = ca >= p.agent.rho - kTravEpsM;
            bool any_free = false, any_occ = false;   // 1.8's column rule (D45)
            for (int z = sl; z <= sh && !any_occ; ++z) {
                const uint8_t st = map.State({x, y, z});
                any_free |= st == kFree;
                any_occ |= st == kOccupied;
            }
            out->slab.at<uint8_t>(r, c) = any_occ ? kOccupied : (any_free ? kFree : kUnknown);
            // Risk counts for this column. "Open beneath" (measured fix, traversability-risk.md §6): no occupied voxel
            // below the band within +-2 voxels (a thin wall's voxels jitter by one between heights), and the column
            // itself observed free somewhere below the band (unobserved is not "open").
            bool open_below = true, seen_free = false;
            for (int z = ol; z < kl && open_below; ++z) {
                seen_free |= map.State({x, y, z}) == kFree;
                for (int dy = -2; dy <= 2 && open_below; ++dy)
                    for (int dx = -2; dx <= 2 && open_below; ++dx) open_below = map.State({x + dx, y + dy, z}) != kOccupied;
            }
            open_below = open_below && seen_free;
            int down = 0, occ = 0;
            for (int z = kl; z <= kh; ++z) {
                if (map.State({x, y, z}) != kOccupied) continue;
                ++occ;
                core::Vec3 n;
                if (SdfNormal(map, {x, y, z}, &n) && n.z() < -p.down_cos) ++down;
            }
            out->n_down.at<float>(r, c) = float(down);
            out->n_hang.at<float>(r, c) = open_below ? float(occ) : 0.f;
        }
    }
    });
}

TraversabilityMaps FinishTraversability(const TravColumns& t, const GridFrame& g, double voxel, double d_max,
                                        const TravParams& p) {
    const double vox = voxel;
    TraversabilityMaps m;
    m.trav_a = t.trav_a.clone(), m.trav_r_strict = t.trav_r_strict.clone(), m.slab = t.slab.clone();
    m.clear_a = t.clear_a.clone();
    m.trav_r = cv::Mat::zeros(g.rows, g.cols, CV_8U), m.clear_r = cv::Mat::zeros(g.rows, g.cols, CV_32F);
    // Observed rule: 1.8's own 2D clearance (planning::EdgeClearance, the GT convention) from each free column to
    // the nearest non-free column, capped like the 3D field.
    const cv::Mat clear2d = EdgeClearance(m.slab != kFree, vox, d_max);
    for (int r = 0; r < g.rows; ++r)
        for (int c = 0; c < g.cols; ++c) {
            const double cr = std::min(clear2d.at<double>(r, c), double(t.cap_occ.at<float>(r, c)));
            m.clear_r.at<float>(r, c) = float(cr);
            m.trav_r.at<uint8_t>(r, c) = m.slab.at<uint8_t>(r, c) == kFree && cr >= p.responder.rho - kTravEpsM;
        }
    const int rad = int(std::floor(p.risk_radius / vox));
    cv::Mat disc = cv::Mat::zeros(2 * rad + 1, 2 * rad + 1, CV_32F);
    for (int i = -rad; i <= rad; ++i)
        for (int j = -rad; j <= rad; ++j) disc.at<float>(i + rad, j + rad) = float(i * i + j * j <= rad * rad);
    cv::Mat s_down, s_hang;
    cv::filter2D(t.n_down, s_down, CV_32F, disc, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(t.n_hang, s_hang, CV_32F, disc, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    // filter2D may take its DFT path for a large kernel: clamp the round-off below 0 too.
    m.overhang = cv::max(cv::min(s_down / p.n_ref, 1.0), 0.0);
    m.unsupported = cv::max(cv::min(s_hang / p.n_ref, 1.0), 0.0);
    m.risk = 1.0 - (1.0 - m.overhang).mul(1.0 - m.unsupported);
    return m;
}

TraversabilityMaps ComputeTraversability(const TsdfMap& map, const Esdf& esdf, const GridFrame& g, const TravParams& p) {
    TravColumns t;
    UpdateTravColumns(map, esdf, g, p, cv::Mat(), &t);
    return FinishTraversability(t, g, map.Voxel(), esdf.DMax(), p);
}

}  // namespace ariadne::planning
