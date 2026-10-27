#include "planning/mission/nbv.hpp"

#include "core/random/philox.hpp"
#include "planning/volumetric/clearance.hpp"
#include "planning/volumetric/occupancy_grid.hpp"

#include <opencv2/core.hpp>
#include <opencv2/geometry/2d.hpp>   // OpenCV 5: DistanceTypes (DIST_L2) live in the geometry module
#include <opencv2/imgproc.hpp>

#include <opencv2/core/utility.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <queue>
#include <stdexcept>

namespace ariadne::planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

uint64_t Mix(uint64_t x) {   // splitmix64 finaliser: a stable task id from (generator, anchor)
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

int64_t TaskId(TaskSource s, int64_t anchor) {
    return int64_t(Mix(uint64_t(anchor) ^ (uint64_t(uint8_t(s)) << 56)) >> 1);   // non-negative
}

double Hb(double p) { return p <= 0 || p >= 1 ? 0.0 : -(p * std::log2(p) + (1 - p) * std::log2(1 - p)); }

struct Target {
    TaskSource src = TaskSource::kFrontier;
    core::Vec3 p = core::Vec3::Zero();
    int64_t anchor = 0;
    int size = 0;
    double gain_conn = 0, gain_alias = 0;   // producer-supplied terms (C, D)
};

Cell CellOf(const GridFrame& g, double x, double y) {
    return Cell{int(std::floor((y - g.oy) / g.cell)), int(std::floor((x - g.ox) / g.cell))};
}

bool In(const cv::Mat& m, const Cell& c) { return c.r >= 0 && c.c >= 0 && c.r < m.rows && c.c < m.cols; }

// Generator A: frontier clusters, split along the first principal axis while the AABB extent exceeds `ext` (FUEL).
void SplitCluster(const TsdfMap& map, std::vector<Idx3> vox, double ext, int depth, std::vector<Target>* out) {
    if (vox.empty()) return;
    Idx3 lo = vox[0], hi = vox[0];
    for (const auto& v : vox) {
        lo = {std::min(lo.x, v.x), std::min(lo.y, v.y), std::min(lo.z, v.z)};
        hi = {std::max(hi.x, v.x), std::max(hi.y, v.y), std::max(hi.z, v.z)};
    }
    const double e = map.Voxel() * double(std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z}) + 1);
    if (e > ext && vox.size() >= 2 && depth < 8) {
        cv::Mat pts(int(vox.size()), 3, CV_64F);
        for (size_t i = 0; i < vox.size(); ++i) {
            pts.at<double>(int(i), 0) = vox[i].x, pts.at<double>(int(i), 1) = vox[i].y, pts.at<double>(int(i), 2) = vox[i].z;
        }
        const cv::PCA pca(pts, cv::Mat(), cv::PCA::DATA_AS_ROW, 1);
        std::vector<Idx3> a, b;
        for (size_t i = 0; i < vox.size(); ++i) {
            const cv::Mat proj = pca.project(pts.row(int(i)));
            (proj.at<double>(0, 0) < 0 ? a : b).push_back(vox[i]);
        }
        if (!a.empty() && !b.empty()) {
            SplitCluster(map, std::move(a), ext, depth + 1, out);
            SplitCluster(map, std::move(b), ext, depth + 1, out);
            return;
        }
    }
    std::sort(vox.begin(), vox.end());
    core::Vec3 c = core::Vec3::Zero();
    for (const auto& v : vox) c += map.Centre(v);
    Target t;
    t.src = TaskSource::kFrontier, t.p = c / double(vox.size()), t.anchor = Pack(vox[0]), t.size = int(vox.size());
    out->push_back(t);
}

// The route's body envelope (D113's support definition): voxels within rho of each route column's capsule axis.
std::unordered_set<int64_t> RouteEnvelope(const TsdfMap& map, const GridFrame& g, const BodyCapsule& b,
                                          const std::vector<Cell>& cells) {
    std::unordered_set<int64_t> env;
    const double vox = map.Voxel();
    const int cx0 = int(std::lround(g.ox / vox)), cy0 = int(std::lround(g.oy / vox)), R = int(std::ceil(b.rho / vox));
    const int z0 = int(std::floor((b.axis_lo - b.rho) / vox)), z1 = int(std::floor((b.axis_hi + b.rho) / vox));
    for (const auto& q : cells)
        for (int z = z0; z <= z1; ++z) {
            const double zc = (z + 0.5) * vox;
            const double dz = zc < b.axis_lo ? b.axis_lo - zc : (zc > b.axis_hi ? zc - b.axis_hi : 0.0);
            const double r2 = b.rho * b.rho - dz * dz;
            if (r2 < 0) continue;
            for (int i = -R; i <= R; ++i)
                for (int j = -R; j <= R; ++j)
                    if ((i * i + j * j) * vox * vox <= r2 + 1e-12) env.insert(Pack({cx0 + q.c + j, cy0 + q.r + i, z}));
        }
    return env;
}

}  // namespace

std::vector<int> OpenTour(const std::vector<std::vector<double>>& D);
double TourLength(const std::vector<std::vector<double>>& D, const std::vector<int>& order);

namespace {

// 4.6 TARE-style scheduling (EXPLORATION_4.6 §1, D116): local horizon first (best view per target inside a box of
// half-width = the depth gate around the agent, open tour on geodesics within that box), else the global tour over
// exploring subspaces (best view per subspace as its representative, geodesics on the full plan space). The scheduled
// view moves to the front; the rest keep their Ariadne order.
void TareSchedule(const cv::Mat& plan, const GridFrame& g, const Cell& me, const NbvParams& p, NbvPlan* out) {
    auto& o = out->options;
    auto CellOfXY = [&](double x, double y) { return Cell{int(std::floor((y - g.oy) / g.cell)), int(std::floor((x - g.ox) / g.cell))}; };
    const double half = p.frustum.max_range_m;
    std::vector<int> local;   // best view per task inside the horizon (options are sorted: first = best)
    std::vector<int64_t> seen;
    for (size_t i = 0; i < o.size(); ++i) {
        const Cell c = CellOfXY(o[i].goal_xy.first, o[i].goal_xy.second);
        if (std::max(std::abs(c.r - me.r), std::abs(c.c - me.c)) * g.cell > half) continue;
        if (std::find(seen.begin(), seen.end(), o[i].task_id) != seen.end()) continue;
        seen.push_back(o[i].task_id), local.push_back(int(i));
    }
    std::vector<int> reps;
    cv::Mat space = plan;
    cv::Rect crop(0, 0, plan.cols, plan.rows);
    if (!local.empty()) {
        reps = local;
        const int H = int(std::ceil(half / g.cell));
        crop = cv::Rect(me.c - H, me.r - H, 2 * H + 1, 2 * H + 1) & cv::Rect(0, 0, plan.cols, plan.rows);
        space = plan(crop).clone();
        out->tare = "local " + std::to_string(local.size());
    } else {
        std::vector<std::pair<int, int>> keys;   // exploring subspaces, best view each
        for (size_t i = 0; i < o.size(); ++i) {
            const std::pair<int, int> k{int(std::floor(o[i].centroid_xy.first / p.tare_subspace_m)),
                                        int(std::floor(o[i].centroid_xy.second / p.tare_subspace_m))};
            if (std::find(keys.begin(), keys.end(), k) != keys.end()) continue;
            keys.push_back(k), reps.push_back(int(i));
        }
        out->tare = "global " + std::to_string(reps.size());
    }
    // Distance matrix on geodesics (node 0 = the agent); cells shifted into the crop.
    std::vector<Cell> pts = {Cell{me.r - crop.y, me.c - crop.x}};
    for (int i : reps) {
        const Cell c = CellOfXY(o[size_t(i)].goal_xy.first, o[size_t(i)].goal_xy.second);
        pts.push_back({c.r - crop.y, c.c - crop.x});
    }
    const size_t n = pts.size();
    std::vector<std::vector<double>> D(n, std::vector<double>(n, 1e12));
    for (size_t a = 0; a < n; ++a) {
        const cv::Mat d = GridDistances(space, pts[a]);
        for (size_t b = 0; b < n; ++b) {
            const Cell& q = pts[b];
            if (q.r < 0 || q.c < 0 || q.r >= d.rows || q.c >= d.cols) continue;
            const double v = d.at<double>(q.r, q.c);
            if (std::isfinite(v)) D[a][b] = v;
        }
    }
    const std::vector<int> tour = OpenTour(D);
    if (tour.empty()) return;
    const int first = reps[size_t(tour[0] - 1)];
    std::rotate(o.begin(), o.begin() + first, o.begin() + first + 1);   // the scheduled view to the front
}

}  // namespace

double CommandTime(const ViewCommand& c, double x, double y, double z, double heading, bool final_yaw, double yaw,
                   const core::scripted::MotionLimits& L, const cv::Mat& clear_a, const GridFrame& g, double rho_a,
                   double z_max, double* turn_s) {
    using namespace core::scripted;
    std::vector<Waypoint> path;
    for (const auto& [wx, wy] : c.wp) path.push_back({wx, wy, z, false, 0.0});
    if (!path.empty() && final_yaw && !c.cut) path.back().has_yaw = true, path.back().yaw = yaw;
    if (std::isnan(heading)) {   // unknown: as if already facing the first move (or the final yaw of a turn here)
        heading = path.empty() || std::isnan(path[0].x) ? (path.empty() ? 0.0 : path[0].yaw)
                                                         : std::atan2(path[0].y - y, path[0].x - x);
    }
    const ClearanceFn clr = [&](double x0, double y0, double z0, double x1, double y1, double z1) {
        const int n = std::max(int(std::ceil(std::hypot(x1 - x0, y1 - y0) / g.cell)), 1);
        double cmin = 1e9;
        for (int i = 0; i <= n; ++i) {   // every cell along the move, as the mover's ClearanceAlong
            const Cell q = CellOf(g, x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
            const bool in = q.r >= 0 && q.c >= 0 && q.r < clear_a.rows && q.c < clear_a.cols;
            cmin = std::min(cmin, in ? double(clear_a.at<float>(q.r, q.c)) : 0.0);
        }
        return std::min({std::max(cmin, rho_a), z0, z1, z_max - std::max(z0, z1)});
    };
    double T = 0, turns = 0;
    for (const Segment& s : PlanCommand({x, y, z, heading, 0.0}, 0.0, path, L, clr, "c_time")) {
        T += s.duration;
        if (s.kind == Segment::Kind::kTurn) turns += s.duration;
    }
    if (turn_s) *turn_s = turns;
    return T;
}

double VoxelEntropy(const TsdfVoxel* v, double w_min) {
    if (!v || v->w <= w_min) return 1.0;
    return Hb(0.5 * std::exp(-double(v->w)));   // free 0.5 e^-w and occupied 1 - 0.5 e^-w have the same entropy
}

bool PeerKnown::Known(const core::Vec3& p) const {
    if (!mask) return false;
    const Cell c = CellOf(grid, p.x(), p.y());
    return In(*mask, c) && mask->at<uint8_t>(c.r, c.c) != 0;
}

IgCounts RaycastIg(const TsdfMap& map, const core::Vec3& eye, double yaw, const RigFrustum& f,
                   const std::unordered_set<int64_t>* route_env, const Box3& bounds, int grid, const PeerKnown* peer,
                   const GridFrame* footprint) {
    IgCounts ig;
    // Footprint: a flat per-block counter (no map, no hashing) and the blocks touched, for D175 R1.
    const int fbw = footprint ? (footprint->cols + 31) / 32 : 0, fbh = footprint ? (footprint->rows + 31) / 32 : 0;
    std::vector<int> per_block(size_t(fbw * fbh), 0);
    std::vector<int> touched;
    const int G = grid > 0 ? grid : f.grid;
    const double w_min = map.Params().w_min, half = 0.5 * map.Voxel();
    const double vox = map.Voxel();
    const core::Vec3 fw(std::cos(yaw), std::sin(yaw), 0), right(std::sin(yaw), -std::cos(yaw), 0), down(0, 0, -1);
    std::unordered_set<int64_t> seen;
    for (int i = 0; i < G; ++i)
        for (int j = 0; j < G; ++j) {
            const double u = (j + 0.5) * f.width / G, v = (i + 0.5) * f.height / G;
            const core::Vec3 d = (fw + ((u - f.cx) / f.fx) * right + ((v - f.cy) / f.fy) * down).normalized();
            const core::Vec3 a = eye / vox, b = (eye + d * f.max_range_m) / vox;
            bool stop = false;
            TraverseVoxels(a, b, [&](int x, int y, int z) {
                if (stop) return;
                const Idx3 q{x, y, z};
                const TsdfVoxel* t = map.Find(q);   // one lookup; State() inlined (D110: unknown iff w <= w_min)
                const uint8_t st = !t || t->w <= w_min ? kUnknown : (t->sdf < half ? kOccupied : kFree);
                const core::Vec3 c = map.Centre(q);
                const bool inside = (c.array() >= bounds.lo.array()).all() && (c.array() <= bounds.hi.array()).all();
                if (inside && seen.insert(Pack(q)).second && !(st == kUnknown && peer && peer->Known(c))) {
                    ++ig.voxels;
                    ig.entropy += VoxelEntropy(t, w_min);
                    if (st == kUnknown) {
                        ++ig.coverage;
                        if (footprint) {
                            const Cell bc = CellOf(*footprint, c.x(), c.y());
                            if (bc.r >= 0 && bc.c >= 0 && bc.r < footprint->rows && bc.c < footprint->cols) {
                                const int k = (bc.r / 32) * fbw + bc.c / 32;   // 32 = multi::kBlock
                                if (per_block[size_t(k)]++ == 0) touched.push_back(k);
                            }
                        }
                        if (route_env && route_env->count(Pack(q))) ++ig.egress;
                    }
                }
                if (st == kOccupied) stop = true;
            });
        }
    if (footprint) {   // the 32 heaviest blocks, ties by block index (deterministic)
        std::sort(touched.begin(), touched.end(), [&](int a, int b) {
            return per_block[size_t(a)] != per_block[size_t(b)] ? per_block[size_t(a)] > per_block[size_t(b)] : a < b;
        });
        for (size_t i = 0; i < touched.size() && i < 32; ++i) {
            const int k = touched[i], n = per_block[size_t(k)];
            ig.footprint.push_back({int16_t(k % fbw), int16_t(k / fbw), uint8_t(std::min(255, (n + kFootprintUnit - 1) / kFootprintUnit))});
        }
    }
    return ig;
}

std::vector<View> SampleViews(const TsdfMap& map, const GridFrame& g, const cv::Mat& trav_a, const core::Vec3& target,
                              const SamplerParams& sp, const RigFrustum& f, const std::unordered_set<int64_t>* route_env,
                              const Box3& bounds, const PeerKnown* peer, const GridFrame* footprint) {
    std::vector<View> views;
    const double vox = map.Voxel();
    const Idx3 tv = map.ToVoxel(target);
    for (double r : sp.radii)
        for (int k = 0; k < sp.azimuths; ++k) {
            const double az = 2.0 * M_PI * k / sp.azimuths;
            const core::Vec3 eye(target.x() + r * std::cos(az), target.y() + r * std::sin(az), sp.z);
            const Cell c = CellOf(g, eye.x(), eye.y());
            if (!In(trav_a, c) || !trav_a.at<uint8_t>(c.r, c.c)) continue;
            bool blocked = false;
            TraverseVoxels(eye / vox, target / vox, [&](int x, int y, int z) {
                if (blocked) return;
                if (std::max({std::abs(x - tv.x), std::abs(y - tv.y), std::abs(z - tv.z)}) <= 1) return;
                blocked = map.State({x, y, z}) == kOccupied;
            });
            if (blocked) continue;
            View v;
            v.eye = eye, v.cell = c, v.yaw = std::atan2(target.y() - eye.y(), target.x() - eye.x());
            v.ig = RaycastIg(map, eye, v.yaw, f, route_env, bounds, f.rank_grid, peer);   // ranking bundle
            views.push_back(v);
        }
    std::stable_sort(views.begin(), views.end(), [](const View& a, const View& b) { return a.ig.coverage > b.ig.coverage; });
    if (int(views.size()) > sp.keep) views.resize(size_t(sp.keep));
    for (auto& v : views) v.ig = RaycastIg(map, v.eye, v.yaw, f, route_env, bounds, -1, peer, footprint);   // the full bundle
    return views;
}

double P90Scale(std::vector<double> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const double p90 = v[size_t(std::floor(0.9 * double(v.size() - 1)))];
    if (p90 > 0) return p90;
    return v.back() > 0 ? v.back() : 0.0;
}

cv::Mat KnownFreePlan(const TraversabilityMaps& tm, double rho_a) {
    cv::Mat ok = tm.clear_r >= float(rho_a - kTravEpsM);   // 255 where the egress clearance allows the agent
    cv::Mat plan = tm.trav_a.clone();
    plan.setTo(0, ok == 0);
    return plan;
}

cv::Mat FlownReturnSpace(const cv::Mat& plan, const cv::Mat& slab, const GridFrame& g, const std::vector<core::Vec3>& track,
                         const std::vector<Cell>& entrances, double body_m, cv::Mat* swept) {
    cv::Mat sw = cv::Mat::zeros(plan.size(), CV_8U);
    if (!track.empty()) {
        std::vector<cv::Point> pts;   // (x = col, y = row)
        auto ColRow = [&](const core::Vec3& p) {
            return cv::Point(int(std::floor((p.x() - g.ox) / g.cell)), int(std::floor((p.y() - g.oy) / g.cell)));
        };
        const cv::Point first = ColRow(track.front());
        double best = kInf;
        for (const Cell& e : entrances) {
            const double d = std::hypot(e.c - first.x, e.r - first.y);
            if (d < best) best = d, pts.assign(1, cv::Point(e.c, e.r));
        }
        for (const auto& p : track) pts.push_back(ColRow(p));
        // The centre line (1 px), then every cell centre within the body radius of it (exact L2 distance transform):
        // never claims a column the body did not cover (a thick cv::line rounds its half-width up).
        cv::Mat off(plan.size(), CV_8U, cv::Scalar(255));
        if (pts.size() == 1) off.at<uint8_t>(pts[0]) = 0;
        for (size_t i = 1; i < pts.size(); ++i) cv::line(off, pts[i - 1], pts[i], 0, 1, cv::LINE_8);
        cv::Mat d;
        cv::distanceTransform(off, d, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        sw = d <= float(body_m / g.cell);
        sw.setTo(1, sw);
        sw.setTo(0, slab == kOccupied);
    }
    cv::Mat ret = plan.clone();
    ret.setTo(1, sw);
    if (swept) *swept = sw;
    return ret;
}

double ArrivalBottleneck(const cv::Mat& B, const Cell& goal, int rad) {
    double best = -1;
    for (int dy = -rad; dy <= rad; ++dy)
        for (int dx = -rad; dx <= rad; ++dx) {
            const int y = goal.r + dy, x = goal.c + dx;
            if (dx * dx + dy * dy > rad * rad || y < 0 || x < 0 || y >= B.rows || x >= B.cols) continue;
            best = std::max(best, B.at<double>(y, x));
        }
    return best;
}

cv::Mat Bottleneck(const cv::Mat& plan, const cv::Mat& clr, const Cell& from, double escape_cells) {
    const int H = plan.rows, W = plan.cols;
    cv::Mat B(H, W, CV_64F, cv::Scalar(-1.0));
    auto ok = [&](int y, int x) { return y >= 0 && x >= 0 && y < H && x < W && plan.at<uint8_t>(y, x); };
    if (!ok(from.r, from.c)) return B;
    auto c = [&](int y, int x) { return std::hypot(y - from.r, x - from.c) <= escape_cells ? kInf : double(clr.at<float>(y, x)); };
    using Node = std::pair<double, int64_t>;
    std::priority_queue<Node> open;   // max-heap on the bottleneck
    B.at<double>(from.r, from.c) = kInf;
    open.emplace(kInf, int64_t(from.r) * W + from.c);
    static const int dr[8] = {0, 0, 1, -1, 1, 1, -1, -1}, dc[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    while (!open.empty()) {
        const auto [g, u] = open.top();
        open.pop();
        const int uy = int(u / W), ux = int(u % W);
        if (g < B.at<double>(uy, ux)) continue;
        for (int k = 0; k < 8; ++k) {
            const int y = uy + dr[k], x = ux + dc[k];
            if (!ok(y, x)) continue;
            if (k >= 4 && (!ok(uy + dr[k], ux) || !ok(uy, ux + dc[k]))) continue;
            const double nb = std::min(g, c(y, x));
            if (nb > B.at<double>(y, x)) {
                B.at<double>(y, x) = nb;
                open.emplace(nb, int64_t(y) * W + x);
            }
        }
    }
    return B;
}

NbvPlan PlanNbv(const NbvInputs& in, int agent, const AgentState& st, const NbvParams& p) {
    if (!in.map || !in.frontiers || !in.coarse || !in.trav) throw std::invalid_argument("PlanNbv: map inputs");
    const TsdfMap& map = *in.map;
    const GridFrame& g = in.grid;
    const TraversabilityMaps& tm = *in.trav;
    NbvPlan out;
    auto lap_t = std::chrono::steady_clock::now();
    const auto lap = [&](const char* stage) {   // the profile's stage split; never read by planning
        const auto now = std::chrono::steady_clock::now();
        out.stage_ms.emplace_back(stage, std::chrono::duration<double, std::milli>(now - lap_t).count());
        lap_t = now;
    };

    // Plan space (D158): trav_a, plus the agent's own escape disc where the slab is not occupied.
    const Cell me = CellOf(g, st.position.x(), st.position.y());
    cv::Mat plan = KnownFreePlan(tm, p.rho_a);
    const double esc = p.escape_m / g.cell;
    for (int r = std::max(0, me.r - int(esc) - 1); r <= std::min(plan.rows - 1, me.r + int(esc) + 1); ++r)
        for (int c = std::max(0, me.c - int(esc) - 1); c <= std::min(plan.cols - 1, me.c + int(esc) + 1); ++c)
            if (std::hypot(r - me.r, c - me.c) <= esc && tm.slab.at<uint8_t>(r, c) != kOccupied) plan.at<uint8_t>(r, c) = 1;
    const cv::Mat dist = GridDistances(plan, me);
    const cv::Mat bott = Bottleneck(plan, tm.clear_r, me, esc);
    lap("space");

    // lambda schedule (D114): the path-proportional costs counted once.
    const double fe = std::min(1.0, st.e_used_m / p.e_total_m), ft = std::min(1.0, in.t_s / p.mission_T_s);
    out.lambda_path = (1.0 + (1.0 + fe * fe * fe) + (1.0 + ft * ft * ft)) / 3.0;
    out.lambda_t = 1.0 + ft * ft * ft;

    // Scored at the commanded cell itself (rad 0): with the known-free plan space every commanded goal has the agent's
    // body clear, the practice of FUEL / NBVP / GBPlanner2, and the score belongs to the pose the agent is sent to.
    // D158's max over the goal's escape disc compensated for goals in unknown space (unknown-as-passable trav_a); it
    // could score a goal by a cell the agent is never sent to, with no precedent in the literature read (NBV_4.3 §13).
    const int arrive = 0;
    auto Risk = [&](const Cell& c, TaskCandidate* t, const cv::Mat& B) {
        const double b = ArrivalBottleneck(B, c, arrive);
        t->risk_geometric = std::isinf(b) ? 0.0 : (b < 0 ? 1.0 : std::clamp((p.rho_r - b) / (p.rho_r - p.rho_a), 0.0, 1.0));
        t->risk_structural = in.risk_calibrated ? double(tm.risk.at<float>(c.r, c.c)) : 0.0;
        t->risk = std::max(t->risk_geometric, t->risk_structural);
        char buf[200];
        std::snprintf(buf, sizeof buf, "bottleneck %.2f m (D158); structural %s", std::isinf(b) ? 99.0 : b,
                      in.risk_calibrated ? "[PROXY] calibrated (D112)" : "[PROXY] not calibrated (D112): 0");
        t->risk_reason = buf;
    };

    // --- the hard rule, outside the utility (§21.4) ----------------------------------------------------------------
    // E(return) over the return space: known-free plus the swept body of the flown track (the way in, never observed
    // by the forward rig). On the swept columns the body itself passed, so their clearance is at least rho_agent.
    cv::Mat swept;
    const cv::Mat ret_space = FlownReturnSpace(plan, tm.slab, g, st.own_keyframes, in.entrances, p.rho_a, &swept);
    const cv::Mat dist_ret = GridDistances(ret_space, me);
    int best_e = -1;
    double d_ret = kInf;
    for (size_t i = 0; i < in.entrances.size(); ++i) {
        const Cell& e = in.entrances[i];
        if (In(dist_ret, e) && dist_ret.at<double>(e.r, e.c) < d_ret) d_ret = dist_ret.at<double>(e.r, e.c), best_e = int(i);
    }
    const double e_return = p.e_per_m * d_ret * g.cell;
    lap("return");
    if (best_e < 0) {
        // No entrance over the return space: the flown track itself is now blocked. That is a lost route, not an
        // energy comparison (E_remaining < 1.3 x inf would hold the agent for the rest of the mission): reported as
        // its own supervisor event, the options are not restricted.
        out.supervisor_event = "return route lost: no entrance over known-free space or the flown track";
    } else if (p.e_total_m - st.e_used_m < p.reserve * e_return) {
        out.reserve_rule = true;
        char buf[200];
        std::snprintf(buf, sizeof buf, "energy reserve: E_remaining %.2f < %.2f x E(return) %.2f -> {return, hold}",
                      p.e_total_m - st.e_used_m, p.reserve, e_return);
        out.supervisor_event = buf;
        {
            TaskCandidate t;
            const Cell& e = in.entrances[size_t(best_e)];
            t.source = TaskSource::kReturn, t.task_id = TaskId(TaskSource::kReturn, best_e);
            t.goal_xy = {g.ox + (e.c + 0.5) * g.cell, g.oy + (e.r + 0.5) * g.cell}, t.centroid_xy = t.goal_xy;
            t.path_m = d_ret * g.cell;
            cv::Mat clr_ret = tm.clear_r.clone();
            cv::Mat floor_a = cv::max(clr_ret, float(p.rho_a));
            floor_a.copyTo(clr_ret, swept);
            Risk(e, &t, Bottleneck(ret_space, clr_ret, me, esc));   // on the space the return actually uses
            t.utility = 1;   // the supervisor's order, not a score: return before hold
            out.options.push_back(t);
        }
        TaskCandidate h;
        h.source = TaskSource::kHold, h.task_id = TaskId(TaskSource::kHold, agent);
        h.goal_xy = h.centroid_xy = {st.position.x(), st.position.y()};
        h.risk_reason = "hold";
        out.options.push_back(h);
        return out;
    }

    // --- targets -----------------------------------------------------------------------------------------------------
    std::vector<Target> targets;
    for (const auto& k : in.frontiers->Clusters()) SplitCluster(map, k.voxels, p.split_extent_m, 0, &targets);
    out.targets[0] = int(targets.size());
    {   // B: surface coarse cells in the bottom decile of mean weight (rank-based)
        std::vector<const CoarseCell*> surf;
        for (const auto& c : in.coarse->cells)
            if (c.occupied > 0 && c.known > 0) surf.push_back(&c);
        std::stable_sort(surf.begin(), surf.end(), [](const CoarseCell* a, const CoarseCell* b) { return a->mean_w < b->mean_w; });
        const size_t n = surf.empty() ? 0 : std::max<size_t>(1, surf.size() / 10);
        for (size_t i = 0; i < n; ++i) {
            const CoarseCell& c = *surf[i];
            Target t;
            t.src = TaskSource::kUncertainty;
            t.p = core::Vec3((c.c.x + 0.5) * in.coarse->cell_m, (c.c.y + 0.5) * in.coarse->cell_m, (c.c.z + 0.5) * in.coarse->cell_m);
            t.anchor = Pack(c.c), t.size = 1;
            targets.push_back(t);
        }
        out.targets[1] = int(n);
    }
    {   // C: peers' keyframes near this agent's trajectory but not visited
        int n = 0;
        for (const auto& pk : in.peers) {
            if (pk.agent == agent) continue;
            double dmin = kInf;
            for (const auto& o : st.own_keyframes) dmin = std::min(dmin, (o - pk.p).head<2>().norm());
            if (!(dmin <= p.peer_near_m) || dmin < p.peer_seen_m) continue;
            Target t;
            t.src = TaskSource::kConnectivity, t.p = pk.p, t.anchor = pk.kf_id * 4 + pk.agent, t.size = 1;
            t.gain_conn = in.connectivity ? in.connectivity->Gain(agent, pk.kf_id) : 0.0;
            targets.push_back(t);
            ++n;
        }
        out.targets[2] = n;
        if (!in.connectivity) out.notes.push_back("pose_connectivity = 0: no ConnectivityOracle (3.4 selector not merged)");
    }
    if (in.alias) {   // D
        for (const auto& a : in.alias->Places()) {
            Target t;
            t.src = TaskSource::kAlias, t.p = a.p, t.anchor = a.id, t.size = 1, t.gain_alias = a.value;
            targets.push_back(t);
            ++out.targets[3];
        }
    } else {
        out.notes.push_back("loop_disambiguation = 0: no AliasSource (no §17.3 producer)");
    }
    std::unordered_set<int64_t> env;
    if (in.route && in.route->found && in.route_support) {   // E: runs of route cells with support < 1
        env = RouteEnvelope(map, g, in.responder, in.route->cells);
        const auto& rc = in.route->cells;
        for (size_t i = 0; i < rc.size();) {
            if (in.route_support->at<float>(rc[i].r, rc[i].c) >= 1.f) { ++i; continue; }
            size_t j = i, arg = i;
            for (; j < rc.size() && in.route_support->at<float>(rc[j].r, rc[j].c) < 1.f; ++j)
                if (in.route_support->at<float>(rc[j].r, rc[j].c) < in.route_support->at<float>(rc[arg].r, rc[arg].c)) arg = j;
            Target t;
            t.src = TaskSource::kEgress;
            t.p = core::Vec3(g.ox + (rc[arg].c + 0.5) * g.cell, g.oy + (rc[arg].r + 0.5) * g.cell,
                             0.5 * (in.responder.axis_lo + in.responder.axis_hi));
            t.anchor = int64_t(rc[arg].r) * 100003 + rc[arg].c, t.size = int(j - i);
            targets.push_back(t);
            ++out.targets[4];
            i = j;
        }
    }
    const std::unordered_set<int64_t>* envp = env.empty() ? nullptr : &env;
    lap("targets");

    const cv::Mat known = KnownFreePlan(tm, p.rho_a);   // view goals only in known-free agent space (D153b)
    // --- views, raw terms, costs ---------------------------------------------------------------------------------
    PeerKnown peer;
    peer.mask = in.peer_known, peer.grid = g;
    const PeerKnown* peerp = in.peer_known ? &peer : nullptr;
    // Targets are independent (read-only map, per-target output slot): evaluated in parallel, gathered in target order,
    // so the options are exactly the serial loop's. No locks, no shared writes.
    std::vector<std::vector<TaskCandidate>> per_target(targets.size());
    cv::parallel_for_(cv::Range(0, int(targets.size())), [&](const cv::Range& range) {
      for (int ti = range.start; ti < range.end; ++ti) {
        const Target& t = targets[size_t(ti)];
        std::vector<TaskCandidate>& slot = per_target[size_t(ti)];
        for (const View& v : SampleViews(map, g, known, t.p, p.sampler, p.frustum, envp, in.bounds, peerp,
                                         in.footprints ? &g : nullptr)) {
            if (!In(dist, v.cell) || !std::isfinite(dist.at<double>(v.cell.r, v.cell.c))) continue;
            // D175: a frontier or uncertainty view that only sees what peers already reported is no task.
            if (peerp && v.ig.coverage == 0 && (t.src == TaskSource::kFrontier || t.src == TaskSource::kUncertainty)) continue;
            TaskCandidate c;
            c.task_id = TaskId(t.src, t.anchor), c.source = t.src;
            c.goal_xy = {v.eye.x(), v.eye.y()}, c.goal_yaw = v.yaw, c.centroid_xy = {t.p.x(), t.p.y()}, c.cells = t.size;
            c.path_m = dist.at<double>(v.cell.r, v.cell.c) * g.cell;
            c.standoff_m = (v.eye - t.p).head<2>().norm();
            c.ig_raw[kCoverage] = v.ig.coverage, c.ig_raw[kEntropy] = v.ig.entropy, c.ig_raw[kEgressConfidence] = v.ig.egress;
            c.ig_raw[kPoseConnectivity] = t.gain_conn, c.ig_raw[kLoopDisambiguation] = t.gain_alias;
            c.footprint = v.ig.footprint;
            c.cost = {c.path_m, t.src == TaskSource::kConnectivity ? p.exchange_bytes : 0.0, p.e_per_m * c.path_m,
                      c.path_m / p.speed_mps, 0.0};
            Risk(v.cell, &c, bott);
            c.cost[4] = c.risk;
            slot.push_back(std::move(c));
        }
      }
    });
    size_t n_opt = 0;
    for (const auto& s : per_target) n_opt += s.size();
    out.options.reserve(out.options.size() + n_opt);
    for (auto& s : per_target)
        for (auto& c : s) out.options.push_back(std::move(c));
    lap("views");
    out.stage_ms.emplace_back("n_targets", double(targets.size()));
    out.stage_ms.emplace_back("n_views", double(out.options.size()));

    // --- D117 executed-command time, D118 arrival twins (Ariadne policy, behind their keys) -----------------------
    const bool timed = p.cost_time && p.policy == NbvPolicy::kAriadne && in.router;
    out.timed = timed;
    if (timed) {
        const double z = st.position.z();
        // Each option is independent (its own route search on the read-only plan space, its own command time and
        // twin raycast): evaluated in parallel into per-option slots, gathered in option order, so kept, twins and
        // unroutable are exactly the serial loop's. A twin's CommandTime error is rethrown as the serial loop would:
        // the first one in option order.
        struct Slot {
            bool ok = false;
            bool has_twin = false;
            TaskCandidate twin;
            std::exception_ptr err;
        };
        std::vector<Slot> slots(out.options.size());
        cv::parallel_for_(cv::Range(0, int(out.options.size())), [&](const cv::Range& range) {
          for (int i = range.start; i < range.end; ++i) {
            TaskCandidate& c = out.options[size_t(i)];
            Slot& s = slots[size_t(i)];
            try {
                const ViewCommand cmd = (*in.router)(c.goal_xy);
                double t = 0, turn = 0;
                bool ok = cmd.ok;
                if (ok) {
                    try {
                        t = CommandTime(cmd, st.position.x(), st.position.y(), z, in.heading, true, c.goal_yaw, p.motion,
                                        tm.clear_a, g, p.rho_a, in.bounds.hi.z(), &turn);
                    } catch (const std::exception&) {
                        ok = false;   // PlanCommand refused it (zero clearance): one view never aborts the plan
                    }
                }
                if (!ok) continue;
                s.ok = true;
                c.cost[3] = t, c.turn_s = turn;
                // D118: a view the command reaches by a move is also offered at that move's heading, no final turn.
                if (p.arrival_twins && !cmd.cut && !cmd.wp.empty() && !std::isnan(cmd.wp.back().first)) {
                    const auto& b = cmd.wp.back();
                    const std::pair<double, double> a = cmd.wp.size() > 1 ? cmd.wp[cmd.wp.size() - 2]
                                                                          : std::make_pair(st.position.x(), st.position.y());
                    TaskCandidate w = c;
                    w.goal_yaw = std::atan2(b.second - a.second, b.first - a.first);
                    w.keep_heading = true;
                    const IgCounts ig = RaycastIg(map, core::Vec3(c.goal_xy.first, c.goal_xy.second, p.sampler.z),
                                                  w.goal_yaw, p.frustum, envp, in.bounds, -1, peerp,
                                                  in.footprints ? &g : nullptr);
                    w.ig_raw[kCoverage] = ig.coverage, w.ig_raw[kEntropy] = ig.entropy, w.ig_raw[kEgressConfidence] = ig.egress;
                    w.footprint = ig.footprint;
                    w.cost[3] = CommandTime(cmd, st.position.x(), st.position.y(), z, in.heading, false, 0.0, p.motion,
                                            tm.clear_a, g, p.rho_a, in.bounds.hi.z(), &w.turn_s);
                    s.twin = std::move(w);
                    s.has_twin = true;
                }
            } catch (...) {
                s.err = std::current_exception();
            }
          }
        });
        std::vector<TaskCandidate> kept, twins;
        kept.reserve(out.options.size());
        for (size_t i = 0; i < slots.size(); ++i) {
            Slot& s = slots[i];
            if (s.err) std::rethrow_exception(s.err);
            if (!s.ok) {
                ++out.unroutable;
                continue;
            }
            if (s.has_twin) twins.push_back(std::move(s.twin));
            kept.push_back(std::move(out.options[i]));
        }
        for (auto& w : twins) kept.push_back(std::move(w));
        out.options = std::move(kept);
    }
    lap("timed");

    // --- self-normalisation and utility (§21.3/§21.4, D114) ------------------------------------------------------
    std::array<double, 5> scale{};
    for (int k = 0; k < 5; ++k) {
        std::vector<double> v;
        for (const auto& c : out.options) v.push_back(c.ig_raw[size_t(k)]);
        scale[size_t(k)] = P90Scale(v);
    }
    std::vector<double> vp, vb, vr;
    for (const auto& c : out.options) vp.push_back(c.path_m), vb.push_back(c.cost[1]), vr.push_back(c.cost[4]);
    const double sp = P90Scale(vp), sb = P90Scale(vb), sr = P90Scale(vr);
    std::vector<double> vt;
    for (const auto& c : out.options) vt.push_back(c.cost[3]);
    const double st_ = timed ? P90Scale(vt) : 0.0;
    out.path_scale = sp, out.time_scale = st_;
    core::PhiloxStream rng(p.random_seed, uint64_t(std::llround(in.t_s * 1000.0)), uint64_t(agent));
    for (auto& c : out.options) {
        c.gain = 0;
        for (int k = 0; k < 5; ++k) {
            c.ig_norm[size_t(k)] = scale[size_t(k)] > 0 ? c.ig_raw[size_t(k)] / scale[size_t(k)] : 0.0;
            c.gain += c.ig_norm[size_t(k)];
        }
        const double cp = sp > 0 ? c.path_m / sp : 0.0, cb = sb > 0 ? c.cost[1] / sb : 0.0, cr = sr > 0 ? c.cost[4] / sr : 0.0;
        switch (p.policy) {
            case NbvPolicy::kAriadne:
                if (timed) {   // D117: lambda_t's third prices the executed time; at t = 0 the weights still sum to 1
                    const double ct = st_ > 0 ? c.cost[3] / st_ : 0.0;
                    c.utility = c.gain - (out.lambda_path - out.lambda_t / 3.0) * cp - out.lambda_t / 3.0 * ct - cb - cr;
                } else {
                    c.utility = c.gain - out.lambda_path * cp - cb - cr;
                }
                break;
            case NbvPolicy::kCoverageGreedy: c.utility = c.ig_norm[kCoverage]; break;
            case NbvPolicy::kUncertaintyOnly: c.utility = c.ig_norm[kEntropy]; break;
            case NbvPolicy::kRandom: c.utility = rng.NextUniform(); break;
            case NbvPolicy::kTare: c.utility = c.gain - out.lambda_path * cp - cb - cr; break;   // views ranked as
                                                                                                  // Ariadne; TARE
                                                                                                  // reorders below
        }
    }
    std::stable_sort(out.options.begin(), out.options.end(), [](const TaskCandidate& a, const TaskCandidate& b) {
        if (a.utility != b.utility) return a.utility > b.utility;
        if (a.path_m != b.path_m) return a.path_m < b.path_m;
        return a.task_id < b.task_id;
    });
    if (p.policy == NbvPolicy::kTare && !out.options.empty()) TareSchedule(plan, g, me, p, &out);
    lap("utility");
    return out;
}

std::vector<int> OpenTour(const std::vector<std::vector<double>>& D) {
    const int n = int(D.size());
    std::vector<int> order;
    std::vector<char> used(size_t(n), 0);
    used[0] = 1;
    int cur = 0;
    for (int step = 1; step < n; ++step) {   // nearest neighbour (ties: lower index)
        int best = -1;
        for (int j = 1; j < n; ++j)
            if (!used[size_t(j)] && (best < 0 || D[size_t(cur)][size_t(j)] < D[size_t(cur)][size_t(best)])) best = j;
        used[size_t(best)] = 1, order.push_back(best), cur = best;
    }
    // 2-opt on the open path 0 -> order: reverse order[i..k] when it shortens; first improvement, until none.
    auto Len = [&](const std::vector<int>& o) { return TourLength(D, o); };
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i + 1 < order.size() && !improved; ++i)
            for (size_t k = i + 1; k < order.size() && !improved; ++k) {
                std::vector<int> o = order;
                std::reverse(o.begin() + std::ptrdiff_t(i), o.begin() + std::ptrdiff_t(k) + 1);
                if (Len(o) < Len(order) - 1e-12) order = o, improved = true;
            }
    }
    return order;
}

double TourLength(const std::vector<std::vector<double>>& D, const std::vector<int>& order) {
    double s = 0;
    int cur = 0;
    for (int j : order) s += D[size_t(cur)][size_t(j)], cur = j;
    return s;
}

std::vector<int> ExactOpenTour(const std::vector<std::vector<double>>& D) {
    std::vector<int> o;
    for (int j = 1; j < int(D.size()); ++j) o.push_back(j);
    std::vector<int> best = o;
    double bl = TourLength(D, o);
    while (std::next_permutation(o.begin(), o.end())) {
        const double l = TourLength(D, o);
        if (l < bl) bl = l, best = o;
    }
    return best;
}

}  // namespace ariadne::planning
