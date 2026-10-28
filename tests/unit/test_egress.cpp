// 4.5 egress (design egress.md §6, D113): D* Lite equals a fresh multi-source Dijkstra after every round of random
// cell flips driven through the same diff -> 3x3 UpdateVertex path the replan loop uses, and expands fewer vertices
// than the fresh searches; the cheapest (not the nearest) exit wins; the corner rule; no route cell is ever
// non-traversable (invariant 4); an obstacle closing the route reroutes it; body support counted by hand.
#include "planning/route/dstar_lite.hpp"
#include "planning/route/egress.hpp"
#include "planning/volumetric/tsdf_map.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <vector>

using namespace ariadne;
using namespace ariadne::planning;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Independent oracle: multi-source Dijkstra from the goals with its own copy of the §22.1 edge cost and corner rule.
std::vector<double> Dijkstra(const cv::Mat& trav, const cv::Mat& pen, const std::vector<Cell>& goals) {
    const int H = trav.rows, W = trav.cols;
    auto ok = [&](int r, int c) { return r >= 0 && c >= 0 && r < H && c < W && trav.at<uint8_t>(r, c); };
    std::vector<double> d(size_t(H) * size_t(W), kInf);
    using N = std::pair<double, int64_t>;
    std::priority_queue<N, std::vector<N>, std::greater<N>> q;
    for (const auto& g : goals)
        if (ok(g.r, g.c)) d[size_t(g.r) * W + g.c] = 0, q.emplace(0.0, int64_t(g.r) * W + g.c);
    while (!q.empty()) {
        const auto [du, u] = q.top();
        q.pop();
        const int r = int(u / W), c = int(u % W);
        if (du > d[size_t(u)]) continue;
        for (int a = -1; a <= 1; ++a)
            for (int b = -1; b <= 1; ++b) {
                if ((a == 0 && b == 0) || !ok(r + a, c + b)) continue;
                if (a != 0 && b != 0 && (!ok(r + a, c) || !ok(r, c + b))) continue;
                // The same per-edge arithmetic as DStarLite (IEEE + is commutative), so equality is exact.
                const double e = (a != 0 && b != 0 ? std::sqrt(2.0) : 1.0) *
                                 (1.0 + 0.5 * (double(pen.at<float>(r, c)) + double(pen.at<float>(r + a, c + b))));
                const size_t v = size_t(r + a) * W + size_t(c + b);
                if (du + e < d[v]) d[v] = du + e, q.emplace(d[v], int64_t(v));
            }
    }
    return d;
}

double PathCost(const DStarLite& ds, const std::vector<Cell>& p) {
    double s = 0;
    for (size_t i = 1; i < p.size(); ++i) s += ds.EdgeCost(p[i - 1].r, p[i - 1].c, p[i].r, p[i].c);
    return s;
}

struct World {
    cv::Mat trav, pen;
    Cell start;
    std::vector<Cell> goals;
};

World Random(std::mt19937& rng, int H, int W, double p_block) {
    World w;
    std::uniform_real_distribution<double> u(0, 1);
    w.trav = cv::Mat(H, W, CV_8U), w.pen = cv::Mat(H, W, CV_32F);
    for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c) {
            w.trav.at<uint8_t>(r, c) = u(rng) > p_block;
            w.pen.at<float>(r, c) = float(2.0 * u(rng));
        }
    w.start = {H / 2, W / 2};
    w.trav.at<uint8_t>(w.start.r, w.start.c) = 1;
    w.goals = {{0, 0}, {H - 1, W - 1}, {0, W - 1}};
    for (const auto& g : w.goals) w.trav.at<uint8_t>(g.r, g.c) = 1;
    return w;
}

}  // namespace

TEST(DStarLite, EqualsFreshDijkstraUnderCellFlips) {
    std::mt19937 rng(20261023);
    size_t incremental = 0, fresh = 0;
    int found_rounds = 0;
    for (int trial = 0; trial < 6; ++trial) {
        World w = Random(rng, 40, 50, 0.25);
        DStarLite ds(w.trav, w.pen, w.start, w.goals);
        std::uniform_int_distribution<int> R(0, w.trav.rows - 1), C(0, w.trav.cols - 1), K(1, 12);
        std::uniform_real_distribution<double> u(0, 1);
        for (int round = 0; round < 30; ++round) {
            const int k = K(rng);   // a handful of cells per round: flips, cost up, cost down
            for (int i = 0; i < k; ++i) {
                const int r = R(rng), c = C(rng);
                const double t = u(rng);
                if (t < 0.4) w.trav.at<uint8_t>(r, c) ^= 1;
                else if (t < 0.7) w.pen.at<float>(r, c) += float(3.0 * u(rng));
                else w.pen.at<float>(r, c) = float(0.5 * w.pen.at<float>(r, c));
            }
            const size_t e0 = ds.Expanded();
            ds.Update(w.trav, w.pen);
            incremental += ds.Expanded() - e0;
            const DStarLite fresh_ds(w.trav, w.pen, w.start, w.goals);
            fresh += fresh_ds.Expanded();
            const std::vector<double> d = Dijkstra(w.trav, w.pen, w.goals);
            const double want = d[size_t(w.start.r) * w.trav.cols + w.start.c];
            ASSERT_EQ(ds.Cost(), want) << "trial " << trial << " round " << round;
            if (std::isfinite(want)) {
                ++found_rounds;
                const auto p = ds.Path();
                ASSERT_FALSE(p.empty());
                EXPECT_NEAR(PathCost(ds, p), want, 1e-9 * want);
                EXPECT_TRUE(ds.GoalIndex(p.back()) >= 0);
            } else {
                EXPECT_TRUE(ds.Path().empty());
            }
        }
    }
    EXPECT_GT(found_rounds, 90);   // the sweep must mostly exercise found routes, not just "none"
    EXPECT_LT(incremental, fresh) << "incremental " << incremental << " vs fresh " << fresh;
    std::printf("[dstar] expanded: incremental %zu vs fresh %zu over 180 rounds; %d with a route\n", incremental, fresh,
                found_rounds);
}

TEST(DStarLite, CheapestExitNotNearest) {
    // Corridor row 5 of a 11 x 30 grid; the start at column 10. Exit A at column 0 (10 cells, octile-nearest),
    // exit B at column 29 (19 cells). Cells 3..6 near A cost +4 each: A costs 6 + 4 * 5 = 26 > 19.
    cv::Mat trav = cv::Mat::zeros(11, 30, CV_8U), pen = cv::Mat::zeros(11, 30, CV_32F);
    trav.row(5).setTo(1);
    pen(cv::Rect(3, 5, 4, 1)).setTo(4.f);
    DStarLite ds(trav, pen, {5, 10}, {{5, 0}, {5, 29}});
    ASSERT_TRUE(ds.Found());
    const auto p = ds.Path();
    EXPECT_EQ(ds.GoalIndex(p.back()), 1);
    EXPECT_DOUBLE_EQ(ds.Cost(), 19.0);
    const auto d = Dijkstra(trav, pen, {{5, 0}});
    EXPECT_GT(d[5 * 30 + 10], 19.0);   // the single-exit search to A is dearer, as designed
}

TEST(DStarLite, CornerRule) {
    // Two free cells touching only at a corner: no route between them.
    cv::Mat trav = cv::Mat::zeros(3, 3, CV_8U), pen = cv::Mat::zeros(3, 3, CV_32F);
    trav.at<uint8_t>(0, 0) = trav.at<uint8_t>(1, 1) = 1;
    DStarLite ds(trav, pen, {1, 1}, {{0, 0}});
    EXPECT_FALSE(ds.Found());
    EXPECT_TRUE(std::isinf(ds.EdgeCost(1, 1, 0, 0)));
    trav.at<uint8_t>(0, 1) = 1;   // one orthogonal neighbour open: still no diagonal, but the two-step route exists
    ds.Update(trav, pen);
    ASSERT_TRUE(ds.Found());
    EXPECT_DOUBLE_EQ(ds.Cost(), 2.0);
    trav.at<uint8_t>(1, 0) = 1;   // both open: the diagonal
    ds.Update(trav, pen);
    EXPECT_DOUBLE_EQ(ds.Cost(), std::sqrt(2.0));
}

TEST(DStarLite, RouteNeverLeavesTraversable) {   // invariant 4: a constraint, whatever the costs
    std::mt19937 rng(4);
    for (int trial = 0; trial < 40; ++trial) {
        World w = Random(rng, 30, 30, 0.3);
        DStarLite ds(w.trav, w.pen, w.start, w.goals);
        for (const auto& q : ds.Path()) ASSERT_TRUE(w.trav.at<uint8_t>(q.r, q.c));
    }
}

TEST(EgressPlanner, ObstacleOnRouteReroutes) {
    // A 9 x 21 room with two exits; the route takes the west door; then the west door closes.
    cv::Mat trav = cv::Mat::zeros(9, 21, CV_8U);
    trav(cv::Rect(1, 1, 19, 7)).setTo(1);
    const std::vector<Cell> exits = {{4, 0}, {4, 20}};
    trav.at<uint8_t>(4, 0) = trav.at<uint8_t>(4, 20) = 1;
    const cv::Mat clear(trav.size(), CV_32F, cv::Scalar(1.0f)), support(trav.size(), CV_32F, cv::Scalar(1.0f));
    const cv::Mat pen = EgressPenalty(clear, support, cv::Mat(), EgressWeights{});
    EXPECT_EQ(cv::countNonZero(pen), 0);
    EgressPlanner ep({4, 7}, exits, 0.05);
    const EgressReplan a = ep.Replan(trav, pen, support, cv::Mat());
    ASSERT_TRUE(a.found);
    EXPECT_EQ(a.exit, 0);
    EXPECT_EQ(a.triggers, std::vector<std::string>{"first"});
    const cv::Mat pen2 = pen + 0.5f;   // a penalty-only change (no column leaves trav_r): no obstacle trigger
    EXPECT_TRUE(ep.Replan(trav, pen2, support, cv::Mat()).triggers.empty());
    trav.at<uint8_t>(4, 1) = trav.at<uint8_t>(4, 0) = 0;   // the west doorway closes behind the route
    const EgressReplan b = ep.Replan(trav, pen, support, cv::Mat());
    ASSERT_TRUE(b.found);
    EXPECT_EQ(b.exit, 1);
    EXPECT_EQ(b.reroute.rfind("reroute: obstacle at", 0), 0u) << b.reroute;
    EXPECT_EQ(b.triggers, std::vector<std::string>{"new_obstacle"});
    for (const auto& q : b.cells) EXPECT_TRUE(trav.at<uint8_t>(q.r, q.c));
    DStarLite fresh(trav, pen, {4, 7}, exits);
    EXPECT_EQ(fresh.Cost(), b.cost_cells);
    trav.at<uint8_t>(4, 19) = 0;   // and the east one: no route, reported as none
    trav.at<uint8_t>(4, 20) = 0;
    const EgressReplan c = ep.Replan(trav, pen, support, cv::Mat());
    EXPECT_FALSE(c.found);
    EXPECT_TRUE(c.cells.empty());
}

TEST(Egress, PenaltyTermsByHand) {
    const cv::Mat clear = (cv::Mat_<float>(1, 3) << 0.5f, 0.4f, 0.0f), support = (cv::Mat_<float>(1, 3) << 1.f, 0.5f, 0.f);
    const cv::Mat risk = (cv::Mat_<float>(1, 3) << 0.f, 1.f, 0.5f);
    EgressWeights w;   // w_r = 0: risk ignored
    cv::Mat p = EgressPenalty(clear, support, risk, w);
    EXPECT_FLOAT_EQ(p.at<float>(0, 0), 0.f);
    EXPECT_FLOAT_EQ(p.at<float>(0, 1), 0.5f + 0.2f);
    EXPECT_FLOAT_EQ(p.at<float>(0, 2), 1.f + 1.f);
    w.w_r = 1;
    p = EgressPenalty(clear, support, risk, w);
    EXPECT_FLOAT_EQ(p.at<float>(0, 1), 0.5f + 0.2f + 1.f);
    EXPECT_DOUBLE_EQ(RouteConfidence(support, {{0, 0}, {0, 1}, {0, 2}}), 0.0);
    EXPECT_DOUBLE_EQ(RouteConfidence(support, {{0, 0}, {0, 1}}), 0.5);
}

TEST(Egress, BodySupportByHand) {
    // Observe every voxel of a 2 x 2 m patch from z = 0 to z = 1.0 m only. The responder capsule (rho 0.4, axis
    // 0.45..1.40) spans layers 0.05..1.80 m: support at an interior column = known voxels / envelope voxels, counted
    // here by brute force. Restricted to the band <= 1.0 m, support is exactly 1.
    TsdfParams tp;
    TsdfMap map(tp);
    for (int x = 0; x < 40; ++x)
        for (int y = 0; y < 40; ++y)
            for (int z = 0; z < 20; ++z) map.SetVoxel({x, y, z}, TsdfVoxel{0.1f, 1.f});
    GridFrame g;
    g.ox = 0, g.oy = 0, g.cell = 0.05, g.cols = 40, g.rows = 40;
    const BodyCapsule body{0.40, 0.45, 1.40};
    const cv::Mat s = BodySupport(map, g, body);
    double known = 0, total = 0;
    for (int z = -10; z < 60; ++z) {
        const double zc = (z + 0.5) * 0.05;
        const double dz = zc < 0.45 ? 0.45 - zc : (zc > 1.40 ? zc - 1.40 : 0.0);
        for (int i = -8; i <= 8; ++i)
            for (int j = -8; j <= 8; ++j)
                if ((i * i + j * j) * 0.0025 + dz * dz <= 0.16 + 1e-12) total += 1, known += (z >= 0 && z < 20);
    }
    EXPECT_NEAR(s.at<float>(20, 20), known / total, 1e-6);
    EXPECT_GT(s.at<float>(20, 20), 0.3f);
    EXPECT_LT(s.at<float>(20, 20), 0.7f);
    const cv::Mat sb = BodySupport(map, g, body, -1e9, 1.0);
    EXPECT_NEAR(sb.at<float>(20, 20), 1.0, 1e-6);
    EXPECT_LT(sb.at<float>(0, 0), 0.5f);   // a corner column: three quarters of its disc lies outside the map
}

namespace {
// The previous implementation (per layer: the known mask filter2D'd with the layer's disc, summed in float): the
// reference the exact counts must match to float precision.
cv::Mat ConvolvedSupport(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body) {
    const double vox = map.Voxel();
    const int cx0 = int(std::lround(g.ox / vox)), cy0 = int(std::lround(g.oy / vox)), R = int(std::ceil(body.rho / vox));
    cv::Mat sum = cv::Mat::zeros(g.rows, g.cols, CV_32F), known(g.rows, g.cols, CV_32F), conv;
    double total = 0;
    for (int z = int(std::floor((body.axis_lo - body.rho) / vox)); z <= int(std::floor((body.axis_hi + body.rho) / vox)); ++z) {
        const double zc = (z + 0.5) * vox;
        const double dz = zc < body.axis_lo ? body.axis_lo - zc : (zc > body.axis_hi ? zc - body.axis_hi : 0.0);
        const double r2 = body.rho * body.rho - dz * dz;
        if (r2 < 0) continue;
        cv::Mat disc = cv::Mat::zeros(2 * R + 1, 2 * R + 1, CV_32F);
        for (int i = -R; i <= R; ++i)
            for (int j = -R; j <= R; ++j)
                if ((i * i + j * j) * vox * vox <= r2 + 1e-12) disc.at<float>(i + R, j + R) = 1.f;
        if (cv::sum(disc)[0] == 0) continue;
        for (int r = 0; r < g.rows; ++r)
            for (int c = 0; c < g.cols; ++c) known.at<float>(r, c) = map.State({cx0 + c, cy0 + r, z}) != kUnknown ? 1.f : 0.f;
        cv::filter2D(known, conv, CV_32F, disc, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
        sum += conv;
        total += cv::sum(disc)[0];
    }
    return sum / total;
}

void RandomVoxels(TsdfMap* map, std::mt19937* rng, int n, int x0, int x1, int y0, int y1) {
    std::uniform_int_distribution<int> X(x0, x1), Y(y0, y1), Z(-8, 40);
    for (int k = 0; k < n; ++k) map->SetVoxel({X(*rng), Y(*rng), Z(*rng)}, TsdfVoxel{0.1f, 1.f});
}
}  // namespace

TEST(Egress, BodySupportCountsMatchTheConvolution) {
    TsdfParams tp;
    TsdfMap map(tp);
    std::mt19937 rng(7);
    RandomVoxels(&map, &rng, 20000, -5, 64, -5, 54);
    GridFrame g;
    g.ox = 0, g.oy = 0, g.cell = 0.05, g.cols = 60, g.rows = 50;
    const BodyCapsule body{0.40, 0.45, 1.40};
    const cv::Mat s = BodySupport(map, g, body), ref = ConvolvedSupport(map, g, body);
    double worst = 0;
    for (int r = 0; r < g.rows; ++r)
        for (int c = 0; c < g.cols; ++c) worst = std::max(worst, std::abs(double(s.at<float>(r, c)) - ref.at<float>(r, c)));
    EXPECT_LT(worst, 1e-5);
    EXPECT_GT(cv::countNonZero(s > 0.05f), 1000) << "the random map must give support, or this proves little";
}

TEST(Egress, PartialRecountEqualsAFullCountBitForBit) {
    // As the live planner: count once, change the map in a few places, recount only the columns within
    // BodySupportReach of the changed voxels; the counts must equal a fresh full count exactly.
    TsdfParams tp;
    TsdfMap map(tp);
    std::mt19937 rng(11);
    RandomVoxels(&map, &rng, 8000, 0, 79, 0, 69);
    GridFrame g;
    g.ox = 0, g.oy = 0, g.cell = 0.05, g.cols = 80, g.rows = 70;
    const BodyCapsule body{0.40, 0.45, 1.40};
    SupportCounts cache;
    UpdateSupportCounts(map, g, body, -1e9, 1e9, cv::Mat(), &cache);
    const int m = BodySupportReach(body, 0.05);
    for (int round = 0; round < 4; ++round) {
        cv::Mat dirty = cv::Mat::zeros(g.rows, g.cols, CV_8U);
        for (int patch = 0; patch < 2; ++patch) {
            const int x = int(rng() % 70), y = int(rng() % 60);
            RandomVoxels(&map, &rng, 300, x, x + 7, y, y + 7);
            dirty(cv::Range(std::max(y - m, 0), std::min(y + 8 + m, g.rows)),
                  cv::Range(std::max(x - m, 0), std::min(x + 8 + m, g.cols))).setTo(1);
        }
        UpdateSupportCounts(map, g, body, -1e9, 1e9, dirty, &cache);
        SupportCounts full;
        UpdateSupportCounts(map, g, body, -1e9, 1e9, cv::Mat(), &full);
        ASSERT_EQ(full.total, cache.total);
        EXPECT_EQ(cv::countNonZero(full.count != cache.count), 0) << "round " << round;
    }
}
