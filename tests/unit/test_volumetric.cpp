// 4.1 volumetric stack (design volumetric.md §5, D110): FH 1D and the 3D ESDF against brute force (the §37.6
// exit-gate oracle) and cv::distanceTransform, the incremental ESDF against a rebuild, the TSDF on a known room and
// against OpenCV's HashTSDF (the §20.1 oracle), frontiers against brute force and incremental against full,
// determinism, and the coarse RLE codec.
#include "planning/volumetric/clearance.hpp"
#include "planning/volumetric/esdf.hpp"
#include "planning/volumetric/frontiers3d.hpp"
#include "planning/volumetric/tsdf_map.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/ptcloud/volume.hpp>
#include <opencv2/ptcloud/volume_settings.hpp>

#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <iterator>
#include <map>
#include <set>

using namespace ariadne;
using namespace ariadne::planning;

namespace {

const double kInf = std::numeric_limits<double>::infinity();

// --- a known scene: an axis-aligned room [0,4] x [0,3] x [0,2.5] m with a pillar [1.8,2.2] x [1.3,1.7] x [0,2.5] ---
struct Box {
    core::Vec3 lo, hi;
};
// Offset off the 5 cm grid (no voxel centre sits exactly half a voxel from a wall).
const core::Vec3 kOff(0.013, 0.021, 0.007);
const Box kRoom{core::Vec3(0, 0, 0) + kOff, core::Vec3(4, 3, 2.5) + kOff};
const Box kPillar{core::Vec3(1.8, 1.3, 0) + kOff, core::Vec3(2.2, 1.7, 2.5) + kOff};

// Distance along ray c + t d (d not normalised) to the first surface: the room's walls from inside, the pillar from
// outside. Returns +inf if none.
double Hit(const core::Vec3& c, const core::Vec3& d, bool pillar) {
    double t = kInf;
    for (int i = 0; i < 3; ++i) {
        if (d[i] > 0) t = std::min(t, (kRoom.hi[i] - c[i]) / d[i]);
        if (d[i] < 0) t = std::min(t, (kRoom.lo[i] - c[i]) / d[i]);
    }
    if (pillar) {   // slab test
        double t0 = 0, t1 = kInf;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(d[i]) < 1e-15) {
                if (c[i] < kPillar.lo[i] || c[i] > kPillar.hi[i]) return t;
                continue;
            }
            double a = (kPillar.lo[i] - c[i]) / d[i], b = (kPillar.hi[i] - c[i]) / d[i];
            if (a > b) std::swap(a, b);
            t0 = std::max(t0, a), t1 = std::min(t1, b);
        }
        if (t0 <= t1 && t0 > 0) t = std::min(t, t0);
    }
    return t;
}

// Distance from p to the nearest true surface (room walls or pillar faces).
double SurfaceDistance(const core::Vec3& p) {
    double d = kInf;
    for (int i = 0; i < 3; ++i) d = std::min({d, std::abs(p[i] - kRoom.lo[i]), std::abs(kRoom.hi[i] - p[i])});
    core::Vec3 q = p;
    bool inside = true;
    for (int i = 0; i < 3; ++i) {
        q[i] = std::clamp(p[i], kPillar.lo[i], kPillar.hi[i]);
        inside = inside && p[i] >= kPillar.lo[i] && p[i] <= kPillar.hi[i];
    }
    if (!inside) return std::min(d, (p - q).norm());
    for (int i = 0; i < 3; ++i) d = std::min({d, p[i] - kPillar.lo[i], kPillar.hi[i] - p[i]});
    return d;
}

struct Cam {
    int w = 160, h = 120;
    double f = 80, cx = 79.5, cy = 59.5;
};

// Camera at c looking horizontally at yaw (rad): optical frame x right, y down, z forward.
core::SE3 Look(const core::Vec3& c, double yaw) {
    const core::Vec3 z(std::cos(yaw), std::sin(yaw), 0), x = z.cross(core::Vec3::UnitZ()), y = z.cross(x);
    core::SE3 T;
    T.R.col(0) = x, T.R.col(1) = y, T.R.col(2) = z;
    T.t = c;
    return T;
}

cv::Mat Render(const core::SE3& T, const Cam& k, bool pillar = true) {
    cv::Mat Z(k.h, k.w, CV_32F);
    for (int v = 0; v < k.h; ++v) {
        for (int u = 0; u < k.w; ++u) {
            const core::Vec3 d = T.R * core::Vec3((u - k.cx) / k.f, (v - k.cy) / k.f, 1.0);
            Z.at<float>(v, u) = float(Hit(T.t, d, pillar));   // t with d's z = 1 in camera is the depth Z
        }
    }
    return Z;
}

DepthFrame Frame(const core::SE3& T, const Cam& k, const cv::Mat& Z, const cv::Mat& S) {
    DepthFrame f;
    f.T_W_C = T, f.fx = k.f, f.fy = k.f, f.cx = k.cx, f.cy = k.cy, f.width = k.w, f.height = k.h;
    f.depth = Z.ptr<float>(), f.sigma = S.ptr<float>();
    return f;
}

std::vector<core::SE3> Poses() {
    std::vector<core::SE3> p;
    const core::Vec3 spots[] = {{0.813, 0.821, 1.107}, {3.213, 0.821, 1.107}, {3.213, 2.221, 1.107}, {0.813, 2.221, 1.107}};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) p.push_back(Look(spots[i], i * M_PI / 2 + j * 2.1));
    }
    return p;
}

TsdfParams RoomParams() {
    TsdfParams p;
    p.pixel_stride = 2;
    p.max_depth = 6.0;
    return p;
}

// Independent oracle: distance from voxel centre q to the nearest POINT of each blocked voxel's cube, found by
// clamping q into the cube (not the implementation's per-axis formula). Returns 4 d^2 (exact: half-integer offsets).
std::vector<int64_t> BruteEdgeSq4(const std::vector<uint8_t>& b, int nx, int ny, int nz) {
    std::vector<int64_t> out(b.size(), std::numeric_limits<int64_t>::max());
    std::vector<Idx3> sites;
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                if (b[size_t(x + nx * (y + ny * z))]) sites.push_back({x, y, z});
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                int64_t m = std::numeric_limits<int64_t>::max();
                for (const auto& s : sites) {
                    const double px = std::clamp(double(x), s.x - 0.5, s.x + 0.5), py = std::clamp(double(y), s.y - 0.5, s.y + 0.5),
                                 pz = std::clamp(double(z), s.z - 0.5, s.z + 0.5);
                    const double d2 = (x - px) * (x - px) + (y - py) * (y - py) + (z - pz) * (z - pz);
                    m = std::min(m, int64_t(std::llround(4 * d2)));
                }
                out[size_t(x + nx * (y + ny * z))] = m;
            }
    return out;
}

}  // namespace

TEST(Volumetric, EdgeMinConvolutionMatchesBruteForceIncludingEmptyRuns) {
    std::mt19937 rng(7);
    for (int trial = 0; trial < 500; ++trial) {
        const int n = 1 + int(rng() % 40);
        std::vector<int32_t> f(static_cast<size_t>(n)), out(static_cast<size_t>(n));
        for (auto& x : f) x = (rng() % 3 == 0) ? kEdgeInf : int32_t(rng() % 200);
        EdgeMinConvolution1D(f.data(), n, 1, n, out.data());
        for (int q = 0; q < n; ++q) {
            int64_t m = kEdgeInf;   // in[p] + 4 * (distance from q to the segment [p - 1/2, p + 1/2])^2
            for (int p = 0; p < n; ++p) {
                if (f[size_t(p)] >= kEdgeInf) continue;
                const double c = std::clamp(double(q), p - 0.5, p + 0.5);
                m = std::min<int64_t>(m, f[size_t(p)] + std::llround(4 * (q - c) * (q - c)));
            }
            ASSERT_EQ(out[size_t(q)], m) << "trial " << trial << " q " << q;
        }
    }
}

TEST(Volumetric, EsdfMatchesBruteForceNearestPointOfCubeOnRandomGrids) {   // §37.6
    std::mt19937 rng(11);
    for (int g = 0; g < 50; ++g) {
        const int nx = 6 + int(rng() % 9), ny = 6 + int(rng() % 9), nz = 3 + int(rng() % 9);
        const double density = 0.01 + 0.29 * (g / 49.0);
        std::vector<uint8_t> b(size_t(nx * ny * nz));
        std::uniform_real_distribution<double> U(0, 1);
        for (auto& x : b) x = U(rng) < density;
        const int reach = std::max({nx, ny, nz});   // uncapped: the whole grid
        const auto fast = EdgeDistanceSq4(b, nx, ny, nz, reach);
        const auto brute = BruteEdgeSq4(b, nx, ny, nz);
        for (size_t i = 0; i < b.size(); ++i) {
            const int64_t bf = brute[i] == std::numeric_limits<int64_t>::max() ? kEdgeInf : brute[i];
            ASSERT_EQ(int64_t(fast[i]), bf) << "grid " << g << " voxel " << i;
        }
    }
}

TEST(Volumetric, EsdfSliceEqualsThePlaceholdersEdgeClearance) {   // the 1.8 / GT convention, a separate implementation
    std::mt19937 rng(3);
    const int nx = 97, ny = 61;
    const double vox = 0.05, cap = 0.50;
    std::vector<uint8_t> b(size_t(nx * ny));
    cv::Mat mask(ny, nx, CV_8U);
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) mask.at<uint8_t>(y, x) = b[size_t(x + nx * y)] = rng() % 25 == 0;
    const cv::Mat ref = EdgeClearance(mask, vox, cap);
    const int reach = int(std::ceil(cap / vox)) + 1;
    const auto e = EdgeDistanceSq4(b, nx, ny, 1, reach);
    double worst = 0;
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x)
            worst = std::max(worst, std::abs(std::min(cap, std::sqrt(e[size_t(x + nx * y)] / 4.0) * vox) - ref.at<double>(y, x)));
    EXPECT_LE(worst, 1e-12);
}

TEST(Volumetric, TsdfOnAKnownRoomCarvesFreeAndPlacesSurfacesWithinAVoxel) {
    const Cam k;
    TsdfMap map(RoomParams());
    const cv::Mat S(k.h, k.w, CV_32F, cv::Scalar(0.01f));
    for (const auto& T : Poses()) {
        const cv::Mat Z = Render(T, k);
        map.Integrate(Frame(T, k, Z, S));
    }
    const double vox = map.Voxel();
    size_t occ = 0, occ_far = 0, occ_bleed = 0, free = 0, free_bad = 0, free_outside = 0, free_pillar = 0, free_near = 0;
    map.ForEachBlock([&](const Idx3& b, const TsdfBlock&) {
        for (int i = 0; i < kBlockVoxels; ++i) {
            const Idx3 v{b.x * kBlockSide + i % 8, b.y * kBlockSide + (i / 8) % 8, b.z * kBlockSide + i / 64};
            const uint8_t s = map.State(v);
            const double d = SurfaceDistance(map.Centre(v));
            const core::Vec3 c0 = map.Centre(v) - kOff;
            const bool open = c0.x() > 0 && c0.y() > 0 && c0.z() > 0 && c0.x() < 4 && c0.y() < 3 && c0.z() < 2.5 &&
                              !(c0.x() > 1.8 && c0.x() < 2.2 && c0.y() > 1.3 && c0.y() < 1.7);
            if (s == kOccupied) {
                ++occ;
                occ_far += open && d > 1.5 * vox;   // floating in open space: an error
                occ_bleed += open && d > vox && d <= 1.5 * vox;   // D45's one-voxel bleed (grazing rays): counted
            } else if (s == kFree) {
                ++free;
                // A free voxel must be in open space: inside the room, outside the pillar, off the surfaces.
                const core::Vec3 c = map.Centre(v) - kOff;
                const bool in_room = c.x() > 0 && c.y() > 0 && c.z() > 0 && c.x() < 4 && c.y() < 3 && c.z() < 2.5;
                const bool in_pillar = c.x() > 1.8 && c.x() < 2.2 && c.y() > 1.3 && c.y() < 1.7;
                free_outside += !in_room && d > 0.5 * vox;              // carved through a wall: an error
                free_pillar += in_room && in_pillar && d > 0.5 * vox;   // carved into the pillar: an error
                free_near += d <= 0.5 * vox;   // the voxel contains a surface (grazing views; reported)
                free_bad += (!in_room || in_pillar) && d > 0.5 * vox;
            }
        }
    });
    std::printf("[ ROOM     ] occupied %zu (%zu floating > 1.5 voxels from a surface, %zu one-voxel bleed); free %zu: %zu "
                "carved through a wall, %zu into the pillar, %zu (%.2f%%) containing a surface\n", occ, occ_far,
                occ_bleed, free, free_outside, free_pillar, free_near, 100.0 * free_near / free);
    EXPECT_GT(occ, 5000u);
    EXPECT_GT(free, 50000u);
    EXPECT_EQ(occ_far, 0u) << "occupied voxels floating in open space";
    EXPECT_LE(double(occ_bleed), 0.002 * occ);   // the bleed the 5 cm voxel (D110) was sized to survive
    EXPECT_EQ(free_bad, 0u) << "free voxels more than half a voxel inside structure";
    // Along-ray SDF >= perpendicular distance, so at grazing incidence a voxel that contains the wall can read free:
    // clearance is then optimistic by at most one voxel. Measured, bounded here, and stated in volumetric.md §7.
    EXPECT_LE(double(free_near), 0.03 * free);
    EXPECT_EQ(map.State(map.ToVoxel({0.813, 0.821, 1.107})), kFree);         // the camera's own voxel was carved
    EXPECT_EQ(map.State(map.ToVoxel(core::Vec3(-0.5, 1.5, 1.2) + kOff)), kUnknown);     // behind a wall by more than tau
    EXPECT_EQ(map.State(map.ToVoxel(core::Vec3(2.0, 1.5, 1.2) + kOff)), kUnknown);      // the pillar's interior is never seen
}

TEST(Volumetric, TsdfSurfaceAgreesWithOpenCvHashTsdfOracle) {   // §20.1: OpenCV kept as the correctness oracle
    const Cam k;
    TsdfParams p = RoomParams();
    p.unit_weights = true;
    p.pixel_stride = 1;
    TsdfMap map(p);
    cv::VolumeSettings vs(cv::VolumeType::HashTSDF);
    vs.setVoxelSize(float(p.voxel));
    vs.setTsdfTruncateDistance(float(p.truncation_voxels * p.voxel));
    vs.setDepthFactor(1.f);
    vs.setMaxDepth(float(p.max_depth));
    vs.setIntegrateWidth(k.w), vs.setIntegrateHeight(k.h), vs.setRaycastWidth(k.w), vs.setRaycastHeight(k.h);
    const cv::Matx33f K(float(k.f), 0, float(k.cx), 0, float(k.f), float(k.cy), 0, 0, 1);
    vs.setCameraIntegrateIntrinsics(cv::Mat(K)), vs.setCameraRaycastIntrinsics(cv::Mat(K));
    cv::Volume cvv(cv::VolumeType::HashTSDF, vs);
    auto Pose = [](const core::SE3& T) {
        cv::Matx44f m = cv::Matx44f::eye();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) m(i, j) = float(T.R(i, j));
            m(i, 3) = float(T.t(i));
        }
        return m;
    };
    for (const auto& T : Poses()) {
        const cv::Mat Z = Render(T, k);
        map.Integrate(Frame(T, k, Z, cv::Mat()));
        cvv.integrate(Z, Pose(T));
    }
    // Raycast both from a held-out pose; ours: march voxel/4, first sign change of the fused SDF among known voxels.
    const core::SE3 Tq = Look(core::Vec3(1.0, 1.5, 1.2) + kOff, 0.3);
    cv::Mat pts, nrm;
    cvv.raycast(Pose(Tq), pts, nrm);
    std::vector<double> d_ours_cv, d_ours_true, d_cv_true;
    for (int v = 0; v < k.h; v += 2) {
        for (int u = 0; u < k.w; u += 2) {
            const core::Vec3 d = Tq.R * core::Vec3((u - k.cx) / k.f, (v - k.cy) / k.f, 1.0);
            const double truth = Hit(Tq.t, d, true);
            double prev_t = -1, prev_s = 0, ours = -1;
            for (double t = 0.1; t < 6.0; t += p.voxel / 4) {
                const TsdfVoxel* x = map.Find(map.ToVoxel(Tq.t + d * t));
                if (!x || x->w <= p.w_min) {
                    prev_t = -1;
                    continue;
                }
                if (prev_t > 0 && prev_s > 0 && x->sdf <= 0) {
                    ours = prev_t + (t - prev_t) * prev_s / (prev_s - x->sdf);
                    break;
                }
                prev_t = t, prev_s = x->sdf;
            }
            const cv::Vec4f q = pts.at<cv::Vec4f>(v, u);
            if (ours < 0 || !std::isfinite(q[2]) || (q[0] == 0 && q[1] == 0 && q[2] == 0)) continue;
            // cv::Volume's raycast points are in CAMERA coordinates (KinFu's convention: its ICP consumes them as the
            // previous frame's points); q[2] is the depth. Their agreement with truth below checks that reading.
            d_ours_cv.push_back(std::abs(ours - q[2]));
            d_ours_true.push_back(std::abs(ours - truth));
            d_cv_true.push_back(std::abs(q[2] - truth));
        }
    }
    ASSERT_GT(d_ours_cv.size(), 3000u);
    auto Pct = [](std::vector<double> x, double q) {
        std::sort(x.begin(), x.end());
        return x[size_t(q * double(x.size() - 1))];
    };
    std::printf("[ ORACLE   ] %zu rays: |ours-cv| p50 %.4f p95 %.4f; |ours-true| p50 %.4f p95 %.4f; |cv-true| p50 %.4f p95 %.4f m\n",
                d_ours_cv.size(), Pct(d_ours_cv, .5), Pct(d_ours_cv, .95), Pct(d_ours_true, .5), Pct(d_ours_true, .95),
                Pct(d_cv_true, .5), Pct(d_cv_true, .95));
    // Agreement where both see a surface, within TWO voxels: the two maps' errors add, and the oracle's own p95 error
    // against truth is 1.35 voxels here (half a voxel of bias in SMOKE §4u), ours 0.7. The tail beyond is rays where
    // the two pick different surfaces; each is individually close to truth, checked next.
    const double agree = double(std::count_if(d_ours_cv.begin(), d_ours_cv.end(), [&](double x) { return x <= 2 * p.voxel; })) /
                         double(d_ours_cv.size());
    std::printf("[ ORACLE   ] agreement within two voxels: %.3f of rays\n", agree);
    EXPECT_GE(agree, 0.90);
    EXPECT_LE(Pct(d_ours_cv, 0.5), p.voxel);
    EXPECT_LE(Pct(d_ours_true, 0.95), p.voxel);
    EXPECT_LE(Pct(d_cv_true, 0.95), 2 * p.voxel);   // the oracle itself, as SMOKE §4u measured it
}

TEST(Volumetric, ReservedTiledEsdfEqualsAnUnreservedRebuildBitForBit) {
    // The live planner reserves the mission grid up front and updates by dirty tiles in parallel: neither may move
    // a single value relative to a plain rebuild of the final map.
    const Cam k;
    TsdfMap map(RoomParams());
    const cv::Mat S(k.h, k.w, CV_32F, cv::Scalar(0.01f));
    Esdf inc(map.Voxel(), 0.50);
    inc.Reserve({-80, -80, -10}, {120, 120, 70});
    int incremental = 0;
    for (const auto& T : Poses()) {
        const auto dirty = map.Integrate(Frame(T, k, Render(T, k), S));
        if (inc.Empty()) inc.Rebuild(map);
        else incremental += inc.Update(map, dirty);
    }
    EXPECT_EQ(incremental, int(Poses().size()) - 1) << "inside the reserve every update is incremental";
    Esdf full(map.Voxel(), 0.50);
    full.Rebuild(map);
    Idx3 lo, hi;
    ASSERT_TRUE(map.Bounds(&lo, &hi));
    for (int z = lo.z - 12; z <= hi.z + 12; ++z)
        for (int y = lo.y - 12; y <= hi.y + 12; ++y)
            for (int x = lo.x - 12; x <= hi.x + 12; ++x) {
                ASSERT_EQ(inc.Egress({x, y, z}), full.Egress({x, y, z})) << x << "," << y << "," << z;
                ASSERT_EQ(inc.Agent({x, y, z}), full.Agent({x, y, z})) << x << "," << y << "," << z;
            }
}

TEST(Volumetric, IncrementalEsdfEqualsARebuildBitForBit) {
    const Cam k;
    TsdfMap map(RoomParams());
    const cv::Mat S(k.h, k.w, CV_32F, cv::Scalar(0.01f));
    Esdf inc(map.Voxel(), 0.50);
    int incremental = 0;
    for (const auto& T : Poses()) {
        const cv::Mat Z = Render(T, k);
        const auto dirty = map.Integrate(Frame(T, k, Z, S));
        if (inc.Empty()) inc.Rebuild(map);
        else incremental += inc.Update(map, dirty);
    }
    EXPECT_GE(incremental, 6) << "most updates must be incremental, or the test proves nothing";
    Esdf full(map.Voxel(), 0.50);
    full.Rebuild(map);
    Idx3 lo, hi;
    ASSERT_TRUE(map.Bounds(&lo, &hi));
    size_t n = 0;
    for (int z = lo.z; z <= hi.z; ++z)
        for (int y = lo.y; y <= hi.y; ++y)
            for (int x = lo.x; x <= hi.x; ++x, ++n) {
                ASSERT_EQ(inc.Egress({x, y, z}), full.Egress({x, y, z})) << x << "," << y << "," << z;
                ASSERT_EQ(inc.Agent({x, y, z}), full.Agent({x, y, z})) << x << "," << y << "," << z;
            }
    EXPECT_GT(n, 100000u);
    // The agent field means what it says. Far from all structure (0.8 m from the pillar, >= 1 m from walls, floor and
    // ceiling) it is capped; 0.2 m from the x = 0 wall it is 0.2 m to within a voxel (edge convention).
    EXPECT_DOUBLE_EQ(full.Agent(map.ToVoxel(core::Vec3(1.0, 1.5, 1.2) + kOff)), 0.50);
    EXPECT_NEAR(full.Agent(map.ToVoxel(core::Vec3(0.2, 1.5, 1.2) + kOff)), 0.2, map.Voxel());
    // Egress counts unknown as blocked, so it is never above agent clearance.
    for (int z = lo.z; z <= hi.z; z += 3)
        for (int y = lo.y; y <= hi.y; y += 3)
            for (int x = lo.x; x <= hi.x; x += 3) ASSERT_LE(full.Egress({x, y, z}), full.Agent({x, y, z}));
}

TEST(Volumetric, FrontiersMatchBruteForceAndIncrementalMatchesFull) {
    const Cam k;
    TsdfMap map(RoomParams());
    const cv::Mat S(k.h, k.w, CV_32F, cv::Scalar(0.01f));
    FrontierParams fp;
    fp.min_voxels = 1;
    fp.z_lo = 0.6, fp.z_hi = 1.6;
    FrontierSet3D inc(fp);
    const auto poses = Poses();
    for (size_t i = 0; i < 6; ++i) {   // half the room: plenty of frontier left
        const cv::Mat Z = Render(poses[i], k);
        const auto dirty = map.Integrate(Frame(poses[i], k, Z, S));
        if (i == 0) {
            inc.Detect(map);
            continue;
        }
        Idx3 lo{1 << 30, 1 << 30, 1 << 30}, hi{-(1 << 30), -(1 << 30), -(1 << 30)};
        for (const auto& b : dirty) {
            lo = {std::min(lo.x, b.x * 8), std::min(lo.y, b.y * 8), std::min(lo.z, b.z * 8)};
            hi = {std::max(hi.x, b.x * 8 + 7), std::max(hi.y, b.y * 8 + 7), std::max(hi.z, b.z * 8 + 7)};
        }
        inc.Update(map, lo, hi);
    }
    FrontierSet3D full(fp);
    full.Detect(map);
    ASSERT_EQ(inc.Clusters().size(), full.Clusters().size());
    for (size_t i = 0; i < full.Clusters().size(); ++i) {
        ASSERT_EQ(inc.Clusters()[i].voxels, full.Clusters()[i].voxels) << "cluster " << i;
        EXPECT_EQ(inc.Clusters()[i].visible_unknown, full.Clusters()[i].visible_unknown);
    }
    // Brute force: every free voxel in the band with an unknown 6-neighbour, over the map bounds (+1).
    Idx3 lo, hi;
    ASSERT_TRUE(map.Bounds(&lo, &hi));
    const int zl = int(std::floor(fp.z_lo / map.Voxel())), zh = int(std::floor(fp.z_hi / map.Voxel()));
    std::set<Idx3> brute, got;
    for (int z = zl; z <= zh; ++z)
        for (int y = lo.y - 1; y <= hi.y + 1; ++y)
            for (int x = lo.x - 1; x <= hi.x + 1; ++x) {
                const Idx3 v{x, y, z};
                if (map.State(v) != kFree) continue;
                const int d[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
                for (const auto& o : d)
                    if (map.State({x + o[0], y + o[1], z + o[2]}) == kUnknown) {
                        brute.insert(v);
                        break;
                    }
            }
    for (const auto& c : full.Clusters()) got.insert(c.voxels.begin(), c.voxels.end());
    std::vector<Idx3> missing, extra;
    std::set_difference(brute.begin(), brute.end(), got.begin(), got.end(), std::back_inserter(missing));
    std::set_difference(got.begin(), got.end(), brute.begin(), brute.end(), std::back_inserter(extra));
    EXPECT_TRUE(missing.empty()) << missing.size() << " brute-force frontier voxels not in any cluster";
    EXPECT_TRUE(extra.empty()) << extra.size() << " cluster voxels are not frontier";
    EXPECT_GT(brute.size(), 200u);
    std::map<Idx3, size_t> owner;   // members of different clusters are never 26-adjacent
    for (size_t i = 0; i < full.Clusters().size(); ++i)
        for (const auto& v : full.Clusters()[i].voxels) owner[v] = i;
    for (const auto& [v, i] : owner)
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto it = owner.find({v.x + dx, v.y + dy, v.z + dz});
                    ASSERT_TRUE(it == owner.end() || it->second == i);
                }
}

TEST(Volumetric, IntegrationIsDeterministicAndTheCoarseRleRoundTrips) {
    const Cam k;
    const cv::Mat S(k.h, k.w, CV_32F, cv::Scalar(0.02f));
    TsdfMap a(RoomParams()), b(RoomParams());
    const auto poses = Poses();
    for (const auto& T : poses) {
        const cv::Mat Z = Render(T, k);
        a.Integrate(Frame(T, k, Z, S));
    }
    // Two independent maps from the same frames: every voxel bit-identical, and so the coarse RLE bytes.
    for (const auto& T : poses) {
        const cv::Mat Z = Render(T, k);
        b.Integrate(Frame(T, k, Z, S));
    }
    size_t compared = 0;
    a.ForEachBlock([&](const Idx3& bi, const TsdfBlock& blk) {
        for (int i = 0; i < kBlockVoxels; ++i) {
            const Idx3 v{bi.x * 8 + i % 8, bi.y * 8 + (i / 8) % 8, bi.z * 8 + i / 64};
            const TsdfVoxel* y = b.Find(v);
            ASSERT_NE(y, nullptr);
            ASSERT_EQ(std::memcmp(&blk.v[size_t(i)], y, sizeof(TsdfVoxel)), 0);
            ++compared;
        }
    });
    EXPECT_EQ(a.Blocks(), b.Blocks());
    EXPECT_GT(compared, 100000u);
    const CoarseLayer c = BuildCoarse(a);
    const std::vector<uint8_t> rle = EncodeCoarseRle(c);
    EXPECT_EQ(rle, EncodeCoarseRle(BuildCoarse(b)));
    const CoarseLayer d = DecodeCoarseRle(rle);
    ASSERT_EQ(d.cells.size(), c.cells.size());
    for (size_t i = 0; i < c.cells.size(); ++i) {
        ASSERT_EQ(d.cells[i].state, c.cells[i].state);
        ASSERT_EQ(d.cells[i].c, c.cells[i].c);
    }
    std::printf("[ COARSE   ] %zu cells (%dx%dx%d at 0.5 m) -> %zu RLE bytes\n", c.cells.size(), c.dims.x, c.dims.y,
                c.dims.z, rle.size());
    EXPECT_LT(rle.size(), c.cells.size());
}
