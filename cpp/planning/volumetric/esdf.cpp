#include "planning/volumetric/esdf.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace ariadne::planning {

void EdgeMinConvolution1D(const int32_t* in, int n, size_t stride, int reach, int32_t* out) {
    for (int q = 0; q < n; ++q) {
        int32_t best = in[size_t(q) * stride];
        for (int d = 1; d <= reach; ++d) {
            const int32_t g = (2 * d - 1) * (2 * d - 1);
            if (g >= best) break;   // g grows with d: nothing farther can win
            if (q - d >= 0) best = std::min(best, in[size_t(q - d) * stride] + g);
            if (q + d < n) best = std::min(best, in[size_t(q + d) * stride] + g);
        }
        out[size_t(q) * stride] = std::min(best, kEdgeInf);
    }
}

std::vector<int32_t> EdgeDistanceSq4(const std::vector<uint8_t>& blocked, int nx, int ny, int nz, int reach) {
    std::vector<int32_t> a(blocked.size()), b(blocked.size());
    for (size_t i = 0; i < blocked.size(); ++i) a[i] = blocked[i] ? 0 : kEdgeInf;
    const size_t sy = size_t(nx), sz = size_t(nx) * size_t(ny);
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j) EdgeMinConvolution1D(&a[k * sz + j * sy], nx, 1, reach, &b[k * sz + j * sy]);
    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i) EdgeMinConvolution1D(&b[k * sz + i], ny, sy, reach, &a[k * sz + i]);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) EdgeMinConvolution1D(&a[j * sy + i], nz, sz, reach, &b[j * sy + i]);
    return b;
}

Esdf::Esdf(double voxel, double d_max, double pad_m)
    : vox_(voxel), d_max_(d_max), pad_m_(pad_m), k_(int(std::ceil(d_max / voxel)) + 1) {
    if (!(d_max > 0) || 4.0 * (d_max / voxel) * (d_max / voxel) > 65535.0) {
        throw std::invalid_argument("Esdf: d_max must be in (0, ~128 voxels] for the uint16 store");
    }
}

double Esdf::Clear(uint16_t q4) const { return std::min(d_max_, std::sqrt(double(q4) / 4.0) * vox_); }

void Esdf::Rebuild(const TsdfMap& map) {
    Idx3 lo, hi;
    if (!map.Bounds(&lo, &hi)) {
        egress_.clear(), agent_.clear(), dims_ = {};
        return;
    }
    if (reserved_) {
        lo = {std::min(lo.x, res_lo_.x), std::min(lo.y, res_lo_.y), std::min(lo.z, res_lo_.z)};
        hi = {std::max(hi.x, res_hi_.x), std::max(hi.y, res_hi_.y), std::max(hi.z, res_hi_.z)};
    }
    // Horizontal margin pad_m (growth headroom) + k; vertical only k (the scenes are bounded in z). A reserved
    // extent is the headroom, so it needs only the k margin.
    const int pad = (reserved_ ? 0 : int(std::ceil(pad_m_ / vox_))) + k_;
    lo_ = {lo.x - pad, lo.y - pad, lo.z - k_};
    dims_ = {hi.x - lo.x + 1 + 2 * pad, hi.y - lo.y + 1 + 2 * pad, hi.z - lo.z + 1 + 2 * k_};
    const size_t n = size_t(dims_.x) * size_t(dims_.y) * size_t(dims_.z);
    egress_.assign(n, 0);
    agent_.assign(n, 65535);
    Recompute(map, lo_, {lo_.x + dims_.x - 1, lo_.y + dims_.y - 1, lo_.z + dims_.z - 1});
}

bool Esdf::Update(const TsdfMap& map, const std::vector<Idx3>& dirty_blocks) {
    if (dirty_blocks.empty()) {
        last_updated_ = 0;
        return true;
    }
    const int k = k_;
    Idx3 lo, hi;
    map.Bounds(&lo, &hi);
    if (Empty() || !Inside({lo.x - k, lo.y - k, lo.z - k}) || !Inside({hi.x + k, hi.y + k, hi.z + k})) {
        Rebuild(map);
        return false;
    }
    // The x-y update tiles (aligned to the box origin) that some dirty block, dilated by k, touches; one z range for
    // all of them (dirty z extent +- k). Every voxel a change can move lies in one of these tiles.
    constexpr int kUpdateTile = 48;
    const Idx3 last{lo_.x + dims_.x - 1, lo_.y + dims_.y - 1, lo_.z + dims_.z - 1};
    int zlo = 1 << 30, zhi = -(1 << 30);
    std::set<std::pair<int, int>> touched;   // (ty, tx) tile indices; ordered, so the work list is deterministic
    for (const Idx3& d : dirty_blocks) {
        const int x0 = std::max(d.x * kBlockSide - k, lo_.x), x1 = std::min((d.x + 1) * kBlockSide - 1 + k, last.x);
        const int y0 = std::max(d.y * kBlockSide - k, lo_.y), y1 = std::min((d.y + 1) * kBlockSide - 1 + k, last.y);
        zlo = std::min(zlo, d.z * kBlockSide - k), zhi = std::max(zhi, (d.z + 1) * kBlockSide - 1 + k);
        for (int ty = (y0 - lo_.y) / kUpdateTile; ty <= (y1 - lo_.y) / kUpdateTile; ++ty)
            for (int tx = (x0 - lo_.x) / kUpdateTile; tx <= (x1 - lo_.x) / kUpdateTile; ++tx) touched.insert({ty, tx});
    }
    zlo = std::clamp(zlo, lo_.z, last.z), zhi = std::clamp(zhi, lo_.z, last.z);
    std::vector<std::pair<Idx3, Idx3>> tiles;
    tiles.reserve(touched.size());
    for (const auto& [ty, tx] : touched) {
        const Idx3 tlo{lo_.x + tx * kUpdateTile, lo_.y + ty * kUpdateTile, zlo};
        tiles.push_back({tlo, {std::min(tlo.x + kUpdateTile - 1, last.x), std::min(tlo.y + kUpdateTile - 1, last.y), zhi}});
    }
    RecomputeTiles(map, tiles);
    return true;
}

void Esdf::Recompute(const TsdfMap& map, Idx3 olo, Idx3 ohi) {
    constexpr int kTile = 96;
    std::vector<std::pair<Idx3, Idx3>> tiles;
    for (int ty = olo.y; ty <= ohi.y; ty += kTile)
        for (int tx = olo.x; tx <= ohi.x; tx += kTile)
            tiles.push_back({{tx, ty, olo.z}, {std::min(tx + kTile - 1, ohi.x), std::min(ty + kTile - 1, ohi.y), ohi.z}});
    RecomputeTiles(map, tiles);
}

void Esdf::RecomputeTiles(const TsdfMap& map, const std::vector<std::pair<Idx3, Idx3>>& tiles) {
    const Idx3 last{lo_.x + dims_.x - 1, lo_.y + dims_.y - 1, lo_.z + dims_.z - 1};
    // Each tile writes only its own voxels and reads the map (const): any order, any thread, same bits.
    cv::parallel_for_(cv::Range(0, int(tiles.size())), [&](const cv::Range& r) {
        for (int i = r.start; i < r.end; ++i) {
            const auto& [tlo, thi] = tiles[size_t(i)];
            const Idx3 slo{std::max(tlo.x - k_, lo_.x), std::max(tlo.y - k_, lo_.y), std::max(tlo.z - k_, lo_.z)};
            const Idx3 shi{std::min(thi.x + k_, last.x), std::min(thi.y + k_, last.y), std::min(thi.z + k_, last.z)};
            RecomputeTile(map, slo, shi, tlo, thi);
        }
    });
    last_updated_ = 0;
    for (const auto& [tlo, thi] : tiles)
        last_updated_ += size_t(thi.x - tlo.x + 1) * size_t(thi.y - tlo.y + 1) * size_t(thi.z - tlo.z + 1);
}

void Esdf::RecomputeTile(const TsdfMap& map, Idx3 slo, Idx3 shi, Idx3 olo, Idx3 ohi) {
    const int nx = shi.x - slo.x + 1, ny = shi.y - slo.y + 1, nz = shi.z - slo.z + 1;
    const size_t n = size_t(nx) * size_t(ny) * size_t(nz);
    std::vector<uint8_t> be(n), ba(n);
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                const uint8_t s = map.State({slo.x + x, slo.y + y, slo.z + z});
                const size_t i = size_t(x) + size_t(nx) * (size_t(y) + size_t(ny) * size_t(z));
                be[i] = s != kFree;
                ba[i] = s == kOccupied;
            }
        }
    }
    const std::vector<int32_t> de = EdgeDistanceSq4(be, nx, ny, nz, k_), da = EdgeDistanceSq4(ba, nx, ny, nz, k_);
    auto Cap = [](int32_t q4) { return uint16_t(std::min<int32_t>(q4, 65535)); };
    for (int z = olo.z; z <= ohi.z; ++z) {
        for (int y = olo.y; y <= ohi.y; ++y) {
            for (int x = olo.x; x <= ohi.x; ++x) {
                const size_t i = size_t(x - slo.x) + size_t(nx) * (size_t(y - slo.y) + size_t(ny) * size_t(z - slo.z));
                egress_[At({x, y, z})] = Cap(de[i]);
                agent_[At({x, y, z})] = Cap(da[i]);
            }
        }
    }
}

double Esdf::Egress(const Idx3& v) const { return Inside(v) ? Clear(egress_[At(v)]) : 0.0; }
double Esdf::Agent(const Idx3& v) const { return Inside(v) ? Clear(agent_[At(v)]) : d_max_; }

}  // namespace ariadne::planning
