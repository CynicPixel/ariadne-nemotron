#pragma once
// ---------------------------------------------------------------------------
// 4.1 ESDF for clearance (arch §20.3; design VOLUMETRIC_4.1 §3.3, D110).
//
// Clearance = distance from a voxel centre to the NEAREST POINT of a blocked voxel
// (its cube), the convention of the scene GT and of 1.8 (worldgen raster._edt_to,
// planning::EdgeClearance): per axis an offset of D voxels contributes
// max(|D| - 1/2, 0), and d^2 is the sum of the squares. (D110 as first committed
// used edt * voxel - voxel / 2, exact only along an axis and optimistic on
// diagonals: it produced 4 false egress claims at GT clearance 0.395 m, corrected.)
//
// Felzenszwalb & Huttenlocher's lower envelope (arch §20.3) is exact for the
// centre-to-centre metric only; the cube metric is (2|D| - 1)^2 / 4 per axis, so
// each axis pass is an exact windowed min-convolution over the capped reach
// k = ceil(d_max / voxel) + 1 (a true distance <= d_max has every axis offset
// <= d_max / voxel + 1/2). Separable like FH: x, then y, then z. Stored as 4 d^2
// (an integer) in uint16, min(., 65535).
//
// Two fields: egress (blocked = occupied or unknown: "observed free", D45) and
// agent (blocked = occupied). Capped at d_max.
//
// Incremental, never global per keyframe: with a capped field a change at a
// voxel moves only voxels within d_max of it, so an update recomputes only the
// x-y tiles (kUpdateTile voxels) that a dirty block dilated by k touches, each
// tile reading sites from its own k margin, and writes back only the tile. Exact
// for the capped field (bit-equal to a rebuild; tested). A frustum's dirty set
// fills a fraction of its bounding box, so tiles beat the one-box update.
// Tiles are independent (disjoint writes, const map reads): cv::parallel_for_.
// Reserve() sizes the box to a known extent (the mission grid) up front, so map
// growth inside it never triggers a global rebuild.
// ---------------------------------------------------------------------------
#include "planning/volumetric/tsdf_map.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace ariadne::planning {

constexpr int32_t kEdgeInf = 1 << 28;

/// One axis pass: out[q] = min over |D| <= reach of in[q + D] + (2|D| - 1)^2 (D != 0), in[q] (D = 0); n samples,
/// stride apart; kEdgeInf = no site. Values are 4 d^2 (voxel units).
void EdgeMinConvolution1D(const int32_t* in, int n, size_t stride, int reach, int32_t* out);

/// 4 x squared cube-edge distance (voxel units) of a dense nx*ny*nz grid (x fastest), exact for distances whose
/// every axis offset is <= reach (kEdgeInf beyond). Blocked voxels are 0.
std::vector<int32_t> EdgeDistanceSq4(const std::vector<uint8_t>& blocked, int nx, int ny, int nz, int reach);

class Esdf {
public:
    /// d_max: clearance cap (m); pad_m: extra margin kept around the map so growth rarely forces a rebuild.
    Esdf(double voxel, double d_max, double pad_m = 2.0);

    /// Global recompute over the map's bounds (+ margin), and over the reserved extent if any. Used at start, on box
    /// growth, and by the tests.
    void Rebuild(const TsdfMap& map);
    /// Keep at least voxels [lo, hi] inside the box (applied at the next Rebuild). Values are unchanged by it:
    /// inside the box with nothing mapped, egress is 0 and agent d_max, exactly as outside the box.
    void Reserve(const Idx3& lo, const Idx3& hi) { res_lo_ = lo, res_hi_ = hi, reserved_ = true; }
    /// Incremental update after Integrate returned dirty_blocks. Rebuilds instead when the map outgrew the box.
    /// Returns true if it was incremental.
    bool Update(const TsdfMap& map, const std::vector<Idx3>& dirty_blocks);

    /// Clearance (m) at a voxel, in [0, d_max]. Outside the box: egress 0 (unknown), agent d_max (nothing seen).
    double Egress(const Idx3& v) const;
    double Agent(const Idx3& v) const;
    double DMax() const { return d_max_; }
    bool Empty() const { return egress_.empty(); }
    Idx3 Lo() const { return lo_; }
    Idx3 Dims() const { return dims_; }
    size_t LastUpdatedVoxels() const { return last_updated_; }

private:
    void Recompute(const TsdfMap& map, Idx3 out_lo, Idx3 out_hi);   // tiled; sites from each tile (+) k
    void RecomputeTiles(const TsdfMap& map, const std::vector<std::pair<Idx3, Idx3>>& tiles);   // in parallel
    void RecomputeTile(const TsdfMap& map, Idx3 site_lo, Idx3 site_hi, Idx3 out_lo, Idx3 out_hi);
    double Clear(uint16_t q4) const;
    size_t At(const Idx3& v) const {
        return size_t(v.x - lo_.x) + size_t(dims_.x) * (size_t(v.y - lo_.y) + size_t(dims_.y) * size_t(v.z - lo_.z));
    }
    bool Inside(const Idx3& v) const {
        return v.x >= lo_.x && v.y >= lo_.y && v.z >= lo_.z && v.x < lo_.x + dims_.x && v.y < lo_.y + dims_.y &&
               v.z < lo_.z + dims_.z;
    }
    double vox_, d_max_, pad_m_;
    int k_;   // ceil(d_max / voxel) + 1: the reach of a capped change, in voxels
    Idx3 lo_, dims_;
    Idx3 res_lo_, res_hi_;
    bool reserved_ = false;
    std::vector<uint16_t> egress_, agent_;   // min(4 d^2, 65535), voxel units (cube-edge convention)
    size_t last_updated_ = 0;
};

}  // namespace ariadne::planning
