#pragma once
// ---------------------------------------------------------------------------
// 4.2 traversability and structural risk (arch §20.5; design traversability-risk.md, D111).
//
// Egress at rho_responder (the product, invariant 4) is the INTERSECTION of two
// tests (D111, measured on the evidence spool, traversability-risk.md §6):
//   observed  D45's accepted rule on 4.1's TSDF: the column is free in the body
//             band (some voxel in [0.15, 1.90] m observed free, none occupied) and
//             its 2D edge clearance to non-free columns is >= rho;
//   capsule   a vertical capsule of radius rho (axis floor+rho+voxel .. 1.8 - rho)
//             fits the OCCUPIED-only 3D field: a low overhang closes the column,
//             which no 2D slab can show.
// So egress never claims more than 1.8 did, with a stricter obstacle test. The
// strict 3D form (capsule on the egress field, unknown = blocked) is computed as
// trav_r_strict: a standing body's whole volume is not observable by a rig flying
// at 1.0 m with a 28-degree vertical half-FOV (scattered pinholes near the floor,
// 11-39 % unknown above 1.3 m), so it closes the target on the spool (reported).
// Agent (rho_agent, the flying rig's altitude band 0.8..1.2 m): the capsule on the
// agent field. Output on the briefing's 2D grid (the GT layer layout: row r covers
// [oy + r*cell, ...), row 0 = south).
//
// Structural risk is a [PROXY] (arch §44): two saturating counts over a 1 m disc in
// the 1.8..2.4 m band (above a standing body, under the lowest ceiling):
//   overhang    occupied voxels whose SDF normal points down (an underside),
//   unsupported occupied voxels over a column that is open beneath the band (no
//               occupied voxel below it within +-2 voxels, and observed free there),
// each / N_ref (= 100: a 0.5 m x 0.5 m slab face), clamped to 1; risk = noisy-OR.
// NOT a load calculation; uncalibrated until scored against GT risk zones.
// ---------------------------------------------------------------------------
#include "planning/volumetric/esdf.hpp"
#include "planning/volumetric/tsdf_map.hpp"

#include <opencv2/core.hpp>

namespace ariadne::planning {

struct GridFrame {   // the briefing's 2D grid (scene.json esdf block)
    double ox = 0, oy = 0, cell = 0.05;
    int cols = 0, rows = 0;
};

struct BodyCapsule {
    double rho = 0.4;
    double axis_lo = 0.45, axis_hi = 1.4;   // world z (m) of the axis' voxel centres to test
};

struct TravParams {
    BodyCapsule responder{0.40, 0.45, 1.40};   // floor + rho + voxel .. 1.8 - rho
    BodyCapsule agent{0.20, 0.80, 1.20};       // the flying rig's altitude band (worldgen motion 1.0 +- 0.2 m)
    double risk_radius = 1.0;                  // m
    double risk_z_lo = 1.8, risk_z_hi = 2.4;   // m
    double open_below_z_lo = 0.05;             // m: "open beneath" starts one voxel above the floor
    double n_ref = 100;                        // saturation count
    double slab_z_lo = 0.15, slab_z_hi = 1.90; // m: the 1.8 body band of the observed-column rule (mission_v1)
    double down_cos = 0.70710678118654752;     // n_z < -cos(45 deg)
};

struct TraversabilityMaps {
    cv::Mat trav_r, trav_a;     // CV_8U 0/1; trav_r = observed AND capsule (the egress product)
    cv::Mat trav_r_strict;      // CV_8U: capsule on the egress field (unknown = blocked); reported, not the product
    cv::Mat slab;               // CV_8U kUnknown/kFree/kOccupied, 1.8's column rule over the body band
    cv::Mat clear_r, clear_a;   // CV_32F (m): egress = min(2D observed clearance, occupied-capsule clearance);
                                // agent = min agent ESDF over its axis; both in [0, d_max]
    cv::Mat overhang, unsupported, risk;   // CV_32F in [0, 1], [PROXY]
};

TraversabilityMaps ComputeTraversability(const TsdfMap& map, const Esdf& esdf, const GridFrame& g, const TravParams& p);

/// The per-column part of ComputeTraversability, which the live planner caches across replans. A column reads the
/// map and the ESDF only within TravColumnReach voxels of itself in x-y (ESDF reach k, plus the +-2 "open beneath"
/// window and the normal's +-1), so after Integrate only columns that close to a dirty block can change.
struct TravColumns {
    cv::Mat cap_occ, clear_a, trav_r_strict, trav_a, slab, n_down, n_hang;
    bool Empty() const { return slab.empty(); }
};
int TravColumnReach(const Esdf& esdf, double voxel);
/// Recompute the columns where `mask` (CV_8U, grid-sized) is nonzero; an empty mask or empty `out` = every column.
void UpdateTravColumns(const TsdfMap& map, const Esdf& esdf, const GridFrame& g, const TravParams& p,
                       const cv::Mat& mask, TravColumns* out);
/// The whole-grid rest (2D observed clearance, risk filters) from the columns: cheap image operations.
TraversabilityMaps FinishTraversability(const TravColumns& t, const GridFrame& g, double voxel, double d_max,
                                        const TravParams& p);

/// Unit SDF normal at a voxel by central differences; false unless all 6 neighbours are known.
bool SdfNormal(const TsdfMap& map, const Idx3& v, core::Vec3* n);

}  // namespace ariadne::planning
