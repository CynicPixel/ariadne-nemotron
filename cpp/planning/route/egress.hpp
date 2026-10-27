#pragma once
// ---------------------------------------------------------------------------
// 4.5 egress (arch §22; design EGRESS_4.5, D113): the responder's route from the target to any exit over
// D111's egress lattice at rho_responder (invariant 4), replanned by D* Lite as the map grows.
//
//   support(col)  fraction of the standing-body envelope (voxels within rho of the capsule axis) that is
//                 observed (TSDF weight > w_min), optionally restricted to a z band (EGRESS_4.5 §7: the
//                 pre-registered fallback if support is flat along the route);
//   penalty(col)  w_u (1 - support) + w_r risk + w_c max(0, rho_safe - clearance) / rho_safe   (§22.1);
//                 w_r = 0 until D112's calibration passes its bar (the risk term is a [PROXY]);
//   confidence    5th percentile of support over the route's cells (§22.2);
//   lattice       trav_r only: a non-traversable column has no edge, whatever the costs (a constraint, not
//                 a term; invariant 4).
// ---------------------------------------------------------------------------
#include "planning/route/dstar_lite.hpp"
#include "planning/volumetric/traversability.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ariadne::planning {

/// CV_32F in [0, 1] on the grid: observed fraction of a vertical capsule of radius `body.rho` whose axis
/// spans [body.axis_lo, body.axis_hi] (m), counting only voxel layers whose centre lies in [band_lo, band_hi].
/// Voxels outside the map count as unobserved. Per layer: the known mask convolved with the layer's disc, counted
/// exactly in integers (count / total, one rounding), so a partial recompute equals a full one bit for bit.
cv::Mat BodySupport(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body, double band_lo = -1e9,
                    double band_hi = 1e9);

/// The live planner's cache: the integer known-voxel count of every column's envelope. A column's count reads the
/// map only within BodySupportReach voxels of itself in x-y, so after Integrate only columns that close to a dirty
/// block can change.
struct SupportCounts {
    cv::Mat count;   // CV_32S, the grid's size
    int total = 0;   // envelope voxels (the same for every column)
    bool Empty() const { return count.empty(); }
};
int BodySupportReach(const BodyCapsule& body, double voxel);
/// Recount the columns where `mask` (CV_8U, grid-sized) is nonzero; an empty mask or empty `out` = every column.
void UpdateSupportCounts(const TsdfMap& map, const GridFrame& g, const BodyCapsule& body, double band_lo,
                         double band_hi, const cv::Mat& mask, SupportCounts* out);
/// count / total as CV_32F in [0, 1] (all zero when the band holds no envelope voxel).
cv::Mat SupportFromCounts(const SupportCounts& c);

struct EgressWeights {
    double w_u = 1, w_r = 0, w_c = 1;   // fixed a priori (EGRESS_4.5 §2); w_r = 0 unless D112 passes
    double rho_safe = 0.5;              // m: d_max = rho_r + 2 voxels
};

/// CV_32F penalty m(col) >= 0 from D111's egress clearance (CV_32F, m), support, and risk (CV_32F or empty).
cv::Mat EgressPenalty(const cv::Mat& clear_r, const cv::Mat& support, const cv::Mat& risk, const EgressWeights& w);

/// 5th percentile of `support` over `cells` (the 1.8 index rule: sorted[floor(0.05 (n - 1))]); 0 if empty.
double RouteConfidence(const cv::Mat& support, const std::vector<Cell>& cells);

struct EgressReplan {
    bool found = false;
    std::vector<Cell> cells;          // target ... exit
    int exit = -1;                    // index into the exit list
    double cost_cells = 0;            // D* Lite g(target)
    double length_m = 0;              // geometric length of `cells`
    double confidence = 0;            // RouteConfidence over `cells`
    size_t changed_cells = 0;         // cells whose (traversable, penalty) changed since the previous replan
    size_t expanded = 0;              // vertices popped by this replan
    std::vector<std::string> triggers;   // §22.3: "first", "new_obstacle", "confidence_fell", "risk_changed"
    std::string reroute;              // "reroute: obstacle at (r, c)" when a previous route cell left trav_r
};

class EgressPlanner {
public:
    EgressPlanner(const Cell& target, std::vector<Cell> exits, double cell_m) : target_(target), exits_(std::move(exits)), cell_m_(cell_m) {}
    /// One replan on the current maps: `trav_r` CV_8U, `penalty` CV_32F (EgressPenalty), `support` CV_32F,
    /// `risk` CV_32F or empty (for the risk_changed trigger only).
    EgressReplan Replan(const cv::Mat& trav_r, const cv::Mat& penalty, const cv::Mat& support, const cv::Mat& risk);

private:
    Cell target_;
    std::vector<Cell> exits_;
    double cell_m_;
    std::unique_ptr<DStarLite> ds_;
    cv::Mat prev_risk_, prev_trav_;
    std::vector<Cell> prev_route_;
    double prev_conf_ = -1;
};

}  // namespace ariadne::planning
