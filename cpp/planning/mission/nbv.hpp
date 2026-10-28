#pragma once
// ---------------------------------------------------------------------------
// 4.3 next-best-view (arch §21.2-§21.4; design next-best-view.md, D114). Planning-level: no runtime, ROS or AWS type.
//
//   targets   generators A frontier clusters (PCA-split above 2 m), B low-weight surface coarse cells (bottom
//             decile), C peers' keyframes this agent has not been near, D alias places, E low-support egress runs;
//   views     one FUEL-style cylindrical sampler per target (radii x azimuths at the motion altitude, in trav_a,
//             line of sight), best `keep` by coverage;
//   IG        a 16 x 16-ray frustum bundle through the TSDF to the depth gate: coverage (unknown voxels), entropy
//             (H(p), p_occ = 0.5 e^-w free / 1 - 0.5 e^-w occupied, unknown 1 bit), egress (unknown voxels in the
//             route's body envelope); connectivity and alias from their producers (0 + a note when absent);
//   utility   each term / its candidate-set p90 (max if the p90 is 0), G = sum; U = G - lambda_path(t) C_path -
//             C_bandwidth - C_risk, costs p90-normalised, lambda_path = (1 + lambda_e + lambda_t) / 3 (D114);
//             D117 (key cost_time): lambda_t's third prices C_time, the executed command's duration, instead of
//             the path: U = G - (1 + lambda_e)/3 C_path - lambda_t/3 C_time - C_b - C_r; D118 (key view_yaw):
//             each routed view also offered at its arrival heading (own IG, no final turn);
//   hard rule E_total - E_used < 1.3 E(return) => only {return, hold}, whatever the gains.
// Deterministic: no RNG except the random baseline's Philox stream keyed by (seed, agent, replan time).
// ---------------------------------------------------------------------------
#include "planning/mission/task_candidate.hpp"
#include "planning/route/astar.hpp"
#include "planning/route/egress.hpp"
#include "planning/volumetric/frontiers3d.hpp"
#include "planning/volumetric/traversability.hpp"
#include "planning/volumetric/tsdf_map.hpp"

#include "core/trajectory/command_planner.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

namespace ariadne::planning {

struct RigFrustum {   // the left camera at the spool's 640 x 360 (D90); optical axis horizontal at the view's yaw
    double fx = 335.75, fy = 335.75, cx = 320, cy = 180;
    int width = 640, height = 360;
    double max_range_m = 2.84;   // mission_v1 depth gate
    int grid = 16;               // grid x grid rays (256: inside arch §21.3's 200-500)
    int rank_grid = 8;           // the sampler ranks views by coverage on this coarser bundle (64 rays), then the
                                 // kept views get the full bundle (replay measurement: 5.9 s / plan at 256 everywhere)
};

struct Box3 {   // world m; IG counts only voxels whose centre is inside (the briefing's extent: grid x [0, z_max])
    core::Vec3 lo = core::Vec3::Constant(-1e9), hi = core::Vec3::Constant(1e9);
};

struct IgCounts {
    int coverage = 0;        // unknown voxels reached (de-duplicated over the bundle)
    double entropy = 0;      // bits over the reached voxels
    int egress = 0;          // unknown voxels reached inside the route's body envelope
    int voxels = 0;          // distinct voxels reached
    std::vector<BlockGain> footprint;   // only when asked (RaycastIg's footprint grid): the coverage voxels per block
};

/// What this agent's peers already know, as RECEIVED (D175): a 2D mask on the briefing grid, nonzero = some peer's
/// patch has the column observed (free or occupied). Null everywhere in the centralised planner.
struct PeerKnown {
    const cv::Mat* mask = nullptr;   // CV_8U, the grid's size
    GridFrame grid;
    bool Known(const core::Vec3& p) const;
};

/// The frustum bundle from `eye` (world m) looking along `yaw` (rad, world, about +z). Rays stop at (and include)
/// the first occupied voxel. `route_env` = Pack()ed voxel indices of the route envelope, or null. With `peer`, an
/// own-unknown voxel in a peer-known column is no gain (no coverage, entropy or egress credit): the team has it.
IgCounts RaycastIg(const TsdfMap& map, const core::Vec3& eye, double yaw, const RigFrustum& f,
                   const std::unordered_set<int64_t>* route_env, const Box3& bounds = Box3{}, int grid = -1,
                   const PeerKnown* peer = nullptr, const GridFrame* footprint = nullptr);

/// Binary entropy (bits) of a known voxel of weight w under D114's mapping; 1 for unknown.
double VoxelEntropy(const TsdfVoxel* v, double w_min);

struct SamplerParams {
    std::vector<double> radii{1.0, 1.5, 2.0};
    int azimuths = 16;
    double z = 1.0;   // motion altitude (worldgen 1.0 +- 0.2 m)
    int keep = 2;
};

struct View {
    core::Vec3 eye = core::Vec3::Zero();
    double yaw = 0;
    Cell cell;
    IgCounts ig;
};

/// FUEL-style views of `target`: in trav_a, line of sight (no occupied voxel farther than one voxel from the
/// target's), the best `keep` by coverage (ties by sampling order).
std::vector<View> SampleViews(const TsdfMap& map, const GridFrame& g, const cv::Mat& trav_a, const core::Vec3& target,
                              const SamplerParams& sp, const RigFrustum& f, const std::unordered_set<int64_t>* route_env,
                              const Box3& bounds = Box3{}, const PeerKnown* peer = nullptr,
                              const GridFrame* footprint = nullptr);   // footprints from the full bundle only

struct PeerKeyframe {
    int agent = 0;
    int64_t kf_id = 0;
    core::Vec3 p = core::Vec3::Zero();
};

/// Delta lambda_2 of the inter-agent edge a view near `peer_kf_id` would enable (3.4's selector, via a runtime
/// adapter; next-best-view.md §1).
class ConnectivityOracle {
public:
    virtual ~ConnectivityOracle() = default;
    virtual double Gain(int agent, int64_t peer_kf_id) const = 0;
};

struct AliasPlace {
    int64_t id = 0;
    core::Vec3 p = core::Vec3::Zero();
    double value = 0;   // the disambiguation value of observing it (§21.3)
};

/// §17.3 alias-flagged places (no producer yet; next-best-view.md §1).
class AliasSource {
public:
    virtual ~AliasSource() = default;
    virtual std::vector<AliasPlace> Places() const = 0;
};

enum class NbvPolicy { kAriadne, kCoverageGreedy, kUncertaintyOnly, kRandom, kTare };   // next-best-view.md §5; kTare: 4.6

/// D117: the command the live loop would send for a view, from the planner's own Route (Phase4Planner): its
/// waypoints (one (NaN, NaN) = turn here, D125) and whether it was cut short of the view (D119: no final yaw).
struct ViewCommand {
    bool ok = false;   // false: no route (the loop would hold)
    std::vector<std::pair<double, double>> wp;
    bool cut = false;
};
using ViewRouter = std::function<ViewCommand(const std::pair<double, double>& goal_xy)>;

/// D117: the duration (s) of core::scripted::PlanCommand over `c` from rest at (x, y, z) with `heading` (NaN =
/// unknown: the first segment's own heading, so no initial turn is priced), ending in `yaw` when `final_yaw` and the
/// command is not cut. Speed is the mover's rule on the planner's clearance `clear_a` (CV_32F, m; floored at rho_a:
/// the mover refuses any move closer, so an executed move never has less) and the height to floor and ceiling.
/// `turn_s` (optional) receives the turning part. Throws what PlanCommand throws.
double CommandTime(const ViewCommand& c, double x, double y, double z, double heading, bool final_yaw, double yaw,
                   const core::scripted::MotionLimits& L, const cv::Mat& clear_a, const GridFrame& g, double rho_a,
                   double z_max, double* turn_s = nullptr);

struct NbvParams {
    SamplerParams sampler;
    RigFrustum frustum;
    NbvPolicy policy = NbvPolicy::kAriadne;
    uint64_t random_seed = 0;
    double tare_subspace_m = 6.0;              // D116: 12 coarse cells = one horizon width (2 x the depth gate)
    double split_extent_m = 2.0;               // FUEL's PCA split above this AABB extent
    double peer_near_m = 4.0, peer_seen_m = 2.0;
    double rho_r = 0.40, rho_a = 0.20, escape_m = 0.5;   // D158's risk mapping (agentic.md)
    double e_total_m = 1e9;                    // energy budget, in metres in the kinematic rig
    double e_per_m = 1.0, speed_mps = 0.5;
    double reserve = 1.3;
    double mission_T_s = 1e9;
    double exchange_bytes = 0;                 // §15.7 byte-table cost of a C view's exchange
    bool cost_time = false;                    // D117 (mission_v2 nbv.cost_time: exact_command); Ariadne policy only
    bool arrival_twins = false;                // D118 (nbv.view_yaw: target+arrival); needs cost_time
    core::scripted::MotionLimits motion;       // the mover's limits (worldgen Motion preset): D117's profile
    bool term_log = false;                     // D120 (nbv.term_log): the runtime traces every candidate's terms
};

struct AgentState {
    core::Vec3 position = core::Vec3::Zero();
    double e_used_m = 0;
    std::vector<core::Vec3> own_keyframes;
};

struct NbvInputs {
    const TsdfMap* map = nullptr;
    const FrontierSet3D* frontiers = nullptr;
    const CoarseLayer* coarse = nullptr;
    const TraversabilityMaps* trav = nullptr;
    GridFrame grid;
    const EgressReplan* route = nullptr;       // null: no route yet
    const cv::Mat* route_support = nullptr;
    BodyCapsule responder{0.40, 0.45, 1.40};
    std::vector<Cell> entrances;
    std::vector<PeerKeyframe> peers;
    const ConnectivityOracle* connectivity = nullptr;
    const AliasSource* alias = nullptr;
    bool risk_calibrated = false;              // D112's bar passed: the structural [PROXY] enters C_risk
    double t_s = 0;
    Box3 bounds;                               // the briefing's extent (rays leaving the building are not gain)
    const cv::Mat* peer_known = nullptr;       // D175 per-agent planning: the received layer's known mask (grid), or null
    bool footprints = false;                   // D175 R1: each option's block footprint (per-agent planning only)
    const ViewRouter* router = nullptr;        // D117: the loop's own Route for a goal (null: cost_time cannot run)
    double heading = std::numeric_limits<double>::quiet_NaN();   // D117: the agent's yaw at plan time (NaN unknown)
};

struct NbvPlan {
    std::vector<TaskCandidate> options;        // sorted by utility (desc), then path, then task_id
    bool reserve_rule = false;                 // the hard rule fired (supervisor event)
    std::string supervisor_event;
    double lambda_path = 1;
    double lambda_t = 1;                       // the time schedule's own factor (D117 splits it off the path term)
    double path_scale = 0, time_scale = 0;     // the p90 scales of C_path and C_time (D120's term log)
    int unroutable = 0;                        // D117: options dropped because Route has no command for them
    bool timed = false;                        // D117 ran: cost[3] is the executed command's time, else path / speed
    std::vector<std::string> notes;            // missing producers etc., for the trace
    int targets[5] = {0, 0, 0, 0, 0};          // per generator A..E
    std::string tare;                          // kTare: "local k" | "global m" | "" (what scheduled the first option)
    std::vector<std::pair<std::string, double>> stage_ms;   // wall ms per stage, plus option counts (profiling only)
};

NbvPlan PlanNbv(const NbvInputs& in, int agent, const AgentState& st, const NbvParams& p);

/// 4.6 (D116) open tour from node 0 over nodes 1..n-1 of a distance matrix: nearest neighbour, then 2-opt until no
/// improving move. Returns the visiting order (node indices, without 0). Infinite entries are unreachable (last).
std::vector<int> OpenTour(const std::vector<std::vector<double>>& D);
/// Its length; and the exact optimum by enumeration (the test oracle, small n only).
double TourLength(const std::vector<std::vector<double>>& D, const std::vector<int>& order);
std::vector<int> ExactOpenTour(const std::vector<std::vector<double>>& D);

/// p90 scale of `v` (sorted[floor(0.9 (n - 1))]); the max if that is 0; 0 if all are 0.
double P90Scale(std::vector<double> v);

/// D158: per cell, the widest way from `from` over `plan` (maximin of `clr` along the path, 8-connected with the
/// corner rule); cells within escape_cells of `from` count as unbounded. CV_64F, -1 unreachable.
cv::Mat Bottleneck(const cv::Mat& plan, const cv::Mat& clr, const Cell& from, double escape_cells);

/// The agent's plan space (D114 amendment, = the D153b standoff rule): trav_a AND egress clearance >= rho_agent, i.e.
/// known-free columns with rho_agent clearance to anything not observed free. trav_a alone counts unknown space as
/// passable, so views and paths ran through unobserved columns where clear_r = 0 (risk 1 on ~90 % of views).
cv::Mat KnownFreePlan(const TraversabilityMaps& tm, double rho_a);

/// The return space of §21.4's hard rule: `plan` plus the agent's swept body along its own flown track (segments
/// between consecutive track positions at body radius `body_m`, starting from the briefing entrance nearest the first
/// position: entrances are the deployment points), minus every column the slab now marks occupied (a collapse on the
/// way back is not passable). The forward rig never observes the space behind it, so the way back is not known-free
/// although the body occupied it. Only the return distance and the return option's risk read it; views and the
/// exploration plan stay known-free. `swept` (optional) receives the swept columns.
cv::Mat FlownReturnSpace(const cv::Mat& plan, const cv::Mat& slab, const GridFrame& g, const std::vector<core::Vec3>& track,
                         const std::vector<Cell>& entrances, double body_m, cv::Mat* swept = nullptr);

/// D158's arrival rule (planner.hpp): the widest way to ARRIVE within `rad` cells of the goal = the max of the
/// bottleneck map over that disc (a view goal sits at the edge of unknown space by design, so its own cell's clearance
/// is not what a responder must pass). -1 if nothing in the disc is reachable.
double ArrivalBottleneck(const cv::Mat& B, const Cell& goal, int rad);

}  // namespace ariadne::planning
