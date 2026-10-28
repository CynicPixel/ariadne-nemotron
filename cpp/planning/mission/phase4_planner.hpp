#pragma once
// ---------------------------------------------------------------------------
// Phase 4's planner behind the runtime's planner seam (runtime/live/planner.hpp MissionPlanner, agentic.md §0b option B), at the
// planning level: no runtime, ROS or AWS type, so replay and live drive the same object. The MissionPlanner adapter
// copies KeyframeInput -> DepthFrame, TaskCandidate -> TaskOption and these maps -> ReplanMaps, field by field.
//
//   Integrate(agent, t, frame)  4.1: TSDF -> incremental ESDF -> frontier update; the agent's track for 4.3.
//   Plan(t, agents, directives) 4.2 maps, 4.5 egress replan (D* Lite), 4.3 NBV options per active agent, then the
//                               operator directives and the mover's refusals: keep_out masks the plan space (no
//                               path or goal through it; seam review) and drops goals in it; assign/explore keep
//                               only goals inside, when one exists (the seam's semantics); priority_region scales G of
//                               every task that OBSERVES the region (goal or observed centroid inside, or the
//                               centroid within the depth gate of it: a region in unexplored space still pulls the
//                               frontier views that look into it, an LLM-run lesson); then the joint
//                               exactly-once pick by 4.4's allocator (D115: CBBA with L_t = 1 by default, broker
//                               SGA, or the greedy pick as an ablation row), event-triggered: re-run only when the
//                               task set or the topology changed.
//                               N agents (Phase 8 §42): everything is sized by the ctor's n_agents; at N = 2 every
//                               rule is the 2-agent one (peers = the other agent, JointPick, CBBA graph, optimum).
//   Route(plan, agent, i, from)A* on the agent's plan space, line-of-sight string-pulled (4-connected chords, so
//                               no corner is cut), cut at max_command_m; never a raw-cell fallback. A goal in
//                               the start cell is one (NaN, NaN) "turn here" waypoint (D125); a cut command or an
//                               arrival twin carries a NaN yaw, i.e. no final turn (D119, D118).
// Deterministic given its inputs: sim time only, no wall clock in any decision.
// ---------------------------------------------------------------------------
#include "planning/mission/allocation.hpp"
#include "planning/mission/nbv.hpp"
#include "planning/route/egress.hpp"
#include "planning/volumetric/esdf.hpp"
#include "planning/volumetric/frontiers3d.hpp"
#include "planning/volumetric/traversability.hpp"
#include "planning/volumetric/tsdf_map.hpp"

#include <opencv2/core.hpp>

#include "core/serialization/serializer.hpp"

#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ariadne::planning {

struct Phase4Briefing {
    GridFrame grid;
    Cell target;
    std::vector<Cell> entrances;   // = exits
    double z_max_m = 3.0;          // the briefing's ceiling bound (mission_v1 grid.z_max_m): IG extent
};

struct Phase4Params {
    TsdfParams tsdf;
    FrontierParams frontiers;
    TravParams trav;
    EgressWeights egress;
    double support_band_lo = -0.31, support_band_hi = 2.31;   // D113's pre-registered band (= the full envelope)
    NbvParams nbv;
    double refused_radius_m = 0.5;   // the PlaceholderParams default: a refused goal excludes options this close
    double max_command_m = 3.0;      // the PlaceholderParams default
    bool risk_calibrated = false;    // D112's bar
    enum class Allocator { kCbba, kSga, kGreedy } allocator = Allocator::kCbba;   // D115; greedy = ablation row
    int max_bundle = 1;              // D115 correction: L_t = 1 by default (CBBA = SGA exactly)
};

/// The region kinds of the Directive (planning/mission/directives.hpp), mirrored field by field.
/// goto / hold / release / cancel are runtime-level and never reach the planner.
struct RegionDirective {
    std::string id, kind;   // priority_region | keep_out | assign | explore
    int agent = -1;         // -1 = every agent
    std::vector<std::pair<double, double>> polygon;
    double weight = 1.0;    // priority_region
    bool Covers(double px, double py) const;     // even-odd, as the Directive's
    double Distance(double px, double py) const;  // 0 inside, else to the nearest edge (as the Directive's)
};

struct Phase4AgentInput {
    bool active = false;
    bool linked = true;              // can exchange with the other linked agents this replan (topology for CBBA)
    core::Vec3 position = core::Vec3::Zero();
    double flown_m = 0;
    std::vector<std::pair<double, double>> refused_goals;
    double yaw = std::numeric_limits<double>::quiet_NaN();   // D117: heading at plan time (camera optical axis), NaN unknown
};

struct Phase4AgentPlan {
    cv::Mat plan;                      // the agent's plan space at Plan time (trav_a + escape disc - keep_out), CV_8U
    cv::Mat keep_out;                  // CV_8U, nonzero = a keep_out region for this agent (Route subtracts it)
    NbvPlan nbv;                       // options after directives and refusals, sorted by utility
    std::vector<std::string> applied;  // ids of the directives that shaped them
    int excluded_refused = 0, excluded_directive = 0;
};

/// One allocation event, logged identically for every allocator (arch §21.7).
struct AllocationRecord {
    std::string method;              // cbba | sga | greedy
    std::string trigger;             // first | tasks | topology | none (reused the last assignment) | reserve_rule
    int tasks = 0, rounds = 0;
    size_t messages = 0, bytes = 0;  // CBBA: TASK_CONSENSUS wire bytes at the serializer
    double total = 0, optimum = -1;  // auction score; the exact optimum (2 agents), -1 if not computed
    std::vector<float> scales;       // the common scales (8: five IG terms, path, bandwidth, risk)
    int components = 1;              // connected components of the comms graph over the active agents
    std::string partition_note;      // set only when components > 1: what a partition does and does not model
};

struct Phase4Plan {
    double t_s = 0;
    AllocationRecord allocation;
    TraversabilityMaps maps;
    cv::Mat support;
    EgressReplan egress;
    std::vector<Phase4AgentPlan> agents;   // one per agent (Plan sizes it N)
    std::vector<int> joint;                // index into agents[a].nbv.options, -1 none; exactly-once by task_id
    std::vector<std::pair<std::string, double>> stage_ms;   // PlanOwn's wall ms per stage (profiling; never read back)
};

/// Filter and rescale one agent's options by the active region directives and the mover's refusals (in place).
void ApplyDirectives(int agent, const std::vector<RegionDirective>& dirs,
                     const std::vector<std::pair<double, double>>& refused, double refused_radius_m, double observe_m,
                     Phase4AgentPlan* ap);

/// Zero `plan` inside every keep_out polygon that applies to `agent` (cv::fillPoly on the grid).
void MaskKeepOut(int agent, const std::vector<RegionDirective>& dirs, const GridFrame& g, cv::Mat* plan);

/// Exactly-once greedy joint pick: agents pick in descending order of their best option's utility (ties: the lower
/// index first), each its best option whose task_id no earlier agent OF ITS COMPONENT took. -1 = none. `component[a]`
/// labels agent a's connected component of the comms graph (empty = one component: the whole fleet), so a partition
/// is never coordinated across. At N = 2, one component: the agent with the higher best utility picks first (ties:
/// agent 0), the other takes its best option with a different task_id.
void JointPick(Phase4Plan* p, const std::vector<int>& component = {});

/// D115's auction instance from both agents' options: common scales (max over agents of each term's p90), per agent
/// and task its best view (by D114 utility), cbar = the D114 value on the common scales net of bandwidth and risk,
/// d0 = its path, rate = lambda_path / P_path; legs (goal -> goal on the agent's plan space) only when L_t > 1.
/// `view[a][j]` is the option index of agent a's view of task j (-1 none).
AuctionInstance BuildAuction(const Phase4Plan& p, const GridFrame& g, int max_bundle, std::vector<float>* scales,
                             std::vector<std::vector<int>>* view);

/// Line-of-sight string pulling of a grid path over `plan` (CV_8U): from each kept cell, the farthest later path
/// cell whose 4-connected chord is all traversable. Returns the kept cells, first and last included.
std::vector<Cell> StringPull(const cv::Mat& plan, const std::vector<Cell>& path);

class Phase4Planner {
public:
    Phase4Planner(const Phase4Briefing& b, const Phase4Params& p, int n_agents = 2);
    /// Returns the number of dirty blocks. Throws std::out_of_range unless 0 <= agent < n_agents.
    size_t Integrate(int agent, int64_t t_ns, const DepthFrame& f);
    /// `agents` has one entry per agent (throws std::invalid_argument otherwise).
    Phase4Plan Plan(double t_s, const std::vector<Phase4AgentInput>& agents, const std::vector<RegionDirective>& dirs);
    int Agents() const { return int(track_.size()); }
    /// How much of agent b's track agent a has received (its NBV peers): synced at replans while they share a component.
    size_t SharedTrack(int a, int b) const { return shared_.at(size_t(a)).at(size_t(b)); }
    /// D175 per-agent planning: this planner is agent `self`'s OWN, fed only its own keyframes. Maps and egress on
    /// that map, then NBV for `self` with the poses it RECEIVED (`peers`) and the received layer's known mask
    /// (`peer_known`, null = none); no allocation (agents[self] only; the component's broker assigns).
    Phase4Plan PlanOwn(double t_s, int self, const Phase4AgentInput& in, const std::vector<PeerKeyframe>& peers,
                       const cv::Mat* peer_known, const std::vector<RegionDirective>& dirs);
    std::pair<std::vector<std::pair<double, double>>, double> Route(const Phase4Plan& p, int agent, int i,
                                                                   const Cell& from) const;
    const TsdfMap& Map() const { return map_; }
    const Esdf& Distance() const { return esdf_; }
    const FrontierSet3D& Frontiers() const { return fr_; }

private:
    cv::Mat PlanSpace(const TraversabilityMaps& m, const Cell& at) const;   // trav_a + the escape disc (D158)
    void PlanMaps(Phase4Plan* out);   // traversability, body support, the egress replan (Plan and PlanOwn alike)
    Phase4AgentPlan PlanAgent(double t_s, int a, const Phase4AgentInput& in, std::vector<PeerKeyframe> peers,
                              const cv::Mat* peer_known, bool footprints, const std::vector<RegionDirective>& dirs, const Phase4Plan& shared,
                              const CoarseLayer& coarse) const;
    Cell CellOf(const core::Vec3& p) const;
    /// Route's command on a given plan space (shared by Route and D117's router, so C_time prices what is sent).
    ViewCommand Command(const cv::Mat& plan, const std::pair<double, double>& goal_xy, const Cell& from) const;

    Phase4Briefing b_;
    Phase4Params p_;
    TsdfMap map_;
    Esdf esdf_;
    TravColumns trav_cols_;   // cached per-column traversability; only columns near new map changes are recomputed
    cv::Mat trav_dirty_;      // CV_8U grid mask: columns to recompute at the next PlanMaps (empty = all)
    SupportCounts support_;   // cached body-support counts (egress.hpp), recounted where support_dirty_ is set
    cv::Mat support_dirty_;   // CV_8U grid mask, as trav_dirty_ (the counts read only the map, never the ESDF)
    FrontierSet3D fr_;
    EgressPlanner egress_;
    std::vector<std::vector<core::Vec3>> track_;   // per agent
    bool detected_ = false;
    void Allocate(const std::vector<Phase4AgentInput>& agents, const std::vector<std::vector<int>>& G,
                  const std::vector<int>& component, Phase4Plan* out);
    core::WireCodec codec_;
    std::set<int64_t> last_tasks_;
    std::vector<std::vector<int>> last_graph_;   // the CBBA comms graph at the last event (the topology trigger)
    bool allocated_ = false;
    std::vector<int64_t> last_pick_;   // task ids assigned at the last event, per agent
    std::vector<std::vector<size_t>> shared_;   // shared_[a][b]: how much of b's track a holds (synced while connected)
};

}  // namespace ariadne::planning
