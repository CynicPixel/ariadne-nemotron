#pragma once
// ---------------------------------------------------------------------------
// D175 (MULTIAGENT_PIPELINE §1-§3): the planning side of agents that coordinate through the radio. Each agent runs
// its own Phase4Planner on its OWN keyframes (Phase4Planner::PlanOwn); what it knows of its peers it RECEIVED:
//   SubmapPatch   (SUBMAP_PATCH, class 8)  the sender's own slab + rho_responder bit, changed 32x32 blocks, 2 bits/cell
//   TaskBid       (TASK_BID, class 9)      the sender's top-M options with BuildAuction's raw terms
//   TaskConsensus (TASK_CONSENSUS, 10)     the component broker's sequential-greedy assignment
// Plain structs, no comms type: M's wire tables mirror them field for field and CommsNode carries them, lossy,
// budgeted and partitionable; every byte is counted at the serializer (invariant 6), not here.
//   SlabExporter     own maps -> versioned blocks -> SubmapPatch (only blocks changed after the receiver's ack)
//   ReceivedLayers   peers' patches placed at T_G_W(j) -> the known mask (NBV gain, PeerKnown) and the merged layer
//                    (the operator's map; its egress at rho_responder, invariant 4: OperatorEgress)
//   BuildBid / BrokerAssign / ApplyConsensus   bids -> SGA over what arrived -> the member's pick, or "uncoordinated"
// Task identity across agents is spatial (r_same; frontier ids are per-agent hashes): the stated D175 deviation from
// CBBA's shared task list. Deterministic: sim time only, no wall clock.
// ---------------------------------------------------------------------------
#include "planning/mission/allocation.hpp"
#include "planning/mission/nbv.hpp"
#include "planning/route/egress.hpp"
#include "planning/volumetric/traversability.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace ariadne::planning::multi {

constexpr int kBlock = 32;   // cells per block side (1.6 m on the 0.05 m briefing grid)

/// One cell of the exchanged layer (2 bits on the wire): the slab tri-state plus the rho_responder bit.
enum CellCode : uint8_t { kCellUnknown = 0, kCellOccupied = 1, kCellFree = 2, kCellFreeResponder = 3 };

struct SlabBlock {
    int32_t bx = 0, by = 0;        // cells [by*32, by*32+32) x [bx*32, bx*32+32) of the briefing grid
    std::vector<uint8_t> cells;    // kBlock * kBlock CellCodes, row-major (unpacked; the codec packs 2 bits/cell)
};

struct SubmapPatch {
    int32_t sender = 0;
    uint32_t epoch = 0;            // the sender's frame epoch
    uint64_t version = 0;          // the sender's map version at export
    int64_t t_ns = 0;              // sim time of the export
    std::vector<SlabBlock> blocks; // in the sender's frame W_j (the briefing grid)
};

struct BidOption {
    int64_t task_key = -1;         // the sender's task_id
    double goal_x = 0, goal_y = 0; // in G
    float ig_raw[5] = {0, 0, 0, 0, 0};
    float path_m = 0;
    float cost_bandwidth = 0, cost_risk = 0;   // D114 cost[1], cost[4]
    float risk = 0;
    uint8_t source = 0;            // TaskSource char
    std::vector<BlockGain> footprint;   // D175 R1: <= 32 blocks of the option's coverage voxels, by w
};

struct TaskBid {
    int32_t sender = 0;
    uint32_t epoch = 0;
    uint64_t seq = 0;
    int64_t t_ns = 0;
    float lambda_path = 1;
    std::vector<BidOption> options;   // descending utility, at most M
};

struct Assignment {
    int32_t agent = 0;
    int64_t task_key = -1;         // the assignee's own key
    double goal_x = 0, goal_y = 0;
};

struct TaskConsensus {
    int32_t broker = 0;
    uint32_t epoch = 0;
    uint64_t seq = 0;
    int64_t t_ns = 0;              // computed at
    int64_t bids_t_ns = 0;         // the replan the bids were from (the newest bid)
    std::vector<int32_t> members;
    std::vector<Assignment> assignments;
};

struct ComponentInfo {             // CommsNode's ComponentOf(agent): local API, not wire
    int32_t broker = 0;
    std::vector<int32_t> members;
    uint32_t epoch = 0;
};

// Allocation identity is stable across pose-graph solves. The existing wire epoch
// field carries this topology token; full broker/member equality is also checked
// before consuming consensus (the 32-bit token alone is not an identity proof).
uint32_t CoordinationToken(int32_t broker, std::vector<int32_t> members);
bool MatchesComponent(const TaskConsensus& consensus, const ComponentInfo& component);

struct PeerPose {                  // from HEARTBEAT / #ckf, in G: local API
    int32_t agent = 0;
    int64_t kf_id = 0;
    double x = 0, y = 0, z = 0;
    int64_t t_ns = 0;
};

struct Rigid2 {                    // the 2D part of T_G_W(j): planar maps under known starts (identity until a loop)
    double x = 0, y = 0, yaw = 0;
    bool Identity() const { return x == 0 && y == 0 && yaw == 0; }
};

/// The exchanged code of each grid cell from an agent's own maps (slab tri-state, rho_responder bit from trav_r).
cv::Mat EncodeCells(const TraversabilityMaps& m);

/// An agent's own layer, versioned per block.
class SlabExporter {
public:
    SlabExporter(int sender, const GridFrame& g);
    /// Re-encode from this agent's latest maps; every block whose codes changed gets the new version. Returns it.
    uint64_t Update(const TraversabilityMaps& m);
    /// The blocks changed after `since` (a receiver's acked version; 0 = everything ever observed).
    SubmapPatch Export(uint64_t since, uint32_t epoch, int64_t t_ns) const;
    uint64_t Version() const { return version_; }

private:
    int sender_;
    GridFrame g_;
    int bw_, bh_;
    cv::Mat codes_;                     // CV_8U, the grid
    std::vector<uint64_t> changed_;     // per block: the version it last changed at (0 = never observed)
    uint64_t version_ = 0;
};

/// What one agent has received from its peers, placed in G.
class ReceivedLayers {
public:
    ReceivedLayers(int self, int n_agents, const GridFrame& g);
    /// Place a peer's patch (in its frame W_j) at T_G_W(j), nearest cell; a newer block overwrites the older one.
    void Integrate(const SubmapPatch& p, const Rigid2& T_G_Wj = {});
    /// CV_8U, nonzero where some peer observed the column (free or occupied): PeerKnown's mask.
    const cv::Mat& KnownMask() const { return known_; }
    /// CellCode per cell over every peer (and `own`, if given): occupied if anyone says occupied, else free if anyone
    /// saw it free WITHOUT the responder bit, else free with the bit, else unknown. The bit survives only if every
    /// observer of the cell had it: a sender's bit also carries its 3D capsule test (an overhang only it saw), which a
    /// peer that lacks the overhang cannot overrule [DESIGN: conservative for the operator's egress, invariant 4].
    cv::Mat Merged(const cv::Mat* own = nullptr) const;
    uint64_t AckedVersion(int sender) const { return acked_.at(size_t(sender)); }

private:
    int self_;
    GridFrame g_;
    std::vector<cv::Mat> layer_;        // per peer: CellCodes in G
    std::vector<uint64_t> acked_;       // per peer: the newest version integrated
    cv::Mat known_;
};

/// D175 §8.2: the operator node's egress claim, computed on what it RECEIVED (its own codes folded in by Merged).
/// The lattice is the merged layer's responder bit AND the observed rule re-applied on the merged layer: a free cell
/// whose 2D edge clearance to the nearest non-free cell (EdgeClearance, the GT convention) is >= rho_responder. So a
/// wall one agent saw next to another agent's certified cells removes them; never more than any sender certified
/// (invariant 4). Penalty: EgressPenalty's clearance term on that clearance; support and risk are not on the wire
/// (the patch is tri-state + the bit), so the w_u and w_r terms are 0 and `confidence` is -1 = not computed.
/// D* Lite, replanned incrementally like EgressPlanner. One instance per operator node (single writer).
class OperatorEgress {
public:
    OperatorEgress(const GridFrame& g, const Cell& target, std::vector<Cell> exits, double rho_responder,
                   const EgressWeights& w = {});
    /// `merged`: CV_8U CellCodes on the briefing grid (ReceivedLayers::Merged(&own)).
    EgressReplan Replan(const cv::Mat& merged);
    const cv::Mat& Lattice() const { return trav_r_; }   // CV_8U 0/1, the last replan's
    const cv::Mat& Clearance() const { return clear_; }  // CV_32F (m), the last replan's

private:
    GridFrame g_;
    double rho_;
    EgressWeights w_;
    EgressPlanner planner_;
    cv::Mat trav_r_, clear_, ones_;
};

/// The sender's top-M options as a bid (return / hold are outside the auction, as in BuildAuction), goals in G.
TaskBid BuildBid(int sender, uint32_t epoch, uint64_t seq, int64_t t_ns, const NbvPlan& nbv, int max_options,
                 const Rigid2& T_G_W = {});

/// The broker's objective (D175 §3 rows): plain = SGA on D114 values; discount (R2) = after each assignment the other
/// options' value x (1 - P(d)), P linear from 1 at the goal to 0 at range_m (Burgard et al. 2005); overlap (R1) = the
/// coverage term replaced by its marginal over block footprints, f(S) = sum_b max_{o in S} w_ob (Lauri et al. 2020's
/// max rule on blocks; monotone submodular), cost subtracted as in D114's utility.
enum class BrokerMode { kPlain, kDiscount, kOverlap };

struct BrokerParams {
    BrokerMode mode = BrokerMode::kPlain;
    double r_same = 1.0;     // exactly-once task identity (live_v1 min_goal_m)
    double range_m = 4.0;    // R2's P(d) range: the depth gate / frustum max_range_m
};

struct BrokerRound {         // one greedy pick (discount / overlap)
    int32_t agent = 0;
    int64_t task_key = -1;
    double delta_f = 0;      // the pick's marginal of f (footprint voxels)
    double score = 0;        // what it won on
    double f = 0;            // f(S) after the pick
};

struct BrokerLog {
    std::vector<BrokerRound> rounds;
    double f = 0;            // f(S_final) on the gain proxy (footprint voxels)
    double upper_bound = 0;  // f(S) + sum over members of the best marginal given S (partition matroid, data-dependent);
                             // f / upper_bound is the a-posteriori ratio, on the gain proxy only (never exploration time)
};

/// The component broker (arch §21.7's sanctioned shortcut) over the bids that ARRIVED from its members: common scales =
/// max over bidders of each raw term's p90 (BuildAuction's rule over the received options); value = D114 G on the
/// common scales net of bandwidth and risk; score = value x exp(-rate x path) (the auction's L_t = 1 PathScore).
/// plain: tasks = options within r_same of one another (greedy, in bid order), SequentialGreedy. discount / overlap:
/// rounds of the best (member, option) under the mode's value, skipping options within r_same of an assigned goal,
/// until every member that has a feasible option is assigned. Bids from non-members or another epoch are ignored; one
/// bid per member (the newest).
TaskConsensus BrokerAssign(const std::vector<TaskBid>& bids, const ComponentInfo& comp, const BrokerParams& prm,
                           int64_t t_ns, uint64_t seq, BrokerLog* log = nullptr);
inline TaskConsensus BrokerAssign(const std::vector<TaskBid>& bids, const ComponentInfo& comp, double r_same,
                                  int64_t t_ns, uint64_t seq) {
    BrokerParams p;
    p.r_same = r_same;
    return BrokerAssign(bids, comp, p, t_ns, seq);
}

struct Pick {
    int option = -1;               // index into the agent's options, -1 none
    bool coordinated = false;
    std::string why;               // "coordinated" | "uncoordinated: <reason>"
};

/// The member's pick from the newest consensus it RECEIVED: its assigned task by key, else the option within r_same
/// of the assigned goal; otherwise (none received, older than timeout_s, another epoch, no assignment, the task gone)
/// its own best option, logged "uncoordinated".
Pick ApplyConsensus(const TaskConsensus* c, int self, uint32_t epoch, int64_t now_ns, double timeout_s,
                    const std::vector<TaskCandidate>& options, double r_same);

/// R2's cut-off rule on top of ApplyConsensus: when the pick is "uncoordinated", the agent ranks its own options by
/// utility x prod over the peers' goals in the newest consensus it holds (any age or epoch) of
/// (1 - P(d) e^(-age / tau_s)) [ASSUMPTION tau_s, logged in why]; positive utilities only are discounted.
Pick ApplyConsensusDiscounted(const TaskConsensus* c, int self, uint32_t epoch, int64_t now_ns, double timeout_s,
                              const std::vector<TaskCandidate>& options, double r_same, double range_m, double tau_s);

}  // namespace ariadne::planning::multi
