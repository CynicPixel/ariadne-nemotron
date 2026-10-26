// D175 per-agent planning (planning/mission/multiagent.hpp): the exchanged cell code; the exporter sends only blocks
// changed since the receiver's ack; received layers place patches at T_G_W(j), refuse an agent's own patch, and merge
// conservatively (the responder bit an AND); the operator's egress routes only over cells every observer certified and
// clear of every reported wall; the broker equals the centralised SGA on the same instance under full delivery, is
// exactly-once by r_same, and ignores bids from another epoch or a non-member; a member's pick follows the consensus it
// received, or falls back "uncoordinated" for each stated reason; PlanOwn without received data equals Plan's own-agent
// output; a peer-known column is no gain; N agents' barrier steps on N threads equal the serial run byte for byte.
#include "planning/mission/multiagent.hpp"
#include "planning/mission/phase4_planner.hpp"
#include "planning/volumetric/occupancy_grid.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

using namespace ariadne;
using namespace ariadne::planning;
using namespace ariadne::planning::multi;

namespace {

GridFrame Grid(int rows = 64, int cols = 96) {
    GridFrame g;
    g.ox = 0, g.oy = 0, g.cell = 0.05, g.rows = rows, g.cols = cols;
    return g;
}

TraversabilityMaps Maps(const GridFrame& g) {
    TraversabilityMaps m;
    m.slab = cv::Mat(g.rows, g.cols, CV_8U, cv::Scalar(kUnknown));
    m.trav_r = cv::Mat::zeros(g.rows, g.cols, CV_8U);
    return m;
}

TaskCandidate Opt(int64_t id, double x, double y, double cov, double path, double util) {
    TaskCandidate c;
    c.task_id = id, c.goal_xy = {x, y}, c.utility = util, c.path_m = path;
    c.ig_raw = {cov, 0.5 * cov, 0, 0, 0};
    return c;
}

DepthFrame WallFrame(std::vector<float>* Z, std::vector<float>* S, double y) {
    const int W = 160, H = 90;
    Z->assign(size_t(W * H), 2.0f), S->assign(size_t(W * H), 0.02f);
    DepthFrame f;
    f.T_W_C.R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    f.T_W_C.t = core::Vec3(0.5, y, 1.0);
    f.fx = 84, f.fy = 84, f.cx = 80, f.cy = 45, f.width = W, f.height = H, f.depth = Z->data(), f.sigma = S->data();
    return f;
}

Phase4Briefing Brief() {
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 80, b.grid.rows = 80;
    b.target = {40, 70};
    b.entrances = {{40, 10}};
    return b;
}

}  // namespace

TEST(MultiAgentPlanning, EncodeCellsTriStatePlusResponderBit) {
    const GridFrame g = Grid(4, 4);
    TraversabilityMaps m = Maps(g);
    m.slab.at<uint8_t>(0, 0) = kOccupied;
    m.slab.at<uint8_t>(0, 1) = kFree;
    m.slab.at<uint8_t>(0, 2) = kFree, m.trav_r.at<uint8_t>(0, 2) = 1;
    const cv::Mat c = EncodeCells(m);
    EXPECT_EQ(c.at<uint8_t>(0, 0), kCellOccupied);
    EXPECT_EQ(c.at<uint8_t>(0, 1), kCellFree);
    EXPECT_EQ(c.at<uint8_t>(0, 2), kCellFreeResponder);
    EXPECT_EQ(c.at<uint8_t>(0, 3), kCellUnknown);
}

TEST(MultiAgentPlanning, ExporterSendsOnlyBlocksChangedSinceTheAck) {
    const GridFrame g = Grid();   // 96 x 64 cells = 3 x 2 blocks
    SlabExporter ex(1, g);
    TraversabilityMaps m = Maps(g);
    EXPECT_TRUE(ex.Export(0, 0, 0).blocks.empty()) << "nothing observed yet";
    m.slab.at<uint8_t>(5, 5) = kFree;         // block (0, 0)
    m.slab.at<uint8_t>(40, 70) = kOccupied;   // block (2, 1)
    const uint64_t v1 = ex.Update(m);
    EXPECT_EQ(v1, 1u);
    const SubmapPatch all = ex.Export(0, 3, 5000);
    ASSERT_EQ(all.blocks.size(), 2u);
    EXPECT_EQ(all.sender, 1);
    EXPECT_EQ(all.epoch, 3u);
    EXPECT_EQ(all.version, 1u);
    EXPECT_EQ(ex.Update(m), 1u) << "no change, no new version";
    m.slab.at<uint8_t>(6, 5) = kFree;         // block (0, 0) again
    EXPECT_EQ(ex.Update(m), 2u);
    const SubmapPatch inc = ex.Export(v1, 3, 10000);
    ASSERT_EQ(inc.blocks.size(), 1u) << "only the block changed after the acked version";
    EXPECT_EQ(inc.blocks[0].bx, 0);
    EXPECT_EQ(inc.blocks[0].by, 0);
    EXPECT_EQ(inc.blocks[0].cells[size_t(6 * kBlock + 5)], kCellFree);
    EXPECT_TRUE(ex.Export(2, 3, 10000).blocks.empty());
}

TEST(MultiAgentPlanning, ReceivedLayersPlaceMergeAndRefuseOwn) {
    const GridFrame g = Grid();
    SlabExporter e1(1, g), e2(2, g);
    TraversabilityMaps m1 = Maps(g), m2 = Maps(g);
    m1.slab.at<uint8_t>(3, 3) = kFree, m1.trav_r.at<uint8_t>(3, 3) = 1;
    m1.slab.at<uint8_t>(4, 4) = kFree;
    m2.slab.at<uint8_t>(4, 4) = kOccupied;   // the peers disagree: the merge is conservative
    m1.slab.at<uint8_t>(6, 6) = kFree, m1.trav_r.at<uint8_t>(6, 6) = 1;
    m2.slab.at<uint8_t>(6, 6) = kFree;       // free, but agent 2 did not certify it at rho_responder
    e1.Update(m1), e2.Update(m2);
    ReceivedLayers R(0, 3, g);
    R.Integrate(e1.Export(0, 0, 0));
    R.Integrate(e2.Export(0, 0, 0));
    EXPECT_EQ(R.AckedVersion(1), 1u);
    EXPECT_NE(R.KnownMask().at<uint8_t>(3, 3), 0);
    EXPECT_EQ(R.KnownMask().at<uint8_t>(10, 10), 0);
    const cv::Mat M = R.Merged();
    EXPECT_EQ(M.at<uint8_t>(3, 3), kCellFreeResponder);
    EXPECT_EQ(M.at<uint8_t>(4, 4), kCellOccupied) << "occupied wins";
    EXPECT_EQ(M.at<uint8_t>(6, 6), kCellFree) << "the responder bit is an AND over the cell's observers";
    EXPECT_THROW(R.Integrate(SlabExporter(0, g).Export(0, 0, 0)), std::invalid_argument);
    ReceivedLayers S(0, 3, g);   // T_G_W(1) = one block (1.6 m) along +x: the patch lands one block over
    S.Integrate(e1.Export(0, 0, 0), Rigid2{1.6, 0, 0});
    EXPECT_EQ(S.KnownMask().at<uint8_t>(3, 3), 0);
    EXPECT_NE(S.KnownMask().at<uint8_t>(3, 3 + kBlock), 0);
}

TEST(MultiAgentPlanning, BrokerEqualsCentralisedSgaUnderFullDelivery) {
    // Two agents, every option bid (M = all), distinct tasks farther apart than r_same: the broker's instance is
    // BuildAuction's (same scales, same values), so its sequential greedy picks what the centralised SGA picks.
    Phase4Plan p;
    p.agents.resize(2);
    p.agents[0].nbv.options = {Opt(11, 1.0, 1.0, 50, 2, 0.9), Opt(12, 4.0, 1.0, 40, 3, 0.5), Opt(13, 7.0, 1.0, 5, 1, 0.1)};
    p.agents[1].nbv.options = {Opt(11, 1.0, 1.0, 60, 4, 0.8), Opt(14, 4.0, 4.0, 45, 1, 0.7), Opt(12, 4.0, 1.0, 30, 2, 0.2)};
    for (auto& a : p.agents) a.nbv.lambda_path = 0.5;
    std::vector<float> sc;
    std::vector<std::vector<int>> view;
    GridFrame g;
    g.cell = 0.05;
    const AuctionInstance in = BuildAuction(p, g, 1, &sc, &view);
    const Allocation central = SequentialGreedy(in);
    std::vector<TaskBid> bids;
    for (int a = 0; a < 2; ++a) bids.push_back(BuildBid(a, 7, 1, 5000000000, p.agents[size_t(a)].nbv, 100));
    const TaskConsensus c = BrokerAssign(bids, ComponentInfo{0, {0, 1}, 7}, 1.0, 5000000000, 1);
    ASSERT_EQ(c.assignments.size(), 2u);
    for (const Assignment& x : c.assignments) {
        const int j = central.agents[size_t(x.agent)].path.front();
        EXPECT_EQ(x.task_key, in.task_ids[size_t(j)]) << "agent " << x.agent;
    }
    EXPECT_EQ(c.bids_t_ns, 5000000000);
}

TEST(MultiAgentPlanning, BrokerIsExactlyOnceByRSameAndIgnoresStrangers) {
    // Both agents' best goals are 0.3 m apart: one task. A bid from another epoch and one from a non-member are ignored.
    NbvPlan n0, n1, n2, n9;
    n0.options = {Opt(101, 2.0, 2.0, 50, 1, 0.9), Opt(102, 6.0, 2.0, 10, 1, 0.2)};
    n1.options = {Opt(201, 2.3, 2.0, 50, 1, 0.9), Opt(202, 2.0, 6.0, 10, 1, 0.2)};
    n2.options = {Opt(301, 9.0, 9.0, 99, 1, 0.9)};
    n9.options = {Opt(901, 9.0, 9.0, 99, 1, 0.9)};
    std::vector<TaskBid> bids = {BuildBid(0, 4, 1, 10, n0, 8), BuildBid(1, 4, 1, 10, n1, 8), BuildBid(2, 3, 1, 10, n2, 8),
                                 BuildBid(9, 4, 1, 10, n9, 8)};
    const TaskConsensus c = BrokerAssign(bids, ComponentInfo{0, {0, 1, 2}, 4}, 1.0, 20, 2);
    std::vector<std::pair<double, double>> goals;
    for (const Assignment& a : c.assignments) {
        EXPECT_NE(a.agent, 9) << "not a member";
        EXPECT_NE(a.agent, 2) << "its bid is from another epoch";
        for (const auto& [x, y] : goals) EXPECT_GE(std::hypot(a.goal_x - x, a.goal_y - y), 1.0) << "exactly-once by r_same";
        goals.push_back({a.goal_x, a.goal_y});
    }
    EXPECT_EQ(c.assignments.size(), 2u);
}

TEST(MultiAgentPlanning, ApplyConsensusFollowsWhatArrivedOrSaysWhyNot) {
    const std::vector<TaskCandidate> o = {Opt(5, 1, 1, 9, 1, 0.9), Opt(6, 3, 3, 5, 1, 0.5), Opt(8, 6, 6, 1, 1, 0.1)};
    TaskConsensus c;
    c.epoch = 2, c.t_ns = 5000000000, c.assignments = {{1, 6, 3, 3}};
    Pick p = ApplyConsensus(&c, 1, 2, 10000000000, 12.0, o, 1.0);
    EXPECT_TRUE(p.coordinated);
    EXPECT_EQ(p.option, 1);
    c.assignments = {{1, 77, 6.2, 6.1}};   // re-keyed since the bid: the option within r_same of the goal
    p = ApplyConsensus(&c, 1, 2, 10000000000, 12.0, o, 1.0);
    EXPECT_TRUE(p.coordinated);
    EXPECT_EQ(p.option, 2);
    const struct { const TaskConsensus* c; int self; uint32_t epoch; int64_t now; const char* why; } cases[] = {
        {nullptr, 1, 2, 10000000000, "uncoordinated: no consensus received"},
        {&c, 1, 3, 10000000000, "uncoordinated: consensus from another epoch"},
        {&c, 1, 2, 30000000000, "uncoordinated: consensus older than the timeout"},
        {&c, 0, 2, 10000000000, "uncoordinated: no assignment for this agent"},
    };
    for (const auto& k : cases) {
        p = ApplyConsensus(k.c, k.self, k.epoch, k.now, 12.0, o, 1.0);
        EXPECT_FALSE(p.coordinated);
        EXPECT_EQ(p.option, 0) << "its own best";
        EXPECT_EQ(p.why, k.why);
    }
    c.assignments = {{1, 77, 20, 20}};
    EXPECT_EQ(ApplyConsensus(&c, 1, 2, 10000000000, 12.0, o, 1.0).why, "uncoordinated: the assigned task is gone");
}

TEST(MultiAgentPlanning, PlanOwnWithoutReceivedDataEqualsPlansOwnAgent) {
    // The refactor guard: an agent's own planner with nothing received plans exactly what the centralised planner
    // plans for that agent when it is the only active one.
    std::vector<float> Z, S;
    const DepthFrame f = WallFrame(&Z, &S, 2.0);
    Phase4Params prm;
    Phase4Planner A(Brief(), prm, 2), B(Brief(), prm, 2);
    A.Integrate(0, 0, f), B.Integrate(0, 0, f);
    std::vector<Phase4AgentInput> ag(2);
    ag[0].active = true, ag[0].position = f.T_W_C.t;
    const Phase4Plan pc = A.Plan(5.0, ag, {});
    const Phase4Plan po = B.PlanOwn(5.0, 0, ag[0], {}, nullptr, {});
    ASSERT_EQ(po.agents.size(), 2u);
    EXPECT_EQ(po.allocation.method, "broker");
    const auto& a = pc.agents[0].nbv.options;
    const auto& b = po.agents[0].nbv.options;
    ASSERT_EQ(a.size(), b.size());
    ASSERT_FALSE(a.empty());
    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].task_id, b[i].task_id);
        EXPECT_EQ(a[i].utility, b[i].utility);
        EXPECT_EQ(a[i].goal_xy, b[i].goal_xy);
    }
    EXPECT_EQ(cv::countNonZero(pc.maps.trav_r != po.maps.trav_r), 0);
}

TEST(MultiAgentPlanning, PeerKnownColumnsAreNoGain) {
    std::vector<float> Z, S;
    const DepthFrame f = WallFrame(&Z, &S, 2.0);
    Phase4Params prm;
    Phase4Planner P(Brief(), prm, 2);
    P.Integrate(0, 0, f);
    Phase4AgentInput in;
    in.active = true, in.position = f.T_W_C.t;
    const Phase4Plan free_plan = P.PlanOwn(5.0, 0, in, {}, nullptr, {});
    const cv::Mat all(80, 80, CV_8U, cv::Scalar(1));   // a peer reported every column
    const Phase4Plan told = P.PlanOwn(5.0, 0, in, {}, &all, {});
    int frontier_before = 0, frontier_after = 0;
    for (const auto& o : free_plan.agents[0].nbv.options) frontier_before += o.source == TaskSource::kFrontier;
    for (const auto& o : told.agents[0].nbv.options) {
        frontier_after += o.source == TaskSource::kFrontier;
        EXPECT_EQ(o.ig_raw[0], 0) << "no coverage gain left anywhere";
    }
    EXPECT_GT(frontier_before, 0);
    EXPECT_EQ(frontier_after, 0) << "frontier views that only see peer-known space are no task";
    // RaycastIg directly: the same ray, with and without the mask.
    PeerKnown pk;
    pk.mask = &all, pk.grid = Brief().grid;
    const RigFrustum fr;
    Box3 box;   // the briefing's extent: the mask covers the grid, and rays leaving it are no gain anyway
    box.lo = core::Vec3(0, 0, 0), box.hi = core::Vec3(4, 4, 3);
    const core::Vec3 eye(1.0, 2.0, 1.0);   // looking back (-x) at the unobserved space behind the camera
    EXPECT_GT(RaycastIg(P.Map(), eye, M_PI, fr, nullptr, box).coverage, 0);
    EXPECT_EQ(RaycastIg(P.Map(), eye, M_PI, fr, nullptr, box, -1, &pk).coverage, 0);
}

namespace {
TaskCandidate FpOpt(int64_t id, double x, double y, double cov, std::vector<BlockGain> fp) {
    TaskCandidate c = Opt(id, x, y, cov, 1.0, cov / 100);
    c.footprint = std::move(fp);
    return c;
}
}  // namespace

TEST(MultiAgentPlanning, FootprintsSumToCoverageAndOnlyPerAgent) {
    std::vector<float> Z, S;
    const DepthFrame f = WallFrame(&Z, &S, 2.0);
    Phase4Params prm;
    Phase4Planner P(Brief(), prm, 2);
    P.Integrate(0, 0, f);
    Box3 box;
    box.lo = core::Vec3(0, 0, 0), box.hi = core::Vec3(4, 4, 3);
    const RigFrustum fr;
    const GridFrame g = Brief().grid;
    const IgCounts ig = RaycastIg(P.Map(), core::Vec3(1.0, 2.0, 1.0), M_PI, fr, nullptr, box, -1, nullptr, &g);
    int sum = 0;
    for (const auto& b : ig.footprint) sum += b.w;
    ASSERT_GT(ig.coverage, 0);
    EXPECT_LE(ig.footprint.size(), 32u);
    for (size_t i = 1; i < ig.footprint.size(); ++i) EXPECT_GE(ig.footprint[i - 1].w, ig.footprint[i].w) << "by w";
    // Every coverage voxel lies inside this 4 m grid: the units (rounded up per block) cover the count exactly.
    EXPECT_GE(sum * kFootprintUnit, ig.coverage);
    EXPECT_LT(sum * kFootprintUnit, ig.coverage + int(ig.footprint.size()) * kFootprintUnit);
    EXPECT_TRUE(RaycastIg(P.Map(), core::Vec3(1.0, 2.0, 1.0), M_PI, fr, nullptr, box).footprint.empty());
    std::vector<Phase4AgentInput> ag(2);
    ag[0].active = true, ag[0].position = f.T_W_C.t;
    const Phase4Plan pc = P.Plan(5.0, ag, {});
    for (const auto& o : pc.agents[0].nbv.options) EXPECT_TRUE(o.footprint.empty()) << "the oracle row pays nothing";
    const Phase4Plan po = P.PlanOwn(5.0, 0, ag[0], {}, nullptr, {});
    size_t with = 0;
    for (const auto& o : po.agents[0].nbv.options) with += !o.footprint.empty();
    EXPECT_GT(with, 0u);
}

TEST(MultiAgentPlanning, OverlapBrokerAvoidsTheSharedBlockDiscountAvoidsTheNearGoal) {
    // Agent 1's best view (B) sees the same block as agent 0's (A); its second (C) sees a fresh one. Goals are farther
    // apart than r_same, so the plain broker assigns A and B (the overlap); R1 assigns A and C; f counts each block once.
    NbvPlan n0, n1;
    n0.options = {FpOpt(1, 1.0, 1.0, 100, {{0, 0, 100}})};
    n1.options = {FpOpt(2, 2.5, 1.0, 100, {{0, 0, 100}}), FpOpt(3, 8.0, 8.0, 60, {{3, 3, 60}})};
    n0.lambda_path = n1.lambda_path = 0.0;
    const std::vector<TaskBid> bids = {BuildBid(0, 1, 1, 0, n0, 8), BuildBid(1, 1, 1, 0, n1, 8)};
    const ComponentInfo comp{0, {0, 1}, 1};
    auto KeyOf = [](const TaskConsensus& c, int agent) {
        for (const auto& a : c.assignments) if (a.agent == agent) return a.task_key;
        return int64_t(-1);
    };
    BrokerParams prm;
    prm.r_same = 1.0, prm.range_m = 4.0;
    EXPECT_EQ(KeyOf(BrokerAssign(bids, comp, prm, 0, 1), 1), 2) << "plain: B, the shared block";
    prm.mode = BrokerMode::kOverlap;
    BrokerLog log;
    const TaskConsensus r1 = BrokerAssign(bids, comp, prm, 0, 1, &log);
    EXPECT_EQ(KeyOf(r1, 0), 1);
    EXPECT_EQ(KeyOf(r1, 1), 3) << "R1: C, the fresh block";
    ASSERT_EQ(log.rounds.size(), 2u);
    EXPECT_DOUBLE_EQ(log.f, 160.0 * kFootprintUnit);
    EXPECT_GE(log.upper_bound, log.f);
    prm.mode = BrokerMode::kDiscount;   // B is 1.5 m from A: x 1.5/4; C is far: x 1
    EXPECT_EQ(KeyOf(BrokerAssign(bids, comp, prm, 0, 1), 1), 3) << "R2: the near goal is discounted";
    prm.range_m = 1.2;                  // with a short range B is not discounted at all
    EXPECT_EQ(KeyOf(BrokerAssign(bids, comp, prm, 0, 1), 1), 2);
}

TEST(MultiAgentPlanning, CutOffAgentDiscountsByRememberedClaims) {
    // Agent 1 is cut off (the consensus it holds is older than the timeout). Its best option is next to agent 0's last
    // assigned goal: discounted, it takes the far one; once the claim is old (age >> tau), the near one again.
    const std::vector<TaskCandidate> o = {Opt(5, 1.2, 1.0, 9, 1, 0.9), Opt(6, 8, 8, 5, 1, 0.5)};
    TaskConsensus c;
    c.epoch = 1, c.t_ns = 0, c.assignments = {{0, 99, 1.0, 1.0}};
    Pick p = ApplyConsensusDiscounted(&c, 1, 1, 20000000000, 12.0, o, 1.0, 4.0, 30.0);
    EXPECT_FALSE(p.coordinated);
    EXPECT_EQ(p.option, 1);
    EXPECT_NE(p.why.find("tau 30 s"), std::string::npos);
    p = ApplyConsensusDiscounted(&c, 1, 1, 400000000000, 12.0, o, 1.0, 4.0, 30.0);
    EXPECT_EQ(p.option, 0) << "a 400 s old claim has decayed";
    c.t_ns = 15000000000, c.assignments = {{1, 6, 8, 8}};   // a fresh consensus for agent 1 itself: coordinated as usual
    EXPECT_TRUE(ApplyConsensusDiscounted(&c, 1, 1, 20000000000, 12.0, o, 1.0, 4.0, 30.0).coordinated);
}

namespace {
TraversabilityMaps Corridor(const GridFrame& g, int c0, int c1) {   // rows 16..47 free and certified by the sender
    TraversabilityMaps m = Maps(g);
    for (int r = 16; r < 48; ++r)
        for (int c = c0; c < c1; ++c) m.slab.at<uint8_t>(r, c) = kFree, m.trav_r.at<uint8_t>(r, c) = 1;
    return m;
}
}  // namespace

TEST(MultiAgentPlanning, OperatorEgressRoutesOnlyOverWhatWasReceivedAndCertified) {
    // A 1.6 m corridor, 4.8 m long: agent 1 mapped its west part, agent 2 its east part; neither alone joins target and
    // exit. Agent 3 then reports a pillar inside the corridor: cells within rho_responder of it leave the lattice even
    // though agents 1 and 2 had certified them, and the incremental replan reroutes around it.
    const GridFrame g = Grid(64, 96);
    SlabExporter e1(1, g), e2(2, g), e3(3, g);
    e1.Update(Corridor(g, 0, 56)), e2.Update(Corridor(g, 40, 96));
    TraversabilityMaps pillar = Maps(g);
    pillar.slab.at<uint8_t>(26, 60) = kOccupied;
    e3.Update(pillar);
    ReceivedLayers R(0, 4, g);
    OperatorEgress op(g, Cell{32, 4}, {Cell{32, 92}}, 0.40);
    R.Integrate(e1.Export(0, 0, 0));
    EXPECT_FALSE(op.Replan(R.Merged()).found) << "the east part was never received";
    R.Integrate(e2.Export(0, 0, 0));
    const EgressReplan a = op.Replan(R.Merged());
    ASSERT_TRUE(a.found);
    EXPECT_EQ(a.confidence, -1) << "no support on the wire: not computed";
    EXPECT_NEAR(a.length_m, 88 * 0.05, 0.3);
    for (const Cell& q : a.cells) {
        EXPECT_TRUE(op.Lattice().at<uint8_t>(q.r, q.c));
        EXPECT_GE(op.Clearance().at<float>(q.r, q.c), 0.40f - 1e-6f) << "rho_responder (invariant 4)";
    }
    EXPECT_EQ(op.Lattice().at<uint8_t>(18, 30), 0) << "certified by agent 1 but within rho of the corridor's edge";
    R.Integrate(e3.Export(0, 0, 0));
    const cv::Mat M = R.Merged();
    EXPECT_EQ(M.at<uint8_t>(30, 60), kCellFreeResponder) << "both observers certified it";
    const EgressReplan b = op.Replan(M);
    ASSERT_TRUE(b.found);
    EXPECT_EQ(op.Lattice().at<uint8_t>(30, 60), 0) << "within rho of agent 3's pillar";
    EXPECT_NE(std::find(b.triggers.begin(), b.triggers.end(), "new_obstacle"), b.triggers.end());
    for (const Cell& q : b.cells)
        if (q.c == 60) EXPECT_GE(q.r, 35) << "around the pillar, >= rho from its edge";
}

namespace {
// One fleet of N agents over K barriers, each agent with its OWN planner, exporter and received layers. Per barrier:
// the broker (agent 0) assigns from the bids that arrived (the previous barrier's), inline; then every agent's step
// (integrate its own frame and what it received, PlanOwn, export, bid, apply the consensus) runs serially or on one
// thread per agent, each thread touching only its own agent; then sends go out serially in id order, full delivery.
// Returns the decision log: every option's id, utility and goal (hex floats), patch, bid and pick, and the broker's
// assignments.
std::string RunFleet(bool threaded) {
    constexpr int N = 3, K = 4;
    const Phase4Briefing b = Brief();
    const Phase4Params prm;
    struct Agent {
        Phase4Planner planner;
        SlabExporter ex;
        ReceivedLayers rx;
        std::vector<float> Z, S;
        std::vector<SubmapPatch> in_patches;
        std::vector<PeerPose> in_poses;
        uint64_t sent = 0;   // the version last sent: every receiver acks it under full delivery
        SubmapPatch out_patch;
        TaskBid out_bid;
        PeerPose out_pose;
        std::string log;
        Agent(const Phase4Briefing& b, const Phase4Params& p, int i) : planner(b, p, N), ex(i, b.grid), rx(i, N, b.grid) {}
    };
    std::vector<std::unique_ptr<Agent>> A;
    for (int i = 0; i < N; ++i) A.push_back(std::make_unique<Agent>(b, prm, i));
    const ComponentInfo comp{0, {0, 1, 2}, 1};
    BrokerParams bp;
    bp.mode = BrokerMode::kOverlap, bp.r_same = 1.0, bp.range_m = 4.0;
    std::vector<TaskBid> bids;
    std::optional<TaskConsensus> cons;
    std::ostringstream log;
    log << std::hexfloat;
    for (int k = 0; k < K; ++k) {
        const int64_t t_ns = int64_t(5 + 5 * k) * 1000000000;
        if (!bids.empty()) {
            cons = BrokerAssign(bids, comp, bp, t_ns, uint64_t(k));
            log << "k" << k << " broker";
            for (const Assignment& x : cons->assignments) log << " " << x.agent << ":" << x.task_key << "@" << x.goal_x << "," << x.goal_y;
            log << "\n";
        }
        auto Step = [&](int i) {
            Agent& a = *A[size_t(i)];
            DepthFrame f = WallFrame(&a.Z, &a.S, 0.8 + 1.2 * i);
            std::fill(a.Z.begin(), a.Z.end(), float(2.0 - 0.3 * k));   // flying east; the wall stays at x = 2.5
            f.T_W_C.t.x() = 0.5 + 0.3 * k;
            a.planner.Integrate(i, t_ns, f);
            for (const SubmapPatch& p : a.in_patches) a.rx.Integrate(p);
            std::vector<PeerKeyframe> peers;
            for (const PeerPose& q : a.in_poses) peers.push_back({q.agent, q.kf_id, core::Vec3(q.x, q.y, q.z)});
            Phase4AgentInput in;
            in.active = true, in.position = f.T_W_C.t;
            const Phase4Plan p = a.planner.PlanOwn(double(t_ns) * 1e-9, i, in, peers, &a.rx.KnownMask(), {});
            a.ex.Update(p.maps);
            a.out_patch = a.ex.Export(a.sent, 1, t_ns);
            a.sent = a.ex.Version();
            const auto& opts = p.agents[size_t(i)].nbv.options;
            a.out_bid = BuildBid(i, 1, uint64_t(k), t_ns, p.agents[size_t(i)].nbv, 8);
            a.out_pose = PeerPose{i, k, f.T_W_C.t.x(), f.T_W_C.t.y(), f.T_W_C.t.z(), t_ns};
            const Pick pick = ApplyConsensus(cons ? &*cons : nullptr, i, 1, t_ns, 12.0, opts, bp.r_same);
            std::ostringstream s;
            s << std::hexfloat << "k" << k << " a" << i << " blocks=" << a.out_patch.blocks.size() << " v=" << a.sent
              << " bid=" << a.out_bid.options.size() << " pick=" << pick.option << " why=" << pick.why << " opts";
            for (const TaskCandidate& o : opts)
                s << " " << o.task_id << ":" << o.utility << "@" << o.goal_xy.first << "," << o.goal_xy.second << "/" << o.footprint.size();
            a.log = s.str() + "\n";
        };
        if (threaded) {
            std::vector<std::thread> T;
            for (int i = 0; i < N; ++i) T.emplace_back(Step, i);
            for (auto& t : T) t.join();
        } else {
            for (int i = 0; i < N; ++i) Step(i);
        }
        bids.clear();
        for (auto& a : A) a->in_patches.clear(), a->in_poses.clear();
        for (int i = 0; i < N; ++i) {
            log << A[size_t(i)]->log;
            bids.push_back(A[size_t(i)]->out_bid);
            for (int j = 0; j < N; ++j)
                if (j != i) A[size_t(j)]->in_patches.push_back(A[size_t(i)]->out_patch), A[size_t(j)]->in_poses.push_back(A[size_t(i)]->out_pose);
        }
    }
    return log.str();
}

size_t Count(const std::string& s, const std::string& what) {
    size_t n = 0;
    for (size_t at = s.find(what); at != std::string::npos; at = s.find(what, at + 1)) ++n;
    return n;
}
}  // namespace

TEST(MultiAgentPlanning, NAgentStepsOnNThreadsEqualTheSerialRun) {
    // The share-nothing claim at the library level: one thread per agent, no lock, and the decision log equals the
    // serial run's byte for byte (twice, so a lucky interleaving does not pass alone).
    const std::string serial = RunFleet(false);
    EXPECT_EQ(RunFleet(true), serial);
    EXPECT_EQ(RunFleet(true), serial);
    EXPECT_EQ(Count(serial, "why=uncoordinated: no consensus received"), 3u) << "barrier 0 only";
    EXPECT_GE(Count(serial, "why=coordinated"), 3u) << "the broker's assignments reach the members";
    EXPECT_EQ(Count(serial, " broker"), 3u);
    EXPECT_EQ(Count(serial, "blocks=0 "), 0u) << "every agent's map changed at every barrier";
}

TEST(Multiagent, AllocationIdentitySurvivesGeometricSolveRevisions) {
    ComponentInfo c; c.broker = 0; c.members = {0, 1};
    c.epoch = CoordinationToken(c.broker, c.members);
    TaskConsensus a; a.broker = 0; a.members = {0, 1}; a.epoch = c.epoch;
    for (uint64_t graph_epoch : {1u, 2u, 17u, 900u}) {
        (void)graph_epoch;  // geometric revisions are deliberately not allocation identity
        EXPECT_EQ(CoordinationToken(0, {1, 0}), c.epoch);
        EXPECT_TRUE(MatchesComponent(a, c));
    }
    c.broker = 1; EXPECT_FALSE(MatchesComponent(a, c));
    c.broker = 0; c.members = {0}; EXPECT_FALSE(MatchesComponent(a, c));
    // Even a token collision cannot authorize a consensus for other members.
    c.members = {0, 2}; EXPECT_FALSE(MatchesComponent(a, c));
    EXPECT_NE(CoordinationToken(0, {0, 1}), CoordinationToken(0, {0, 2}));
}
