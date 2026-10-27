// Phase4Planner (the planning side of B's MissionPlanner seam): string pulling never cuts a blocked cell and keeps few
// waypoints; directives (keep_out drops, assign restricts, priority_region scales G and reorders) and refusals; the
// keep_out masks paths, not only goals (B's review); the joint pick is exactly-once and the higher-utility agent
// picks first (also at N = 3: in descending best utility); an Integrate -> Plan -> Route smoke on a synthetic wall,
// deterministic across two planners; a 3-agent Plan (sizes, exactly-once picks, the CBBA graph's topology trigger).
#include "planning/mission/phase4_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

using namespace ariadne;
using namespace ariadne::planning;

namespace {

TaskCandidate Opt(int64_t id, double x, double y, double gain, double util) {
    TaskCandidate c;
    c.task_id = id, c.goal_xy = {x, y}, c.gain = gain, c.utility = util;
    return c;
}

bool SegmentClear(const cv::Mat& plan, const Cell& a, const Cell& b) {   // dense sampling of the continuous chord
    const int n = 400;
    for (int k = 0; k <= n; ++k) {
        const double t = double(k) / n;
        const int r = int(std::floor(a.r + 0.5 + t * (b.r - a.r))), c = int(std::floor(a.c + 0.5 + t * (b.c - a.c)));
        if (!plan.at<uint8_t>(r, c)) return false;
    }
    return true;
}

}  // namespace

TEST(Phase4Planner, StringPullLCorridor) {
    cv::Mat plan = cv::Mat::zeros(40, 40, CV_8U);
    plan(cv::Rect(2, 2, 4, 30)).setTo(1);    // vertical leg, columns 2..5
    plan(cv::Rect(2, 28, 30, 4)).setTo(1);   // horizontal leg, rows 28..31
    const GridPath p = AStar(plan, {3, 3}, {{29, 30}});
    ASSERT_TRUE(p.found);
    const auto k = StringPull(plan, p.cells);
    EXPECT_LE(k.size(), 4u) << "an L needs one corner";
    EXPECT_EQ(k.front(), p.cells.front());
    EXPECT_EQ(k.back(), p.cells.back());
    for (size_t i = 1; i < k.size(); ++i) EXPECT_TRUE(SegmentClear(plan, k[i - 1], k[i])) << i;
}

TEST(Phase4Planner, DirectivesAndRefusals) {
    Phase4AgentPlan ap;
    ap.nbv.options = {Opt(1, 1, 1, 1.0, 0.9), Opt(2, 5, 5, 1.0, 0.8), Opt(3, 9, 9, 1.0, 0.7), Opt(4, 2, 8, 1.0, 0.6)};
    RegionDirective keep{"k1", "keep_out", -1, {{0, 0}, {2, 0}, {2, 2}, {0, 2}}, 1.0};
    RegionDirective prio{"p1", "priority_region", 0, {{8, 8}, {10, 8}, {10, 10}, {8, 10}}, 3.0};
    RegionDirective other{"x1", "keep_out", 1, {{4, 4}, {6, 4}, {6, 6}, {4, 6}}, 1.0};   // agent 1 only
    ApplyDirectives(0, {keep, prio, other}, {{2.1, 8.1}}, 0.5, 0.0, &ap);
    ASSERT_EQ(ap.nbv.options.size(), 2u);
    EXPECT_EQ(ap.nbv.options[0].task_id, 3) << "priority x3: utility 0.7 + 2 * 1.0";
    EXPECT_DOUBLE_EQ(ap.nbv.options[0].utility, 2.7);
    EXPECT_EQ(ap.nbv.options[1].task_id, 2);
    EXPECT_EQ(ap.excluded_directive, 1);
    EXPECT_EQ(ap.excluded_refused, 1);
    EXPECT_EQ(ap.applied, (std::vector<std::string>{"k1", "p1"}));
    Phase4AgentPlan as;
    as.nbv.options = {Opt(1, 1, 1, 1, 0.9), Opt(2, 5, 5, 1, 0.8)};
    ApplyDirectives(1, {RegionDirective{"a1", "assign", 1, {{4, 4}, {6, 4}, {6, 6}, {4, 6}}, 1.0}}, {}, 0.5, 0.0, &as);
    ASSERT_EQ(as.nbv.options.size(), 1u);
    EXPECT_EQ(as.nbv.options[0].task_id, 2);
    Phase4AgentPlan none;   // an assign region holding no option restricts nothing (B's semantics)
    none.nbv.options = {Opt(1, 1, 1, 1, 0.9)};
    ApplyDirectives(1, {RegionDirective{"a2", "assign", 1, {{4, 4}, {6, 4}, {6, 6}, {4, 6}}, 1.0}}, {}, 0.5, 0.0, &none);
    EXPECT_EQ(none.nbv.options.size(), 1u);
    Phase4AgentPlan pull;   // a priority region pulls a view whose observed centroid is within observe_m of it
    TaskCandidate near = Opt(1, 0, 0, 1.0, 0.5), far = Opt(2, 0, 0, 1.0, 0.6);
    near.centroid_xy = {7.0, 9.0}, far.centroid_xy = {0.0, 0.0};
    pull.nbv.options = {far, near};
    ApplyDirectives(0, {prio}, {}, 0.5, 2.84, &pull);
    EXPECT_EQ(pull.nbv.options[0].task_id, 1) << "1 m from the region, inside the 2.84 m depth gate: x3";
    EXPECT_DOUBLE_EQ(RegionDirective(prio).Distance(7.0, 9.0), 1.0);
    EXPECT_DOUBLE_EQ(RegionDirective(prio).Distance(9.0, 9.0), 0.0);
    Phase4AgentPlan hr;   // the hard rule's set is outside every directive
    hr.nbv.reserve_rule = true;
    hr.nbv.options = {Opt(9, 1, 1, 0, 1)};
    ApplyDirectives(0, {keep}, {}, 0.5, 0.0, &hr);
    EXPECT_EQ(hr.nbv.options.size(), 1u);
}

TEST(Phase4Planner, KeepOutMasksPaths) {
    // A 2 m-wide corridor with a keep_out band across its full width: no path crosses it. The same directive aimed
    // at agent 1 leaves agent 0's space open.
    GridFrame g;
    g.ox = 0, g.oy = 0, g.cell = 0.05, g.cols = 40, g.rows = 100;
    cv::Mat plan(100, 40, CV_8U, cv::Scalar(1));
    MaskKeepOut(0, {RegionDirective{"k", "keep_out", -1, {{-1, 2.0}, {3, 2.0}, {3, 2.5}, {-1, 2.5}}, 1.0}}, g, &plan);
    EXPECT_FALSE(AStar(plan, {10, 20}, {{90, 20}}).found);
    cv::Mat plan1(100, 40, CV_8U, cv::Scalar(1));   // the same directive for agent 1 only leaves agent 0 unmasked
    MaskKeepOut(0, {RegionDirective{"k", "keep_out", 1, {{-1, 2.0}, {3, 2.0}, {3, 2.5}, {-1, 2.5}}, 1.0}}, g, &plan1);
    EXPECT_TRUE(AStar(plan1, {10, 20}, {{90, 20}}).found);
}

TEST(Phase4Planner, JointPickExactlyOnce) {
    Phase4Plan p;
    p.agents.resize(2);
    p.agents[0].nbv.options = {Opt(7, 0, 0, 1, 0.5), Opt(8, 0, 0, 1, 0.4)};
    p.agents[1].nbv.options = {Opt(7, 0, 0, 1, 0.9), Opt(9, 0, 0, 1, 0.1)};
    JointPick(&p);
    EXPECT_EQ(p.joint[1], 0) << "agent 1's best utility is higher: it picks task 7";
    EXPECT_EQ(p.joint[0], 1) << "agent 0 takes its best other task";
    Phase4Plan q;
    q.agents.resize(2);
    q.agents[0].nbv.options = {Opt(5, 0, 0, 1, 1.0)};
    JointPick(&q);
    EXPECT_EQ(q.joint[0], 0);
    EXPECT_EQ(q.joint[1], -1);
}

TEST(Phase4Planner, IntegratePlanRouteSmoke) {
    // A 4 x 4 m briefing; one camera at (0.5, 2.0, 1.0) facing +x sees a wall at x = 2.5 m (depth = 2 m on the axis).
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 80, b.grid.rows = 80;
    b.target = {40, 70};
    b.entrances = {{40, 10}};
    Phase4Params prm;
    const int W = 160, H = 90;
    const double fx = 84, fy = 84, cx = 80, cy = 45;
    std::vector<float> Z(size_t(W * H)), S(size_t(W * H), 0.02f);
    for (int v = 0; v < H; ++v)
        for (int u = 0; u < W; ++u) Z[size_t(v * W + u)] = 2.0f;   // a fronto-parallel plane: z_cam = 2 m everywhere
    DepthFrame f;
    f.T_W_C.R << 0, 0, 1, -1, 0, 0, 0, -1, 0;   // optical z -> world +x, optical x -> world -y, optical y -> world -z
    f.T_W_C.t = core::Vec3(0.5, 2.0, 1.0);
    f.fx = fx, f.fy = fy, f.cx = cx, f.cy = cy, f.width = W, f.height = H, f.depth = Z.data(), f.sigma = S.data();
    Phase4Planner A(b, prm), B(b, prm);
    EXPECT_GT(A.Integrate(0, 0, f), 0u);
    B.Integrate(0, 0, f);
    std::vector<Phase4AgentInput> ag(2);
    ag[0].active = true, ag[0].position = f.T_W_C.t;
    const Phase4Plan pa = A.Plan(5.0, ag, {}), pb = B.Plan(5.0, ag, {});
    ASSERT_EQ(pa.agents[0].nbv.options.size(), pb.agents[0].nbv.options.size());
    for (size_t i = 0; i < pa.agents[0].nbv.options.size(); ++i) {
        EXPECT_EQ(pa.agents[0].nbv.options[i].task_id, pb.agents[0].nbv.options[i].task_id);
        EXPECT_EQ(pa.agents[0].nbv.options[i].utility, pb.agents[0].nbv.options[i].utility);
    }
    EXPECT_FALSE(pa.egress.found) << "the target is behind the wall, unobserved";
    if (!pa.agents[0].nbv.options.empty()) {
        // The joint pick is CBBA's (auction value, D115), not necessarily the utility-top option: any valid option.
        ASSERT_GE(pa.joint[0], 0);
        ASSERT_LT(size_t(pa.joint[0]), pa.agents[0].nbv.options.size());
        const Cell from{40, 10};
        const auto [wp, yaw] = A.Route(pa, 0, pa.joint[0], from);
        EXPECT_DOUBLE_EQ(yaw, pa.agents[0].nbv.options[size_t(pa.joint[0])].goal_yaw);
        double len = 0, px = (from.c + 0.5) * 0.05, py = (from.r + 0.5) * 0.05;
        for (const auto& [x, y] : wp) len += std::hypot(x - px, y - py), px = x, py = y;
        EXPECT_LE(len, prm.max_command_m + 1e-9);
    }
}

TEST(Phase4Planner, AuctionUsesCommonScales) {
    // D115 test 8: agent 0 also holds many weak candidates (which shrink its own p90); the shared task 42, seen with
    // identical raw terms by both agents, must get the SAME value for both: the scales are common, not per agent.
    Phase4Plan p;
    p.agents.resize(2);
    auto Raw = [](int64_t id, double cov, double ent, double path) {
        TaskCandidate c;
        c.task_id = id, c.ig_raw = {cov, ent, 0, 0, 0}, c.path_m = path, c.utility = cov;
        return c;
    };
    p.agents[0].nbv.options = {Raw(42, 100, 50, 2)};
    for (int i = 0; i < 30; ++i) p.agents[0].nbv.options.push_back(Raw(1000 + i, 1, 1, 2));
    p.agents[1].nbv.options = {Raw(42, 100, 50, 2), Raw(7, 300, 80, 3)};
    GridFrame g;
    g.cell = 0.05;
    std::vector<float> scales;
    std::vector<std::vector<int>> view;
    const AuctionInstance in = BuildAuction(p, g, 1, &scales, &view);
    const size_t j = size_t(std::lower_bound(in.task_ids.begin(), in.task_ids.end(), 42) - in.task_ids.begin());
    EXPECT_DOUBLE_EQ(in.value[0][j], in.value[1][j]);
    EXPECT_EQ(scales.size(), 8u);
    // Common scales: coverage max(p90 agent 0 = 1, p90 agent 1 over {100, 300} = 100) = 100, entropy max(1, 50) = 50,
    // so value = 100/100 + 50/50 = 2 for both. Per-agent scales would give agent 0 100/1 + 50/1 = 150.
    EXPECT_DOUBLE_EQ(in.value[0][j], 2.0);
    EXPECT_FLOAT_EQ(scales[0], 100.f);
    EXPECT_FLOAT_EQ(scales[1], 50.f);
}

TEST(Phase4Planner, AllocationIsEventTriggered) {
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 80, b.grid.rows = 80;
    b.target = {40, 70};
    b.entrances = {{40, 10}};
    Phase4Params prm;
    const int W = 160, H = 90;
    std::vector<float> Z(size_t(W * H), 2.0f), S(size_t(W * H), 0.02f);
    DepthFrame f;
    f.T_W_C.R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    f.T_W_C.t = core::Vec3(0.5, 2.0, 1.0);
    f.fx = 84, f.fy = 84, f.cx = 80, f.cy = 45, f.width = W, f.height = H, f.depth = Z.data(), f.sigma = S.data();
    Phase4Planner P(b, prm);
    P.Integrate(0, 0, f);
    std::vector<Phase4AgentInput> ag(2);
    ag[0].active = true, ag[0].position = f.T_W_C.t;
    const Phase4Plan p1 = P.Plan(5.0, ag, {});
    EXPECT_EQ(p1.allocation.method, "cbba");
    EXPECT_EQ(p1.allocation.trigger, "first");
    const Phase4Plan p2 = P.Plan(10.0, ag, {});   // nothing new was observed: same tasks, same topology
    EXPECT_EQ(p2.allocation.trigger, "none");
    EXPECT_EQ(p2.allocation.messages, 0u);
    if (p1.joint[0] >= 0) {
        ASSERT_GE(p2.joint[0], 0);
        EXPECT_EQ(p2.agents[0].nbv.options[size_t(p2.joint[0])].task_id, p1.agents[0].nbv.options[size_t(p1.joint[0])].task_id);
    }
}

TEST(Phase4Planner, JointPickExactlyOnceThreeAgents) {
    // N = 3 (Phase 8 §42): descending best utility picks first; each takes its best task no earlier agent took.
    Phase4Plan p;
    p.agents.resize(3);
    p.agents[0].nbv.options = {Opt(7, 0, 0, 1, 0.5), Opt(8, 0, 0, 1, 0.4), Opt(9, 0, 0, 1, 0.3)};
    p.agents[1].nbv.options = {Opt(7, 0, 0, 1, 0.9), Opt(9, 0, 0, 1, 0.1)};
    p.agents[2].nbv.options = {Opt(7, 0, 0, 1, 0.7), Opt(8, 0, 0, 1, 0.6), Opt(10, 0, 0, 1, 0.2)};
    JointPick(&p);
    ASSERT_EQ(p.joint.size(), 3u);
    EXPECT_EQ(p.joint[1], 0) << "agent 1 (0.9) picks first: task 7";
    EXPECT_EQ(p.joint[2], 1) << "agent 2 (0.7) next: task 8";
    EXPECT_EQ(p.joint[0], 2) << "agent 0 (0.5) last: tasks 7 and 8 are taken, so task 9";
    Phase4Plan q;   // ties go to the lower index; an agent left with only taken tasks gets none
    q.agents.resize(3);
    q.agents[0].nbv.options = {Opt(5, 0, 0, 1, 1.0)};
    q.agents[1].nbv.options = {Opt(5, 0, 0, 1, 1.0)};
    q.agents[2].nbv.options = {Opt(6, 0, 0, 1, 1.0)};
    JointPick(&q);
    EXPECT_EQ(q.joint[0], 0);
    EXPECT_EQ(q.joint[1], -1);
    EXPECT_EQ(q.joint[2], 0);
}

TEST(Phase4Planner, ThreeAgentPlanAndTopology) {
    // Three agents on the synthetic wall: Plan sizes everything N = 3, the picks are exactly-once by task_id, the
    // CBBA graph is every active linked pair (a link change re-triggers as "topology"), and the API refuses sizes
    // that do not match the agent count.
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 80, b.grid.rows = 80;
    b.target = {40, 70};
    b.entrances = {{40, 10}};
    Phase4Params prm;
    const int W = 160, H = 90;
    std::vector<float> Z(size_t(W * H), 2.0f), S(size_t(W * H), 0.02f);
    DepthFrame f;
    f.T_W_C.R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    f.T_W_C.t = core::Vec3(0.5, 2.0, 1.0);
    f.fx = 84, f.fy = 84, f.cx = 80, f.cy = 45, f.width = W, f.height = H, f.depth = Z.data(), f.sigma = S.data();
    Phase4Planner P(b, prm, 3);
    EXPECT_EQ(P.Agents(), 3);
    for (int a = 0; a < 3; ++a) {
        DepthFrame g = f;
        g.T_W_C.t = core::Vec3(0.5, 1.0 + 0.5 * a, 1.0);
        P.Integrate(a, int64_t(a), g);
    }
    EXPECT_THROW(P.Integrate(3, 0, f), std::out_of_range);
    std::vector<Phase4AgentInput> ag(3);
    for (int a = 0; a < 3; ++a) ag[size_t(a)].active = true, ag[size_t(a)].position = core::Vec3(0.5, 1.0 + 0.5 * a, 1.0);
    EXPECT_THROW(P.Plan(5.0, std::vector<Phase4AgentInput>(2), {}), std::invalid_argument);
    const Phase4Plan p1 = P.Plan(5.0, ag, {});
    ASSERT_EQ(p1.agents.size(), 3u);
    ASSERT_EQ(p1.joint.size(), 3u);
    EXPECT_EQ(p1.allocation.trigger, "first");
    EXPECT_EQ(p1.allocation.optimum, -1) << "the exact optimum is 2-agent only";
    std::set<int64_t> picked;
    for (int a = 0; a < 3; ++a) {
        const int j = p1.joint[size_t(a)];
        if (j < 0) continue;
        ASSERT_LT(size_t(j), p1.agents[size_t(a)].nbv.options.size());
        EXPECT_TRUE(picked.insert(p1.agents[size_t(a)].nbv.options[size_t(j)].task_id).second) << "exactly-once";
    }
    EXPECT_EQ(P.Plan(10.0, ag, {}).allocation.trigger, "none");
    ag[2].linked = false;   // agent 2 loses its link: the graph changes, the task set does not
    EXPECT_EQ(P.Plan(15.0, ag, {}).allocation.trigger, "topology");
}

TEST(Phase4Planner, JointPickNeverCoordinatesAcrossAPartition) {
    // Two components cannot deconflict (no link, no broker): each picks exactly-once within its own component only.
    Phase4Plan p;
    p.agents.resize(3);
    p.agents[0].nbv.options = {Opt(7, 0, 0, 1, 0.9), Opt(8, 0, 0, 1, 0.1)};
    p.agents[1].nbv.options = {Opt(7, 0, 0, 1, 0.8), Opt(9, 0, 0, 1, 0.2)};
    p.agents[2].nbv.options = {Opt(7, 0, 0, 1, 0.7), Opt(10, 0, 0, 1, 0.3)};
    JointPick(&p, {0, 0, 2});   // agents 0 and 1 linked; agent 2 alone
    EXPECT_EQ(p.joint[0], 0) << "task 7";
    EXPECT_EQ(p.joint[1], 1) << "task 9: 7 is taken in its component";
    EXPECT_EQ(p.joint[2], 0) << "task 7 too: agent 2 cannot know agent 0 took it";
    JointPick(&p);   // one component: exactly-once over the whole fleet
    EXPECT_EQ(p.joint[2], 1);
}

TEST(Phase4Planner, PartitionFreezesPeersAndIsLabelled) {
    // Peers are what an agent RECEIVED: while agent 2 is cut off, agents 0 and 1 keep its track as of the last shared
    // replan (and it theirs); on reconnecting they catch up. The plan says what is and is not modelled.
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 80, b.grid.rows = 80;
    b.target = {40, 70};
    b.entrances = {{40, 10}};
    Phase4Params prm;
    const int W = 160, H = 90;
    std::vector<float> Z(size_t(W * H), 2.0f), S(size_t(W * H), 0.02f);
    DepthFrame f;
    f.T_W_C.R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    f.fx = 84, f.fy = 84, f.cx = 80, f.cy = 45, f.width = W, f.height = H, f.depth = Z.data(), f.sigma = S.data();
    Phase4Planner P(b, prm, 3);
    auto Kf = [&](int a, double y) { DepthFrame g = f; g.T_W_C.t = core::Vec3(0.5, y, 1.0); P.Integrate(a, 0, g); };
    for (int a = 0; a < 3; ++a) Kf(a, 1.0 + 0.5 * a);
    std::vector<Phase4AgentInput> ag(3);
    for (int a = 0; a < 3; ++a) ag[size_t(a)].active = true, ag[size_t(a)].position = core::Vec3(0.5, 1.0 + 0.5 * a, 1.0);
    const Phase4Plan p1 = P.Plan(5.0, ag, {});
    EXPECT_EQ(p1.allocation.components, 1);
    EXPECT_TRUE(p1.allocation.partition_note.empty());
    EXPECT_EQ(P.SharedTrack(0, 2), 1u);
    ag[2].linked = false;
    Kf(2, 2.2), Kf(1, 1.6);   // new keyframes on both sides of the cut
    const Phase4Plan p2 = P.Plan(10.0, ag, {});
    EXPECT_EQ(p2.allocation.components, 2);
    EXPECT_EQ(p2.allocation.partition_note, "allocation partition-aware; map and peers centralised [PROXY]");
    EXPECT_EQ(P.SharedTrack(0, 2), 1u) << "agent 2's new keyframe never reached agent 0";
    EXPECT_EQ(P.SharedTrack(2, 1), 1u) << "nor agent 1's reached agent 2";
    EXPECT_EQ(P.SharedTrack(0, 1), 2u) << "agents 0 and 1 are still linked";
    ag[2].linked = true;
    P.Plan(15.0, ag, {});
    EXPECT_EQ(P.SharedTrack(0, 2), 2u);
    EXPECT_EQ(P.SharedTrack(2, 1), 2u);
}

TEST(Phase4Planner, CachedTraversabilityEqualsAFullRecomputeBitForBit) {
    // The live planner recomputes only the columns near new map changes between replans; after several views and
    // replans every traversability layer must equal ComputeTraversability on the final map, bit for bit.
    Phase4Briefing b;
    b.grid.ox = 0, b.grid.oy = 0, b.grid.cell = 0.05, b.grid.cols = 120, b.grid.rows = 120;
    b.target = {60, 110};
    b.entrances = {{60, 10}};
    Phase4Params prm;
    const int W = 160, H = 90;
    std::vector<Phase4AgentInput> ag(2);
    ag[0].active = true;
    Phase4Planner P(b, prm);
    // Fronto-parallel planes at different depths seen from different spots and headings: each view dirties a
    // different part of the grid, and replans fall between views.
    const struct { double x, y, yaw, depth; } views[] = {
        {1.0, 3.0, 0.0, 2.0}, {3.0, 1.0, M_PI / 2, 1.5}, {5.0, 3.0, M_PI, 2.5}, {3.0, 5.0, -M_PI / 2, 1.2}, {2.0, 2.0, 0.7, 3.0}};
    int i = 0;
    for (const auto& v : views) {
        std::vector<float> Z(size_t(W * H), float(v.depth)), S(size_t(W * H), 0.02f);
        DepthFrame f;
        const double c = std::cos(v.yaw), s = std::sin(v.yaw);
        // optical z -> world (c, s, 0), optical x -> world (s, -c, 0), optical y -> world -z
        f.T_W_C.R << s, 0, c, -c, 0, s, 0, -1, 0;
        f.T_W_C.t = core::Vec3(v.x, v.y, 1.0);
        f.fx = 84, f.fy = 84, f.cx = 80, f.cy = 45, f.width = W, f.height = H, f.depth = Z.data(), f.sigma = S.data();
        P.Integrate(0, int64_t(i), f);
        ag[0].position = f.T_W_C.t;
        if (i % 2 == 0) P.Plan(5.0 * (i + 1), ag, {});   // replans between views (cache in use)
        ++i;
    }
    const Phase4Plan last = P.Plan(100.0, ag, {});
    const TraversabilityMaps full = ComputeTraversability(P.Map(), P.Distance(), b.grid, prm.trav);
    auto Same = [](const cv::Mat& a, const cv::Mat& b) {
        return a.size() == b.size() && a.type() == b.type() && cv::countNonZero(a.reshape(1) != b.reshape(1)) == 0;
    };
    EXPECT_TRUE(Same(last.maps.trav_r, full.trav_r));
    EXPECT_TRUE(Same(last.maps.trav_a, full.trav_a));
    EXPECT_TRUE(Same(last.maps.trav_r_strict, full.trav_r_strict));
    EXPECT_TRUE(Same(last.maps.slab, full.slab));
    EXPECT_TRUE(Same(last.maps.clear_r, full.clear_r));
    EXPECT_TRUE(Same(last.maps.clear_a, full.clear_a));
    EXPECT_TRUE(Same(last.maps.risk, full.risk));
    EXPECT_GT(cv::countNonZero(full.slab == kFree), 100) << "the views must observe free space, or this proves little";
}
