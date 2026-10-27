#include "planning/mission/phase4_planner.hpp"

#include "planning/mission/allocation_wire.hpp"
#include "planning/route/visibility_graph.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace ariadne::planning {

bool RegionDirective::Covers(double px, double py) const {
    bool in = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto [xi, yi] = polygon[i];
        const auto [xj, yj] = polygon[j];
        if ((yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) in = !in;
    }
    return in;
}

double RegionDirective::Distance(double px, double py) const {
    if (Covers(px, py)) return 0;
    double best = 1e300;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const double ax = polygon[j].first, ay = polygon[j].second, bx = polygon[i].first, by = polygon[i].second;
        const double dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
        const double t = L2 > 0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0) : 0.0;
        best = std::min(best, std::hypot(px - (ax + t * dx), py - (ay + t * dy)));
    }
    return best;
}

void MaskKeepOut(int agent, const std::vector<RegionDirective>& dirs, const GridFrame& g, cv::Mat* plan) {
    for (const auto& d : dirs) {
        if (d.kind != "keep_out" || (d.agent >= 0 && d.agent != agent)) continue;
        std::vector<cv::Point> poly;   // grid (col, row) of each vertex: row 0 = south, as every layer here
        for (const auto& [x, y] : d.polygon)
            poly.emplace_back(int(std::lround((x - g.ox) / g.cell - 0.5)), int(std::lround((y - g.oy) / g.cell - 0.5)));
        cv::fillPoly(*plan, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(0));
    }
}

void ApplyDirectives(int agent, const std::vector<RegionDirective>& dirs,
                     const std::vector<std::pair<double, double>>& refused, double refused_radius_m, double observe_m,
                     Phase4AgentPlan* ap) {
    auto& opts = ap->nbv.options;
    if (ap->nbv.reserve_rule) return;   // the hard rule's {return, hold} is outside every directive (§21.4)
    std::vector<TaskCandidate> kept;
    std::vector<char> used(dirs.size(), 0);
    // assign / explore restrict only when at least one option's goal lies inside (B's semantics): an agent is never
    // left without a task because its region holds no candidate yet.
    std::vector<char> restrict_ok(dirs.size(), 0);
    for (size_t k = 0; k < dirs.size(); ++k) {
        const auto& d = dirs[k];
        if ((d.kind != "assign" && d.kind != "explore") || (d.agent >= 0 && d.agent != agent)) continue;
        for (const auto& c : opts) restrict_ok[k] |= d.Covers(c.goal_xy.first, c.goal_xy.second);
    }
    for (auto c : opts) {
        const auto [x, y] = c.goal_xy;
        bool drop = false;
        for (const auto& r : refused) {
            if (std::hypot(x - r.first, y - r.second) <= refused_radius_m) {
                drop = true;
                ++ap->excluded_refused;
                break;
            }
        }
        if (drop) continue;
        for (size_t k = 0; k < dirs.size() && !drop; ++k) {
            const auto& d = dirs[k];
            if (d.agent >= 0 && d.agent != agent) continue;
            if (d.kind == "keep_out" && d.Covers(x, y)) drop = true, used[k] = 1;
            if ((d.kind == "assign" || d.kind == "explore") && restrict_ok[k] && !d.Covers(x, y)) drop = true, used[k] = 1;
            const auto [ox, oy] = c.centroid_xy;
            if (d.kind == "priority_region" && (d.Covers(x, y) || d.Distance(ox, oy) <= observe_m)) {
                c.utility += (d.weight - 1.0) * c.gain;   // the task value G scales; the costs do not
                c.gain *= d.weight;
                used[k] = 1;
            }
        }
        if (drop) {
            ++ap->excluded_directive;
            continue;
        }
        kept.push_back(c);
    }
    std::stable_sort(kept.begin(), kept.end(), [](const TaskCandidate& a, const TaskCandidate& b) {
        if (a.utility != b.utility) return a.utility > b.utility;
        if (a.path_m != b.path_m) return a.path_m < b.path_m;
        return a.task_id < b.task_id;
    });
    opts = std::move(kept);
    for (size_t k = 0; k < dirs.size(); ++k)
        if (used[k]) ap->applied.push_back(dirs[k].id);
}

void JointPick(Phase4Plan* p, const std::vector<int>& component) {
    const size_t n = p->agents.size();
    p->joint.assign(n, -1);
    auto comp = [&](size_t a) { return component.empty() ? 0 : component.at(a); };
    auto best = [&](size_t a) { const auto& o = p->agents[a].nbv.options; return o.empty() ? -1e300 : o[0].utility; };
    std::vector<size_t> order(n);
    for (size_t a = 0; a < n; ++a) order[a] = a;
    std::stable_sort(order.begin(), order.end(), [&](size_t x, size_t y) { return best(x) > best(y); });
    std::map<int, std::set<int64_t>> taken;   // per component; task ids are non-negative (TaskId): none is "no task"
    std::set<int> started;                    // components whose first picker has picked
    for (size_t k = 0; k < n; ++k) {
        const auto& o = p->agents[order[k]].nbv.options;
        const int c = comp(order[k]);
        int pick = -1;
        if (started.insert(c).second) pick = o.empty() ? -1 : 0;   // a component's first picker takes its best option
        else
            for (size_t i = 0; i < o.size() && pick < 0; ++i)   // OtherTask's rule, over its component's taken tasks
                if (!taken[c].count(o[i].task_id) && o[i].risk <= 1e300) pick = int(i);
        p->joint[order[k]] = pick;
        if (pick >= 0) taken[c].insert(o[size_t(pick)].task_id);
    }
}

std::vector<Cell> StringPull(const cv::Mat& plan, const std::vector<Cell>& path) {
    std::vector<Cell> out;
    if (path.empty()) return out;
    auto Clear = [&](const Cell& a, const Cell& b) { return LineOfSight(plan, a, b); };   // 4-connected, both ways
    size_t i = 0;
    out.push_back(path[0]);
    while (i + 1 < path.size()) {
        size_t j = i + 1;   // adjacent path cells are always mutually visible (A*'s corner rule)
        for (size_t k = path.size() - 1; k > i + 1; --k)
            if (Clear(path[i], path[k])) {
                j = k;
                break;
            }
        out.push_back(path[j]);
        i = j;
    }
    return out;
}

Phase4Planner::Phase4Planner(const Phase4Briefing& b, const Phase4Params& p, int n_agents)
    : b_(b), p_(p), map_(p.tsdf), esdf_(p.tsdf.voxel, p.trav.responder.rho + 2 * p.tsdf.voxel), fr_(p.frontiers),
      egress_(b.target, b.entrances, b.grid.cell) {
    if (n_agents < 1) throw std::invalid_argument("Phase4Planner: n_agents must be >= 1");
    // The mission grid bounds what the planner can use: size the ESDF to it once, so exploring never forces a
    // global rebuild (values are the same either way, esdf.hpp).
    esdf_.Reserve(map_.ToVoxel(core::Vec3(b.grid.ox, b.grid.oy, 0.0)),
                  map_.ToVoxel(core::Vec3(b.grid.ox + b.grid.cols * b.grid.cell, b.grid.oy + b.grid.rows * b.grid.cell,
                                          b.z_max_m)));
    track_.resize(size_t(n_agents));
    last_pick_.assign(size_t(n_agents), -1);
    shared_.assign(size_t(n_agents), std::vector<size_t>(size_t(n_agents), 0));
}

size_t Phase4Planner::Integrate(int agent, int64_t /*t_ns*/, const DepthFrame& f) {
    if (agent < 0 || agent >= Agents()) throw std::out_of_range("Phase4Planner::Integrate: agent " + std::to_string(agent));
    const std::vector<Idx3> dirty = map_.Integrate(f);
    if (esdf_.Empty()) esdf_.Rebuild(map_);
    else esdf_.Update(map_, dirty);
    if (!dirty.empty()) {
        Idx3 lo{1 << 30, 1 << 30, 1 << 30}, hi{-(1 << 30), -(1 << 30), -(1 << 30)};
        for (const auto& bk : dirty) {
            lo = {std::min(lo.x, bk.x * kBlockSide), std::min(lo.y, bk.y * kBlockSide), std::min(lo.z, bk.z * kBlockSide)};
            hi = {std::max(hi.x, bk.x * kBlockSide + kBlockSide - 1), std::max(hi.y, bk.y * kBlockSide + kBlockSide - 1),
                  std::max(hi.z, bk.z * kBlockSide + kBlockSide - 1)};
        }
        if (!detected_) fr_.Detect(map_), detected_ = true;
        else fr_.Update(map_, lo, hi);
    }
    track_[agent].push_back(f.T_W_C.t);
    return dirty.size();
}

Cell Phase4Planner::CellOf(const core::Vec3& p) const {
    return Cell{int(std::floor((p.y() - b_.grid.oy) / b_.grid.cell)), int(std::floor((p.x() - b_.grid.ox) / b_.grid.cell))};
}

cv::Mat Phase4Planner::PlanSpace(const TraversabilityMaps& m, const Cell& at) const {
    cv::Mat plan = KnownFreePlan(m, p_.nbv.rho_a);   // known-free agent space (D153b), as PlanNbv
    const double esc = p_.nbv.escape_m / b_.grid.cell;
    for (int r = std::max(0, at.r - int(esc) - 1); r <= std::min(plan.rows - 1, at.r + int(esc) + 1); ++r)
        for (int c = std::max(0, at.c - int(esc) - 1); c <= std::min(plan.cols - 1, at.c + int(esc) + 1); ++c)
            if (std::hypot(r - at.r, c - at.c) <= esc && m.slab.at<uint8_t>(r, c) != kOccupied) plan.at<uint8_t>(r, c) = 1;
    return plan;
}

Phase4Plan Phase4Planner::Plan(double t_s, const std::vector<Phase4AgentInput>& agents,
                                const std::vector<RegionDirective>& dirs) {
    const int n = Agents();
    if (int(agents.size()) != n)
        throw std::invalid_argument("Phase4Planner::Plan: " + std::to_string(agents.size()) + " inputs for " +
                                    std::to_string(n) + " agents");
    Phase4Plan out;
    out.t_s = t_s;
    out.agents.resize(size_t(n));
    out.joint.assign(size_t(n), -1);
    // The comms graph: an edge between two active, linked agents (N = 2: the one "both linked" edge); its connected
    // components bound what can be coordinated (JointPick) and shared (peers) this replan.
    std::vector<std::vector<int>> G(size_t(n), std::vector<int>(size_t(n), 0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            G[size_t(i)][size_t(j)] = i != j && agents[size_t(i)].active && agents[size_t(j)].active &&
                                      agents[size_t(i)].linked && agents[size_t(j)].linked;
    std::vector<int> component(size_t(n), -1);
    int n_comp = 0;
    for (int s = 0; s < n; ++s) {
        if (component[size_t(s)] >= 0) continue;
        std::vector<int> stack{s};
        component[size_t(s)] = s;   // labelled by its lowest member
        while (!stack.empty()) {
            const int u = stack.back();
            stack.pop_back();
            for (int v = 0; v < n; ++v)
                if (G[size_t(u)][size_t(v)] && component[size_t(v)] < 0) component[size_t(v)] = s, stack.push_back(v);
        }
        if (agents[size_t(s)].active) ++n_comp;   // counted over the active agents
    }
    // Peers are what an agent has RECEIVED: another agent's track up to the last replan they shared a component
    // [PROXY]: synced at replans, the map itself stays one fused map (partition_note).
    for (int a = 0; a < n; ++a)
        for (int b = 0; b < n; ++b)
            if (a != b && component[size_t(a)] == component[size_t(b)]) shared_[size_t(a)][size_t(b)] = track_[size_t(b)].size();
    out.allocation.components = std::max(1, n_comp);
    if (n_comp > 1)
        out.allocation.partition_note = "allocation partition-aware; map and peers centralised [PROXY]";
    PlanMaps(&out);
    const CoarseLayer coarse = BuildCoarse(map_);
    for (int a = 0; a < n; ++a) {
        if (!agents[size_t(a)].active) continue;
        std::vector<PeerKeyframe> peers;
        for (int b = 0; b < n; ++b)   // every other agent's received track, ascending index (N = 2, linked: the other's)
            if (b != a)
                for (size_t i = 0; i < shared_[size_t(a)][size_t(b)]; ++i) peers.push_back({b, int64_t(i), track_[size_t(b)][i]});
        out.agents[size_t(a)] = PlanAgent(t_s, a, agents[size_t(a)], std::move(peers), nullptr, false, dirs, out, coarse);
    }
    Allocate(agents, G, component, &out);
    return out;
}

void Phase4Planner::PlanMaps(Phase4Plan* out) {
    out->maps = ComputeTraversability(map_, esdf_, b_.grid, p_.trav);
    out->support = BodySupport(map_, b_.grid, p_.trav.responder, p_.support_band_lo, p_.support_band_hi);
    EgressWeights w = p_.egress;
    if (!p_.risk_calibrated) w.w_r = 0;   // invariant: the [PROXY] enters only after D112's bar
    const cv::Mat pen = EgressPenalty(out->maps.clear_r, out->support, out->maps.risk, w);
    out->egress = egress_.Replan(out->maps.trav_r, pen, out->support, out->maps.risk);
}

Phase4AgentPlan Phase4Planner::PlanAgent(double t_s, int a, const Phase4AgentInput& in, std::vector<PeerKeyframe> peers,
                                         const cv::Mat* peer_known, bool footprints, const std::vector<RegionDirective>& dirs,
                                         const Phase4Plan& shared, const CoarseLayer& coarse) const {
    Phase4AgentPlan ap;
    NbvInputs ni;
    ni.map = &map_, ni.frontiers = &fr_, ni.coarse = &coarse, ni.trav = &shared.maps, ni.grid = b_.grid;
    ni.route = &shared.egress, ni.route_support = &shared.support, ni.responder = p_.trav.responder;
    ni.entrances = b_.entrances, ni.risk_calibrated = p_.risk_calibrated, ni.t_s = t_s;
    ni.peers = std::move(peers);
    ni.peer_known = peer_known;
    ni.footprints = footprints;
    ni.bounds.lo = core::Vec3(b_.grid.ox, b_.grid.oy, 0.0);
    ni.bounds.hi = core::Vec3(b_.grid.ox + b_.grid.cols * b_.grid.cell, b_.grid.oy + b_.grid.rows * b_.grid.cell, b_.z_max_m);
    // keep_out masks the agent's plan space before NBV, so neither a view nor a path to it can use the region.
    cv::Mat ones(shared.maps.trav_a.size(), CV_8U, cv::Scalar(1));
    MaskKeepOut(a, dirs, b_.grid, &ones);
    ap.keep_out = ones == 0;
    TraversabilityMaps masked = shared.maps;
    masked.trav_a = shared.maps.trav_a.clone();
    masked.trav_a.setTo(0, ap.keep_out);
    ni.trav = &masked;
    AgentState st;
    st.position = in.position, st.e_used_m = in.flown_m, st.own_keyframes = track_[size_t(a)];
    ap.plan = PlanSpace(shared.maps, CellOf(in.position));
    ap.plan.setTo(0, ap.keep_out);   // the escape disc never re-opens a keep_out
    // D117: C_time is priced on the command Route will send from here: the same plan space, the same cut.
    const ViewRouter router = [this, &ap, from = CellOf(in.position)](const std::pair<double, double>& goal) {
        return Command(ap.plan, goal, from);
    };
    ni.router = &router;
    ni.heading = in.yaw;
    ap.nbv = PlanNbv(ni, a, st, p_.nbv);
    ApplyDirectives(a, dirs, in.refused_goals, p_.refused_radius_m, p_.nbv.frustum.max_range_m, &ap);
    return ap;
}

Phase4Plan Phase4Planner::PlanOwn(double t_s, int self, const Phase4AgentInput& in, const std::vector<PeerKeyframe>& peers,
                                  const cv::Mat* peer_known, const std::vector<RegionDirective>& dirs) {
    if (self < 0 || self >= Agents()) throw std::out_of_range("Phase4Planner::PlanOwn: agent " + std::to_string(self));
    Phase4Plan out;
    out.t_s = t_s;
    out.agents.resize(size_t(Agents()));
    out.joint.assign(size_t(Agents()), -1);
    out.allocation.method = "broker";   // allocated by the component's broker over TASK_BID (D175), not here
    PlanMaps(&out);
    if (!in.active) return out;
    const CoarseLayer coarse = BuildCoarse(map_);
    out.agents[size_t(self)] = PlanAgent(t_s, self, in, peers, peer_known, true, dirs, out, coarse);
    return out;
}

AuctionInstance BuildAuction(const Phase4Plan& p, const GridFrame& g, int max_bundle, std::vector<float>* scales,
                             std::vector<std::vector<int>>* view) {
    // Common scales: per term, the max over agents of that agent's p90 (P90Scale over its own options).
    const int n = int(p.agents.size());
    std::array<double, 8> P{};
    for (int a = 0; a < n; ++a) {
        const auto& o = p.agents[a].nbv.options;
        for (int k = 0; k < 8; ++k) {
            std::vector<double> v;
            for (const auto& c : o) v.push_back(k < 5 ? c.ig_raw[size_t(k)] : (k == 5 ? c.path_m : c.cost[k == 6 ? 1 : 4]));
            P[size_t(k)] = std::max(P[size_t(k)], P90Scale(v));
        }
    }
    scales->assign(P.begin(), P.end());
    std::vector<int64_t> ids;   // the union of task ids, sorted (deterministic task order)
    for (int a = 0; a < n; ++a)
        for (const auto& c : p.agents[a].nbv.options)
            if (c.source != TaskSource::kReturn && c.source != TaskSource::kHold) ids.push_back(c.task_id);
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    AuctionInstance in;
    in.n_agents = n, in.max_bundle = max_bundle, in.task_ids = ids;
    const size_t N = ids.size();
    in.value.assign(size_t(n), std::vector<double>(N, 0.0));
    in.d0.assign(size_t(n), std::vector<double>(N, 0.0));
    in.leg.assign(size_t(n), std::vector<std::vector<double>>(N, std::vector<double>(N, 0.0)));
    view->assign(size_t(n), std::vector<int>(N, -1));
    for (int a = 0; a < n; ++a) {
        in.rate.push_back(P[5] > 0 ? p.agents[a].nbv.lambda_path / P[5] : 0.0);
        const auto& o = p.agents[a].nbv.options;
        for (size_t i = 0; i < o.size(); ++i) {   // options are sorted by utility: the first view of a task is its best
            const size_t j = size_t(std::lower_bound(ids.begin(), ids.end(), o[i].task_id) - ids.begin());
            if (j >= N || ids[j] != o[i].task_id || (*view)[size_t(a)][j] >= 0) continue;
            double g_common = 0;
            for (int k = 0; k < 5; ++k) g_common += P[size_t(k)] > 0 ? o[i].ig_raw[size_t(k)] / P[size_t(k)] : 0.0;
            const double cb = P[6] > 0 ? o[i].cost[1] / P[6] : 0.0, cr = P[7] > 0 ? o[i].cost[4] / P[7] : 0.0;
            in.value[size_t(a)][j] = std::max(0.0, g_common - cb - cr);
            in.d0[size_t(a)][j] = o[i].path_m;
            (*view)[size_t(a)][j] = int(i);
        }
        if (max_bundle > 1 && !p.agents[a].plan.empty()) {   // goal -> goal legs on the agent's plan space
            for (size_t j = 0; j < N; ++j) {
                const int vj = (*view)[size_t(a)][j];
                if (vj < 0) continue;
                const auto& gj = o[size_t(vj)].goal_xy;
                const Cell cj{int(std::floor((gj.second - g.oy) / g.cell)), int(std::floor((gj.first - g.ox) / g.cell))};
                const cv::Mat d = GridDistances(p.agents[a].plan, cj);
                for (size_t k = 0; k < N; ++k) {
                    const int vk = (*view)[size_t(a)][k];
                    if (vk < 0 || k == j) continue;
                    const auto& gk = o[size_t(vk)].goal_xy;
                    const Cell ck{int(std::floor((gk.second - g.oy) / g.cell)), int(std::floor((gk.first - g.ox) / g.cell))};
                    const double dd = d.at<double>(ck.r, ck.c);
                    in.leg[size_t(a)][j][k] = std::isfinite(dd) ? dd * g.cell : 1e9;   // unreachable: discounted to ~0
                }
            }
        }
    }
    return in;
}

void Phase4Planner::Allocate(const std::vector<Phase4AgentInput>& agents, const std::vector<std::vector<int>>& G,
                             const std::vector<int>& component, Phase4Plan* out) {
    const int n = Agents();
    AllocationRecord& rec = out->allocation;
    rec.method = p_.allocator == Phase4Params::Allocator::kCbba ? "cbba"
                 : p_.allocator == Phase4Params::Allocator::kSga ? "sga" : "greedy";
    // The hard rule's {return, hold} is outside the auction: such an agent takes its first option.
    bool hard = false;
    for (int a = 0; a < n; ++a) hard = hard || out->agents[size_t(a)].nbv.reserve_rule;
    // Only the Ariadne row allocates by auction: the auction values are Ariadne's (D114 G on common scales), so a
    // baseline policy (random, coverage, uncertainty, TARE) fed to it would be re-scored as Ariadne and the §31.6 rows
    // would collapse. Baselines coordinate by the exactly-once greedy pick over their own ranking.
    const bool baseline = p_.nbv.policy != NbvPolicy::kAriadne;
    if (baseline) rec.method = "greedy";
    if (p_.allocator == Phase4Params::Allocator::kGreedy || baseline || hard) {
        JointPick(out, component);
        for (int a = 0; a < n; ++a)
            if (out->agents[size_t(a)].nbv.reserve_rule) out->joint[size_t(a)] = out->agents[size_t(a)].nbv.options.empty() ? -1 : 0;
        rec.trigger = hard ? "reserve_rule" : "every_replan";
        return;
    }
    std::vector<std::vector<int>> view;
    const AuctionInstance in = BuildAuction(*out, b_.grid, p_.max_bundle, &rec.scales, &view);
    rec.tasks = in.Tasks();
    const std::set<int64_t> tasks(in.task_ids.begin(), in.task_ids.end());
    // Event triggers (§21.7): a new or completed task (the task set changed), or a topology change.
    rec.trigger = !allocated_ ? "first" : (tasks != last_tasks_ ? "tasks" : (G != last_graph_ ? "topology" : "none"));
    if (rec.trigger == "none") {   // reuse the last assignment: each agent keeps its task while it still has a view
        for (int a = 0; a < n; ++a) {
            out->joint[size_t(a)] = -1;
            const auto& o = out->agents[size_t(a)].nbv.options;
            for (size_t i = 0; i < o.size(); ++i)
                if (o[i].task_id == last_pick_[size_t(a)]) {
                    out->joint[size_t(a)] = int(i);
                    break;
                }
        }
        return;
    }
    Allocation al;
    if (p_.allocator == Phase4Params::Allocator::kSga) {
        al = SequentialGreedy(in);
    } else {
        CbbaParams cp;
        cp.bytes = WireBytes(codec_, rec.scales);
        cp.transport = WireTransport(codec_, rec.scales);   // CBBA runs on exactly what the wire carries (D115a)
        al = RunCbba(in, [&](int) { return G; }, cp);
    }
    rec.rounds = al.rounds, rec.messages = al.messages, rec.bytes = al.bytes, rec.total = al.total;
    if (in.n_agents == 2 && in.max_bundle <= 2) rec.optimum = TwoAgentOptimum(in);
    for (int a = 0; a < n; ++a) {
        out->joint[size_t(a)] = -1;
        last_pick_[size_t(a)] = -1;
        if (!agents[size_t(a)].active) continue;
        const auto& A = al.agents[size_t(a)];
        if (A.path.empty()) continue;
        const int j = A.path.front();   // execute the first task on its path
        out->joint[size_t(a)] = view[size_t(a)][size_t(j)];
        last_pick_[size_t(a)] = in.task_ids[size_t(j)];
    }
    last_tasks_ = tasks, last_graph_ = G, allocated_ = true;
}


ViewCommand Phase4Planner::Command(const cv::Mat& plan, const std::pair<double, double>& goal_xy, const Cell& from) const {
    const GridFrame& g = b_.grid;
    const Cell goal{int(std::floor((goal_xy.second - g.oy) / g.cell)), int(std::floor((goal_xy.first - g.ox) / g.cell))};
    ViewCommand out;
    const GridPath gp = AStar(plan, from, {goal});
    if (!gp.found) return out;   // unreachable: the loop holds
    out.ok = true;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (gp.cells.size() < 2) {   // D125: an at-cell view is a turn here, not a hold
        out.wp = {{nan, nan}};
        return out;
    }
    const std::vector<Cell> keep = StringPull(plan, gp.cells);
    double px = g.ox + (keep[0].c + 0.5) * g.cell, py = g.oy + (keep[0].r + 0.5) * g.cell, left = p_.max_command_m;
    for (size_t k = 1; k < keep.size(); ++k) {
        if (left <= 1e-9) {   // vertices remain past the command length
            out.cut = true;
            break;
        }
        double x = g.ox + (keep[k].c + 0.5) * g.cell, y = g.oy + (keep[k].r + 0.5) * g.cell;
        const double d = std::hypot(x - px, y - py);
        if (d > left) x = px + (x - px) * left / d, y = py + (y - py) * left / d, out.cut = true;   // on a clear chord
        out.wp.emplace_back(x, y);
        left -= std::min(d, left);
        px = x, py = y;
    }
    return out;
}

std::pair<std::vector<std::pair<double, double>>, double> Phase4Planner::Route(const Phase4Plan& p, int agent, int i,
                                                                              const Cell& from) const {
    const auto& opts = p.agents[agent].nbv.options;
    if (i < 0 || size_t(i) >= opts.size()) return {{}, 0.0};
    const TaskCandidate& o = opts[size_t(i)];
    if (o.source == TaskSource::kHold) return {{}, o.goal_yaw};   // the reserve rule's hold: no command (not D125)
    cv::Mat plan = PlanSpace(p.maps, from);   // trav_a + the escape disc at `from` (D158)
    if (!p.agents[agent].keep_out.empty()) plan.setTo(0, p.agents[agent].keep_out);
    const ViewCommand c = Command(plan, o.goal_xy, from);
    if (!c.ok) return {{}, o.goal_yaw};
    // D119: the view's yaw only where the command ends at the view; D118: an arrival twin keeps its heading.
    const bool no_yaw = c.cut || o.keep_heading;
    return {c.wp, no_yaw ? std::numeric_limits<double>::quiet_NaN() : o.goal_yaw};
}

}  // namespace ariadne::planning
