#pragma once
// ---------------------------------------------------------------------------
// A1Planner: the Phase 4 planner (planning::Phase4Planner) behind the runtime's MissionPlanner seam
// (agentic.md §0b, option B). A field copy in both directions, nothing decided here:
//   Integrate  KeyframeInput -> planning::DepthFrame (the spool-scale depth and sigma, the camera pose in W)
//   Plan       PlanContext -> Phase4AgentInput[2] (active, linked = true, position, flown_m, refused goals) and the
//              region directives -> RegionDirective; Phase4Plan -> PlanOutput: TaskCandidate -> TaskOption (same
//              names), the joint pick, the maps the runtime renders (slab, egress route), and a trace record (the
//              allocation event, the reserve rule, which policy).
//   Route      Phase4Planner::Route on the plan of this replan (kept here: Route follows Plan within one barrier).
// `linked` is always true: the live runtime has no comms link state (the 3.3 comms node is not wired into L1); the
// trace says so. `flown_m` is the estimated odometry path length up to T (the rig's executed motion), computed by the
// runtime and named in the trace.
// ---------------------------------------------------------------------------
#include "planning/mission/phase4_planner.hpp"
#include "runtime/live/planner.hpp"

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ariadne::runtime::live {

/// planner-config (configs/mission/mission_v2.yaml) -> Phase4Params, the planner's key mapping (2026-10-23, def61de).
/// Live-config values the seam already owns (refused radius, command length, clearances) come from the runtime.
inline planning::Phase4Params LoadPhase4Params(const std::string& path, const mis::MissionConfig& mc, double refused_radius_m,
                                               double max_command_m, double escape_m, uint64_t run_seed,
                                               std::string* policy_name) {
    const YAML::Node n = YAML::LoadFile(path);
    planning::Phase4Params p;
    const std::string pol = n["planner"]["policy"].as<std::string>();
    using P = planning::NbvPolicy;
    if (pol == "ariadne") p.nbv.policy = P::kAriadne;
    else if (pol == "coverage_greedy") p.nbv.policy = P::kCoverageGreedy;
    else if (pol == "uncertainty_only") p.nbv.policy = P::kUncertaintyOnly;
    else if (pol == "random") p.nbv.policy = P::kRandom;
    else if (pol == "tare") p.nbv.policy = P::kTare;
    else throw std::invalid_argument("planner config: policy " + pol);
    *policy_name = pol;
    // the run's seed mixed in ("mix in the run's seed"): splitmix64 of the pair, so seeds 0..2 differ per policy seed
    uint64_t z = n["planner"]["random_seed"].as<uint64_t>() * 0x9E3779B97F4A7C15ULL + run_seed;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    p.nbv.random_seed = z ^ (z >> 31);
    const std::string alloc = n["planner"]["allocator"].as<std::string>();
    using A = planning::Phase4Params::Allocator;
    if (alloc == "cbba") p.allocator = A::kCbba;
    else if (alloc == "sga") p.allocator = A::kSga;
    else if (alloc == "greedy") p.allocator = A::kGreedy;
    else throw std::invalid_argument("planner config: allocator " + alloc);
    p.max_bundle = n["planner"]["max_bundle"].as<int>();
    p.risk_calibrated = n["planner"]["risk_calibrated"].as<bool>();
    const YAML::Node e = n["energy"];
    p.nbv.e_total_m = e["e_total_m"].as<double>();
    p.nbv.e_per_m = e["e_per_m"].as<double>();
    p.nbv.speed_mps = e["speed_mps"].as<double>();
    p.nbv.reserve = e["reserve"].as<double>();
    p.nbv.mission_T_s = e["end_s"].as<double>();
    p.nbv.frustum.grid = n["nbv"]["rays_grid"].as<int>();
    p.nbv.frustum.rank_grid = n["nbv"]["rank_grid"].as<int>();
    p.nbv.tare_subspace_m = n["nbv"]["tare_subspace_m"].as<double>();
    // D117/D118/D120 keys (next-best-view.md §15.6); absent = the gated behaviour (cost_time none, view_yaw target, no log).
    const std::string ct = n["nbv"]["cost_time"] ? n["nbv"]["cost_time"].as<std::string>() : "none";
    if (ct == "exact_command") p.nbv.cost_time = true;
    else if (ct != "none") throw std::invalid_argument("planner config: nbv.cost_time " + ct);
    const std::string vy = n["nbv"]["view_yaw"] ? n["nbv"]["view_yaw"].as<std::string>() : "target";
    if (vy == "target+arrival") p.nbv.arrival_twins = true;
    else if (vy != "target") throw std::invalid_argument("planner config: nbv.view_yaw " + vy);
    if (p.nbv.arrival_twins && !p.nbv.cost_time) throw std::invalid_argument("planner config: view_yaw needs cost_time");
    p.nbv.term_log = n["nbv"]["term_log"] && n["nbv"]["term_log"].as<bool>();
    p.nbv.rho_r = mc.rho_r, p.nbv.rho_a = mc.rho_a, p.nbv.escape_m = escape_m;
    p.refused_radius_m = refused_radius_m;
    p.max_command_m = max_command_m;
    return p;
}

/// The runtime's keyframe as Phase 4's depth frame (views into the keyframe's depth and sigma; no copy).
inline planning::DepthFrame ToDepthFrame(const KeyframeInput& k) {
    planning::DepthFrame f;
    f.T_W_C = k.T_W_C;
    f.fx = k.fx, f.fy = k.fy, f.cx = k.cx, f.cy = k.cy;
    f.width = k.Z.cols, f.height = k.Z.rows;
    f.depth = k.Z.ptr<float>();
    f.sigma = k.S.empty() ? nullptr : k.S.ptr<float>();
    f.row_stride = k.Z.step1();
    return f;
}

/// The runtime's briefing as Phase 4's.
inline planning::Phase4Briefing Phase4BriefFrom(const mis::MissionConfig& mc, const mis::Briefing& b) {
    planning::Phase4Briefing pb;
    pb.grid.ox = b.gs.origin_x, pb.grid.oy = b.gs.origin_y, pb.grid.cell = b.gs.cell;
    pb.grid.cols = b.gs.nx, pb.grid.rows = b.gs.ny;
    pb.target = b.target;
    pb.entrances = b.exits;
    pb.z_max_m = mc.z_max;
    return pb;
}

class A1Planner : public MissionPlanner {
public:
    A1Planner(const mis::MissionConfig& mc, const mis::Briefing& b, const planning::Phase4Params& p, std::string policy,
              double autonomous_threshold, int n_agents = 2)
        : b_(b), policy_(std::move(policy)), threshold_(autonomous_threshold), term_log_(p.nbv.term_log),
          planner_(Phase4BriefFrom(mc, b), p, n_agents) {}

    size_t Integrate(const KeyframeInput& k) override { return planner_.Integrate(k.agent, k.t_ns, ToDepthFrame(k)); }

    PlanOutput Plan(const PlanContext& ctx) override {
        std::vector<planning::Phase4AgentInput> in(size_t(ctx.n()));   // P8 D213: N agents end to end
        for (int a = 0; a < ctx.n(); ++a) {
            in[a].active = ctx.active[a];
            in[a].linked = true;
            in[a].position = ctx.position[a];
            in[a].yaw = ctx.yaw[size_t(a)];
            in[a].flown_m = ctx.flown_m[a];
            for (const auto& [k, g] : ctx.refused_goals) if (k == a) in[a].refused_goals.push_back(g);
        }
        std::vector<planning::RegionDirective> dirs;
        for (const auto& d : ctx.directives) {
            if (d.kind != "priority_region" && d.kind != "keep_out" && d.kind != "assign" && d.kind != "explore") continue;
            planning::RegionDirective r;
            r.id = d.id, r.kind = d.kind, r.agent = d.agent, r.polygon = d.polygon, r.weight = d.weight;
            dirs.push_back(std::move(r));
        }
        plan_ = planner_.Plan(ctx.T * 1e-9, in, dirs);

        PlanOutput out(ctx.n());
        out.maps.slab = plan_.maps.slab;
        out.maps.trav_r = plan_.maps.trav_r;
        out.maps.trav_a = plan_.maps.trav_a;
        out.maps.support = plan_.support;
        out.maps.egress.found = plan_.egress.found;
        out.maps.egress.cells = plan_.egress.cells;
        out.maps.egress.goal_index = plan_.egress.exit;
        out.maps.egress.length_cells = plan_.egress.length_m / b_.gs.cell;
        std::ostringstream tj;
        tj.precision(10);
        const auto& al = plan_.allocation;
        tj << "{\"name\":\"a1_phase4\",\"policy\":\"" << policy_ << "\",\"linked\":true,\"linked_source\":\"constant: no comms "
           << "link state in L1\",\"flown_m\":[" << [&] {   // D225: N agents (this trace line was still 2-agent after D213)
               std::ostringstream fm;
               fm.precision(10);
               for (size_t q = 0; q < ctx.flown_m.size(); ++q) fm << (q ? "," : "") << ctx.flown_m[q];
               return fm.str();
           }() << "],\"flown_source\":\"estimated odometry path "
           << "length to T\",\"allocation\":{\"method\":\"" << al.method << "\",\"trigger\":\"" << al.trigger << "\",\"tasks\":"
           << al.tasks << ",\"rounds\":" << al.rounds << ",\"messages\":" << al.messages << ",\"bytes\":" << al.bytes
           << ",\"total\":" << al.total << ",\"optimum\":" << al.optimum << "},\"egress\":{\"confidence\":"
           << plan_.egress.confidence << ",\"reroute\":\"" << plan_.egress.reroute << "\"},\"agents\":[";
        for (int a = 0; a < ctx.n(); ++a) {
            const auto& ap = plan_.agents[size_t(a)];
            AgentOptions& ao = out.agents[a];
            for (const auto& t : ap.nbv.options) {
                TaskOption o;
                o.task_id = t.task_id, o.goal_xy = t.goal_xy, o.goal_yaw = t.goal_yaw, o.path_m = t.path_m;
                o.standoff_m = t.standoff_m, o.centroid_xy = t.centroid_xy, o.cells = t.cells;
                o.risk = t.risk, o.risk_reason = t.risk_reason;
                o.supervisor_return = ap.nbv.reserve_rule && t.source == planning::TaskSource::kReturn;
                o.utility = t.utility;
                o.c.cluster = int(t.task_id & 0x7fffffff);   // only for the runtime's map rendering
                o.c.goal = b_.ToCell(t.goal_xy.first, t.goal_xy.second);
                o.c.cells = t.cells;
                ao.options.push_back(std::move(o));
            }
            ao.excluded_refused = ap.excluded_refused, ao.excluded_directive = ap.excluded_directive;
            ao.applied = ap.applied;
            ao.plan = ap.plan;
            // The planner's own pick is its "classical" choice (the ablation's no-chooser row) when within the gate;
            // otherwise the best option within it.
            const int j = plan_.joint[a];
            const bool takeable = j >= 0 && (ao.options[size_t(j)].risk <= threshold_ || ao.options[size_t(j)].supervisor_return);
            ao.classical = takeable ? j : OtherTask(out, a, -1, threshold_);
            out.joint[a] = j;
            tj << (a ? "," : "") << "{\"reserve_rule\":" << (ap.nbv.reserve_rule ? "true" : "false")
               << ",\"supervisor_event\":\"" << ap.nbv.supervisor_event << "\",\"tare\":\"" << ap.nbv.tare << "\"";
            if (term_log_) TermLog(ap.nbv, tj);
            tj << "}";
        }
        tj << "]}";
        out.trace_json = tj.str();
        return out;
    }

    std::pair<std::vector<std::pair<double, double>>, double> Route(const PlanOutput&, int agent, int i,
                                                                   const Cell& from) const override {
        return planner_.Route(plan_, agent, i, from);
    }

    /// D175 item 4: this instance as agent `self`'s OWN planner (fed only its keyframes via Integrate). PlanOwn
    /// with the peers it RECEIVED and the peers' known-mask; only agents[self] is filled, joint[self] = -1 (the
    /// component broker assigns; ApplyConsensus's pick replaces it). Maps/egress are this agent's own map.
    PlanOutput PlanOwnFor(const PlanContext& ctx, int self, const std::vector<planning::PeerKeyframe>& peers,
                          const cv::Mat* peer_known, bool linked = true) {
        planning::Phase4AgentInput in;
        in.active = ctx.active[size_t(self)];
        in.linked = linked;   // D174: the fleet's comms state; true before collab runs (coordination-only rows)
        in.position = ctx.position[size_t(self)];
        in.yaw = ctx.yaw[size_t(self)];
        in.flown_m = ctx.flown_m[size_t(self)];
        for (const auto& [k, g] : ctx.refused_goals) if (k == self) in.refused_goals.push_back(g);
        std::vector<planning::RegionDirective> dirs;
        for (const auto& d : ctx.directives) {
            if (d.kind != "priority_region" && d.kind != "keep_out" && d.kind != "assign" && d.kind != "explore") continue;
            planning::RegionDirective r;
            r.id = d.id, r.kind = d.kind, r.agent = d.agent, r.polygon = d.polygon, r.weight = d.weight;
            dirs.push_back(std::move(r));
        }
        plan_ = planner_.PlanOwn(ctx.T * 1e-9, self, in, peers, peer_known, dirs);
        PlanOutput out(ctx.n());
        out.maps.slab = plan_.maps.slab;
        out.maps.trav_r = plan_.maps.trav_r;
        out.maps.trav_a = plan_.maps.trav_a;
        out.maps.support = plan_.support;
        out.maps.egress.found = plan_.egress.found;
        out.maps.egress.cells = plan_.egress.cells;
        out.maps.egress.goal_index = plan_.egress.exit;
        out.maps.egress.length_cells = plan_.egress.length_m / b_.gs.cell;
        const auto& ap = plan_.agents[size_t(self)];
        AgentOptions& ao = out.agents[size_t(self)];
        for (const auto& t : ap.nbv.options) {
            TaskOption o;
            o.task_id = t.task_id, o.goal_xy = t.goal_xy, o.goal_yaw = t.goal_yaw, o.path_m = t.path_m;
            o.standoff_m = t.standoff_m, o.centroid_xy = t.centroid_xy, o.cells = t.cells;
            o.risk = t.risk, o.risk_reason = t.risk_reason;
            o.supervisor_return = ap.nbv.reserve_rule && t.source == planning::TaskSource::kReturn;
            o.utility = t.utility;
            o.c.cluster = int(t.task_id & 0x7fffffff);
            o.c.goal = b_.ToCell(t.goal_xy.first, t.goal_xy.second);
            o.c.cells = t.cells;
            ao.options.push_back(std::move(o));
        }
        ao.excluded_refused = ap.excluded_refused, ao.excluded_directive = ap.excluded_directive;
        ao.applied = ap.applied;
        ao.plan = ap.plan;
        ao.classical = OwnBest(ao);
        out.joint[size_t(self)] = -1;
        return out;
    }
    /// The raw plan of the last Plan/PlanOwnFor on THIS instance (BuildBid reads its NbvPlan; the exporter its maps).
    const planning::Phase4Plan& LastPlan() const { return plan_; }

private:
    /// D120: every candidate's normalised IG terms, C_path and C_time (raw, with their p90 scales), turn time and U,
    /// in the options' final order, so a term's share of each decision is read from the trace, not inferred.
    static void TermLog(const planning::NbvPlan& nb, std::ostringstream& tj) {
        // time_model: what time_s means in this row (D117's command time, or the gated path / speed), so the
        // predicted-vs-realised analysis never mixes the two.
        tj << ",\"terms\":{\"time_model\":\"" << (nb.timed ? "exact_command" : "path_over_speed")
           << "\",\"lambda_path\":" << nb.lambda_path << ",\"lambda_t\":" << nb.lambda_t << ",\"path_scale\":"
           << nb.path_scale << ",\"time_scale\":" << nb.time_scale << ",\"unroutable\":" << nb.unroutable
           << ",\"candidates\":[";
        for (size_t i = 0; i < nb.options.size(); ++i) {
            const auto& t = nb.options[i];
            tj << (i ? "," : "") << "{\"task\":" << t.task_id << ",\"xy\":[" << t.goal_xy.first << "," << t.goal_xy.second
               << "],\"yaw\":" << t.goal_yaw << ",\"arrival\":" << (t.keep_heading ? "true" : "false") << ",\"ig\":[";
            for (int k = 0; k < 5; ++k) tj << (k ? "," : "") << t.ig_norm[size_t(k)];
            tj << "],\"gain\":" << t.gain << ",\"path_m\":" << t.path_m << ",\"time_s\":" << t.cost[3]
               << ",\"turn_s\":" << t.turn_s << ",\"u\":" << t.utility << "}";
        }
        tj << "]}";
    }

    /// The agent's own best takeable option under the risk gate (the uncoordinated fallback's "classical").
    int OwnBest(const AgentOptions& ao) const {
        for (int i = 0; i < int(ao.options.size()); ++i)
            if (ao.options[size_t(i)].risk <= threshold_ || ao.options[size_t(i)].supervisor_return) return i;
        return -1;
    }

    mis::Briefing b_;
    std::string policy_;
    double threshold_;
    bool term_log_;
    planning::Phase4Planner planner_;
    planning::Phase4Plan plan_;
};

}  // namespace ariadne::runtime::live
