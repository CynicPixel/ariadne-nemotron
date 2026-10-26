#include "planning/mission/multiagent.hpp"

#include "planning/volumetric/clearance.hpp"
#include "planning/volumetric/occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <stdexcept>

namespace ariadne::planning::multi {

uint32_t CoordinationToken(int32_t broker, std::vector<int32_t> members) {
    std::sort(members.begin(), members.end());
    members.erase(std::unique(members.begin(), members.end()), members.end());
    uint32_t h = 2166136261u;
    const auto mix = [&](uint32_t value) {
        for (int b = 0; b < 4; ++b) { h ^= (value >> (8 * b)) & 255u; h *= 16777619u; }
    };
    mix(uint32_t(broker)); mix(uint32_t(members.size()));
    for (int32_t member : members) mix(uint32_t(member));
    return h;
}

bool MatchesComponent(const TaskConsensus& consensus, const ComponentInfo& component) {
    auto actual = consensus.members, expected = component.members;
    std::sort(actual.begin(), actual.end()); std::sort(expected.begin(), expected.end());
    return consensus.broker == component.broker && actual == expected && consensus.epoch == component.epoch;
}

cv::Mat EncodeCells(const TraversabilityMaps& m) {
    cv::Mat out(m.slab.size(), CV_8U);
    for (int r = 0; r < m.slab.rows; ++r) {   // row pointers: one pass, no per-pixel lookups
        const uint8_t* s = m.slab.ptr<uint8_t>(r);
        const uint8_t* t = m.trav_r.ptr<uint8_t>(r);
        uint8_t* o = out.ptr<uint8_t>(r);
        for (int c = 0; c < m.slab.cols; ++c)
            o[c] = s[c] == kOccupied ? kCellOccupied : s[c] == kFree ? (t[c] ? kCellFreeResponder : kCellFree) : kCellUnknown;
    }
    return out;
}

SlabExporter::SlabExporter(int sender, const GridFrame& g)
    : sender_(sender), g_(g), bw_((g.cols + kBlock - 1) / kBlock), bh_((g.rows + kBlock - 1) / kBlock),
      codes_(g.rows, g.cols, CV_8U, cv::Scalar(kCellUnknown)), changed_(size_t(bw_ * bh_), 0) {}

uint64_t SlabExporter::Update(const TraversabilityMaps& m) {
    const cv::Mat now = EncodeCells(m);
    if (now.size() != codes_.size()) throw std::invalid_argument("SlabExporter: maps are not on the briefing grid");
    bool any = false;
    for (int by = 0; by < bh_; ++by)
        for (int bx = 0; bx < bw_; ++bx) {
            const int x0 = bx * kBlock, y0 = by * kBlock, w = std::min(kBlock, g_.cols - x0), h = std::min(kBlock, g_.rows - y0);
            bool diff = false;   // row memcmp, then copy only a changed block's rows
            for (int r = 0; r < h && !diff; ++r) diff = std::memcmp(now.ptr<uint8_t>(y0 + r) + x0, codes_.ptr<uint8_t>(y0 + r) + x0, size_t(w)) != 0;
            if (!diff) continue;
            if (!any) ++version_, any = true;
            for (int r = 0; r < h; ++r) std::memcpy(codes_.ptr<uint8_t>(y0 + r) + x0, now.ptr<uint8_t>(y0 + r) + x0, size_t(w));
            changed_[size_t(by * bw_ + bx)] = version_;
        }
    return version_;
}

SubmapPatch SlabExporter::Export(uint64_t since, uint32_t epoch, int64_t t_ns) const {
    SubmapPatch p;
    p.sender = sender_, p.epoch = epoch, p.version = version_, p.t_ns = t_ns;
    for (int by = 0; by < bh_; ++by)
        for (int bx = 0; bx < bw_; ++bx) {
            if (changed_[size_t(by * bw_ + bx)] <= since) continue;
            SlabBlock b;
            b.bx = bx, b.by = by;
            b.cells.assign(size_t(kBlock * kBlock), kCellUnknown);   // cells past the grid's edge stay unknown
            for (int r = 0; r < kBlock && by * kBlock + r < g_.rows; ++r)
                for (int c = 0; c < kBlock && bx * kBlock + c < g_.cols; ++c)
                    b.cells[size_t(r * kBlock + c)] = codes_.at<uint8_t>(by * kBlock + r, bx * kBlock + c);
            p.blocks.push_back(std::move(b));
        }
    return p;
}

ReceivedLayers::ReceivedLayers(int self, int n_agents, const GridFrame& g)
    : self_(self), g_(g), acked_(size_t(n_agents), 0), known_(g.rows, g.cols, CV_8U, cv::Scalar(0)) {
    for (int k = 0; k < n_agents; ++k) layer_.emplace_back(g.rows, g.cols, CV_8U, cv::Scalar(kCellUnknown));
}

void ReceivedLayers::Integrate(const SubmapPatch& p, const Rigid2& T) {
    if (p.sender == self_) throw std::invalid_argument("ReceivedLayers: an agent does not receive its own patch");
    cv::Mat& L = layer_.at(size_t(p.sender));
    const double cs = std::cos(T.yaw), sn = std::sin(T.yaw);
    for (const SlabBlock& b : p.blocks) {
        if (b.cells.size() != size_t(kBlock * kBlock)) throw std::invalid_argument("ReceivedLayers: bad block");
        for (int r = 0; r < kBlock; ++r)
            for (int c = 0; c < kBlock; ++c) {
                int rr = b.by * kBlock + r, cc = b.bx * kBlock + c;
                if (!T.Identity()) {   // nearest cell of the cell centre in G [DESIGN: small corrections, no resampling]
                    const double x = g_.ox + (cc + 0.5) * g_.cell, y = g_.oy + (rr + 0.5) * g_.cell;
                    const double gx = cs * x - sn * y + T.x, gy = sn * x + cs * y + T.y;
                    cc = int(std::floor((gx - g_.ox) / g_.cell)), rr = int(std::floor((gy - g_.oy) / g_.cell));
                }
                if (rr < 0 || cc < 0 || rr >= g_.rows || cc >= g_.cols) continue;
                L.at<uint8_t>(rr, cc) = b.cells[size_t(r * kBlock + c)];
                uint8_t k = 0;
                for (size_t j = 0; j < layer_.size() && !k; ++j) k = layer_[j].at<uint8_t>(rr, cc) != kCellUnknown;
                known_.at<uint8_t>(rr, cc) = k;
            }
    }
    acked_.at(size_t(p.sender)) = std::max(acked_.at(size_t(p.sender)), p.version);
}

cv::Mat ReceivedLayers::Merged(const cv::Mat* own) const {
    cv::Mat out(g_.rows, g_.cols, CV_8U, cv::Scalar(kCellUnknown));
    // Precedence: occupied > free without the responder bit > free with it > unknown (the bit is an AND).
    static constexpr uint8_t kRank[4] = {0, 3, 2, 1};   // indexed by CellCode
    auto Fold = [&](const cv::Mat& L) {
        for (int r = 0; r < g_.rows; ++r) {
            const uint8_t* l = L.ptr<uint8_t>(r);
            uint8_t* o = out.ptr<uint8_t>(r);
            for (int c = 0; c < g_.cols; ++c)
                if (kRank[l[c] & 3] > kRank[o[c] & 3]) o[c] = l[c];
        }
    };
    if (own) Fold(*own);
    for (const auto& L : layer_) Fold(L);
    return out;
}

OperatorEgress::OperatorEgress(const GridFrame& g, const Cell& target, std::vector<Cell> exits, double rho_responder,
                               const EgressWeights& w)
    : g_(g), rho_(rho_responder), w_(w), planner_(target, std::move(exits), g.cell),
      ones_(g.rows, g.cols, CV_32F, cv::Scalar(1)) {
    w_.w_u = 0, w_.w_r = 0;   // support and risk are not on the wire
}

EgressReplan OperatorEgress::Replan(const cv::Mat& merged) {
    if (merged.type() != CV_8U || merged.rows != g_.rows || merged.cols != g_.cols)
        throw std::invalid_argument("OperatorEgress: CellCodes on the briefing grid");
    cv::Mat blocked(merged.size(), CV_8U);
    for (int r = 0; r < merged.rows; ++r) {
        const uint8_t* m = merged.ptr<uint8_t>(r);
        uint8_t* b = blocked.ptr<uint8_t>(r);
        for (int c = 0; c < merged.cols; ++c) b[c] = m[c] != kCellFree && m[c] != kCellFreeResponder;
    }
    const cv::Mat c64 = EdgeClearance(blocked, g_.cell, std::max(rho_, w_.rho_safe));
    c64.convertTo(clear_, CV_32F);
    trav_r_.create(merged.size(), CV_8U);
    for (int r = 0; r < merged.rows; ++r) {
        const uint8_t* m = merged.ptr<uint8_t>(r);
        const double* d = c64.ptr<double>(r);
        uint8_t* t = trav_r_.ptr<uint8_t>(r);
        for (int c = 0; c < merged.cols; ++c) t[c] = m[c] == kCellFreeResponder && d[c] >= rho_ - kTravEpsM;
    }
    EgressReplan out = planner_.Replan(trav_r_, EgressPenalty(clear_, ones_, cv::Mat(), w_), ones_, cv::Mat());
    out.confidence = -1;   // not computed: no support on the wire
    return out;
}

TaskBid BuildBid(int sender, uint32_t epoch, uint64_t seq, int64_t t_ns, const NbvPlan& nbv, int max_options,
                 const Rigid2& T) {
    TaskBid b;
    b.sender = sender, b.epoch = epoch, b.seq = seq, b.t_ns = t_ns, b.lambda_path = float(nbv.lambda_path);
    const double cs = std::cos(T.yaw), sn = std::sin(T.yaw);
    for (const TaskCandidate& o : nbv.options) {   // sorted by utility: the first view of a task is its best
        if (int(b.options.size()) >= max_options) break;
        if (o.source == TaskSource::kReturn || o.source == TaskSource::kHold) continue;
        BidOption x;
        x.task_key = o.task_id;
        x.goal_x = cs * o.goal_xy.first - sn * o.goal_xy.second + T.x;
        x.goal_y = sn * o.goal_xy.first + cs * o.goal_xy.second + T.y;
        for (int k = 0; k < 5; ++k) x.ig_raw[k] = float(o.ig_raw[size_t(k)]);
        x.path_m = float(o.path_m), x.cost_bandwidth = float(o.cost[1]), x.cost_risk = float(o.cost[4]);
        x.risk = float(o.risk), x.source = uint8_t(o.source);
        x.footprint = o.footprint;
        b.options.push_back(x);
    }
    return b;
}

namespace {

int64_t BlockKey(const BlockGain& b) { return (int64_t(b.by) << 32) | uint32_t(uint16_t(b.bx)); }

double Marginal(const BidOption& o, const std::map<int64_t, int>& residual) {   // in voxels
    double d = 0;
    for (const BlockGain& b : o.footprint) {
        const auto it = residual.find(BlockKey(b));
        d += std::max(0, int(b.w) - (it == residual.end() ? 0 : it->second));
    }
    return d * kFootprintUnit;
}

// The greedy rounds of the discount (R2) and overlap (R1) modes over the members' bids (P: the common scales).
void RoundsAssign(const std::vector<const TaskBid*>& by, const ComponentInfo& comp, const std::array<double, 8>& P,
                  const BrokerParams& prm, TaskConsensus* out, BrokerLog* log) {
    const size_t A = comp.members.size();
    struct O {
        const BidOption* o;
        double other;   // the D114 terms but coverage, net of bandwidth and risk, on the common scales
        double cov;     // the coverage term on its scale
        double disc;    // exp(-rate x path)
    };
    std::vector<std::vector<O>> opts(A);
    for (size_t a = 0; a < A; ++a) {
        if (!by[a]) continue;
        const double rate = P[5] > 0 ? by[a]->lambda_path / P[5] : 0.0;
        for (const BidOption& o : by[a]->options) {
            double other = 0;
            for (int k = 1; k < 5; ++k) other += P[size_t(k)] > 0 ? o.ig_raw[k] / P[size_t(k)] : 0.0;
            other -= (P[6] > 0 ? o.cost_bandwidth / P[6] : 0.0) + (P[7] > 0 ? o.cost_risk / P[7] : 0.0);
            opts[a].push_back({&o, other, P[0] > 0 ? o.ig_raw[0] / P[0] : 0.0, std::exp(-rate * o.path_m)});
        }
    }
    std::map<int64_t, int> residual;   // per block: max w over the assigned options
    std::vector<std::pair<double, double>> goals;
    std::vector<bool> done(A, false);
    double f = 0;
    auto Value = [&](const O& x) {
        if (prm.mode == BrokerMode::kOverlap) {
            const double cov = P[0] > 0 ? Marginal(*x.o, residual) / P[0] : 0.0;
            return std::max(0.0, cov + x.other);
        }
        double v = std::max(0.0, x.cov + x.other);   // discount: x (1 - P(d)) per assigned goal
        for (const auto& [gx, gy] : goals) v *= std::min(1.0, std::hypot(x.o->goal_x - gx, x.o->goal_y - gy) / prm.range_m);
        return v;
    };
    auto Feasible = [&](const O& x) {
        for (const auto& [gx, gy] : goals)
            if (std::hypot(x.o->goal_x - gx, x.o->goal_y - gy) < prm.r_same) return false;
        return true;
    };
    for (;;) {
        size_t ba = A, bi = 0;
        double best = -1;
        for (size_t a = 0; a < A; ++a) {
            if (done[a]) continue;
            for (size_t i = 0; i < opts[a].size(); ++i) {
                if (!Feasible(opts[a][i])) continue;
                const double s = Value(opts[a][i]) * opts[a][i].disc;
                if (s > best) best = s, ba = a, bi = i;   // strict: ties to member order, then the bid's order
            }
        }
        if (ba == A) break;
        const BidOption& o = *opts[ba][bi].o;
        const double df = Marginal(o, residual);
        for (const BlockGain& b : o.footprint) {
            int& r = residual[BlockKey(b)];
            r = std::max(r, int(b.w));
        }
        f += df;
        goals.push_back({o.goal_x, o.goal_y});
        done[ba] = true;
        out->assignments.push_back({comp.members[ba], o.task_key, o.goal_x, o.goal_y});
        if (log) log->rounds.push_back({comp.members[ba], o.task_key, df, best, f});
    }
    if (log) {
        log->f = f, log->upper_bound = f;
        for (size_t a = 0; a < A; ++a) {   // partition matroid: OPT <= f(S) + sum over parts of the best marginal
            double m = 0;
            for (const O& x : opts[a]) m = std::max(m, Marginal(*x.o, residual));
            log->upper_bound += m;
        }
    }
}

}  // namespace

TaskConsensus BrokerAssign(const std::vector<TaskBid>& bids, const ComponentInfo& comp, const BrokerParams& prm,
                           int64_t t_ns, uint64_t seq, BrokerLog* log) {
    const double r_same = prm.r_same;
    TaskConsensus out;
    out.broker = comp.broker, out.epoch = comp.epoch, out.seq = seq, out.t_ns = t_ns, out.members = comp.members;
    // One bid per member, the newest; the auction's agent index = the member's position in comp.members.
    std::vector<const TaskBid*> by(comp.members.size(), nullptr);
    for (const TaskBid& b : bids) {
        if (b.epoch != comp.epoch) continue;
        const auto it = std::find(comp.members.begin(), comp.members.end(), b.sender);
        if (it == comp.members.end()) continue;
        const TaskBid*& slot = by[size_t(it - comp.members.begin())];
        if (!slot || b.t_ns > slot->t_ns || (b.t_ns == slot->t_ns && b.seq > slot->seq)) slot = &b;
    }
    for (const TaskBid* b : by) if (b) out.bids_t_ns = std::max(out.bids_t_ns, b->t_ns);
    // Common scales (BuildAuction's rule): per term the max over bidders of that bidder's p90.
    std::array<double, 8> P{};
    for (const TaskBid* b : by) {
        if (!b) continue;
        for (int k = 0; k < 8; ++k) {
            std::vector<double> v;
            for (const BidOption& o : b->options)
                v.push_back(k < 5 ? o.ig_raw[k] : (k == 5 ? o.path_m : (k == 6 ? o.cost_bandwidth : o.cost_risk)));
            P[size_t(k)] = std::max(P[size_t(k)], P90Scale(v));
        }
    }
    const size_t A = comp.members.size();
    if (prm.mode != BrokerMode::kPlain) {
        RoundsAssign(by, comp, P, prm, &out, log);
        return out;
    }
    // Tasks: greedy spatial clustering in bid order (member order, then each bid's utility order).
    std::vector<std::pair<double, double>> task_goal;
    std::vector<std::vector<int>> task_of(A);
    for (size_t a = 0; a < A; ++a) {
        if (!by[a]) continue;
        for (const BidOption& o : by[a]->options) {
            int j = -1;
            for (size_t t = 0; t < task_goal.size() && j < 0; ++t)
                if (std::hypot(o.goal_x - task_goal[t].first, o.goal_y - task_goal[t].second) < r_same) j = int(t);
            if (j < 0) j = int(task_goal.size()), task_goal.push_back({o.goal_x, o.goal_y});
            task_of[a].push_back(j);
        }
    }
    AuctionInstance in;
    in.n_agents = int(A), in.max_bundle = 1;
    for (size_t t = 0; t < task_goal.size(); ++t) in.task_ids.push_back(int64_t(t));
    const size_t T = task_goal.size();
    in.value.assign(A, std::vector<double>(T, 0.0));
    in.d0.assign(A, std::vector<double>(T, 0.0));
    in.leg.assign(A, std::vector<std::vector<double>>(T, std::vector<double>(T, 0.0)));
    std::vector<std::vector<int>> view(A, std::vector<int>(T, -1));
    for (size_t a = 0; a < A; ++a) {
        in.rate.push_back(by[a] && P[5] > 0 ? by[a]->lambda_path / P[5] : 0.0);
        if (!by[a]) continue;
        for (size_t i = 0; i < by[a]->options.size(); ++i) {
            const size_t j = size_t(task_of[a][i]);
            if (view[a][j] >= 0) continue;   // the first (best) of this member's options in the task
            const BidOption& o = by[a]->options[i];
            double g = 0;
            for (int k = 0; k < 5; ++k) g += P[size_t(k)] > 0 ? o.ig_raw[k] / P[size_t(k)] : 0.0;
            const double cb = P[6] > 0 ? o.cost_bandwidth / P[6] : 0.0, cr = P[7] > 0 ? o.cost_risk / P[7] : 0.0;
            in.value[a][j] = std::max(0.0, g - cb - cr);
            in.d0[a][j] = o.path_m;
            view[a][j] = int(i);
        }
    }
    const Allocation al = SequentialGreedy(in);
    for (size_t a = 0; a < A; ++a) {
        if (!by[a] || al.agents[a].path.empty()) continue;
        const BidOption& o = by[a]->options[size_t(view[a][size_t(al.agents[a].path.front())])];
        out.assignments.push_back({comp.members[a], o.task_key, o.goal_x, o.goal_y});
    }
    return out;
}

Pick ApplyConsensus(const TaskConsensus* c, int self, uint32_t epoch, int64_t now_ns, double timeout_s,
                    const std::vector<TaskCandidate>& options, double r_same) {
    Pick p;
    auto Own = [&](const std::string& why) {
        p.option = options.empty() ? -1 : 0, p.coordinated = false, p.why = "uncoordinated: " + why;
        return p;
    };
    if (!c) return Own("no consensus received");
    if (c->epoch != epoch) return Own("consensus from another epoch");
    if (double(now_ns - c->t_ns) * 1e-9 > timeout_s) return Own("consensus older than the timeout");
    const Assignment* mine = nullptr;
    for (const Assignment& a : c->assignments)
        if (a.agent == self) mine = &a;
    if (!mine) return Own("no assignment for this agent");
    for (size_t i = 0; i < options.size(); ++i)
        if (options[i].task_id == mine->task_key) {
            p.option = int(i), p.coordinated = true, p.why = "coordinated";
            return p;
        }
    for (size_t i = 0; i < options.size(); ++i)   // the task re-keyed since the bid (map changed): the nearest goal
        if (std::hypot(options[i].goal_xy.first - mine->goal_x, options[i].goal_xy.second - mine->goal_y) < r_same) {
            p.option = int(i), p.coordinated = true, p.why = "coordinated";
            return p;
        }
    return Own("the assigned task is gone");
}

Pick ApplyConsensusDiscounted(const TaskConsensus* c, int self, uint32_t epoch, int64_t now_ns, double timeout_s,
                              const std::vector<TaskCandidate>& options, double r_same, double range_m, double tau_s) {
    Pick p = ApplyConsensus(c, self, epoch, now_ns, timeout_s, options, r_same);
    if (p.coordinated || !c || options.empty()) return p;
    const double age = std::max(0.0, double(now_ns - c->t_ns) * 1e-9), keep = std::exp(-age / tau_s);
    double best = -1e300;
    for (size_t i = 0; i < options.size(); ++i) {
        double u = options[i].utility;
        if (u > 0)
            for (const Assignment& a : c->assignments) {
                if (a.agent == self) continue;
                const double d = std::hypot(options[i].goal_xy.first - a.goal_x, options[i].goal_xy.second - a.goal_y);
                u *= 1.0 - std::max(0.0, 1.0 - d / range_m) * keep;
            }
        if (u > best) best = u, p.option = int(i);
    }
    char buf[96];
    std::snprintf(buf, sizeof buf, " (peers' claims discounted: age %.1f s, tau %.0f s)", age, tau_s);
    p.why += buf;
    return p;
}

}  // namespace ariadne::planning::multi
