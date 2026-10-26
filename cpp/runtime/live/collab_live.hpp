#pragma once
// ---------------------------------------------------------------------------
// The parent's side of the live collaboration, ASYNCHRONOUS (design LIVE_COLLAB §9, D179; replaces D174's lockstep).
//
// One fleet thread owns the CollabFleet and its radio nodes (one per participant, the operator included). Every call the
// runtime makes is non-blocking:
//   Step(t)            enqueue sim tick t (the runtime's 0.5 s ladder)
//   Send(k, fn, t)     enqueue a D175 send: fn(node k, t) runs on the fleet thread, in call order
//   CutComms / RestoreComms / InjectFalse   enqueue the panel actions (LIVE_COLLAB §5)
//   Drain(k, T)        agent k's coordination deliveries produced so far (ticks in order, FIFO per sender)
//   Snapshot(T)        the latest published {T_O0_O, Linked, per-agent Component/LastHeartbeat/AckedPatchVersion,
//                      Telemetry, version}: ONE atomically swapped value
// The fleet thread processes the queue in order. At each tick t it commits what the collab workers sent since the last
// tick (#ckf records are released at t, #fresp replies become answerable at t), delivers the network up to t, ticks
// every node and publishes a snapshot. Feature hooks never block: a reply not yet committed is "not ready" (CollabNode's
// deferred path). Queue order is the runtime's call order (deterministic); only worker arrival timing is not, so every
// such choice is logged with its tick: commits, drains, snapshot reads (collab_events.jsonl). Replay (params.replay_log)
// runs the same code synchronously and serves exactly the logged choices (waiting for the workers if needed - replay
// wall time only), so replays equal the live run they log.
// GROUND TRUTH IS EVALUATION ONLY: the records' gt feeds edges.csv and injections, never a hook's decision.
// ---------------------------------------------------------------------------
#include "runtime/agent/collab_fleet.hpp"
#include "runtime/live/collab_link.hpp"
#include "runtime/live/link_frame.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace ariadne::runtime::live {

struct CollabLiveParams {
    std::string log_path;            // live: write collab_events.jsonl here (empty = no log, not replayable)
    std::string replay_log;          // replay: serve this log's choices (synchronous); empty = live
    double wait_timeout_s = 120.0;   // replay: a worker silent this long (wall) is a hard error
    // What decides the fleet's behaviour besides the inputs: e.g. a hash of the collab config and the build. Live
    // writes it as the log's first line; replay refuses a log written under a different one (a replay under a
    // changed config cannot reproduce the run: collab60d's replay read a config that had since enabled D178's gate).
    // Empty = unchecked (logs from before this field).
    std::string identity;
    // D174 lockstep fields, IGNORED since D179 (kept so existing call sites compile until rewired).
    bool lockstep = false;
    double feature_latency_s = 0.0;
};

/// What agent k's radio node knew at the snapshot's tick: the D175 coordination reads (the fork-join must never touch
/// a node the fleet thread owns). Same values as the node accessors of the same name, frozen at publish.
struct CollabAgentView {
    ac::CollabNode::ComponentView component;                // Component(): broker, members, accepted epoch
    std::map<ac::AgentId, core::HeartbeatValue> heartbeat;  // LastHeartbeat(j), only peers ever heard
    std::map<ac::AgentId, uint64_t> acked_patch;            // AckedPatchVersion(j) for every other fleet member
};

struct CollabSnapshot {
    uint64_t version = 0;
    int64_t t_ns = 0;
    std::map<ac::AgentId, core::SE3> T_O0_O;   // agents merged with agent 0: T_G_W(0)^-1 T_G_W(k)
    std::map<ac::AgentId, bool> linked;        // in agent 0's connected component
    std::map<ac::AgentId, CollabAgentView> agent;   // per fleet member (D175 reads)
    std::map<core::MessageClass, uint64_t> wire_bytes;
    std::string telemetry;                     // one JSON line for the console's comms panel

    // Called only by the fleet owner before publication. Mission readers must
    // never sample its mutable ledger while asynchronous ticks are running.
    void CaptureBytes(const core::ByteLedger& ledger) {
        for (auto cls : {core::MessageClass::SUBMAP_PATCH, core::MessageClass::TASK_BID,
                         core::MessageClass::TASK_CONSENSUS, core::MessageClass::HEARTBEAT})
            wire_bytes[cls] = ledger.TotalForClass(cls);
    }
    uint64_t Bytes(core::MessageClass cls) const {
        const auto it = wire_bytes.find(cls);
        return it == wire_bytes.end() ? 0 : it->second;
    }
};

class CollabLive {
public:
    using NodeFn = std::function<void(ac::CollabNode&, core::Timestamp)>;

    CollabLive(const CollabConfig& cfg, const std::vector<ac::AgentId>& fleet, ac::CommsNode& comms,
               const perception::HardPairMatcher& lightglue, uint64_t seed, int64_t t_first_ns, CollabLiveParams p,
               std::ostream* edges_csv, std::function<void(ac::AgentId, ac::NodeHooks&)> customize = {})
        : p_(p), fleet_ids_(fleet), enc_(ac::ParseLocalEncoding(cfg.local_encoding)), comms_(comms),
          replay_(!p.replay_log.empty()),
          fleet_(cfg, fleet, comms, lightglue, seed, t_first_ns,
                 [this](ac::AgentId k, uint64_t kf) { return Features(k, kf); },
                 [this](const ac::KeyframeRef& r) { return GtCamera(r); }, edges_csv,
                 [this, customize](ac::AgentId k, ac::NodeHooks& h) {
                     if (customize) customize(k, h);
                     // Deliveries go to the fleet-thread outbox, drained by the runtime (never its own containers).
                     h.coordination = [this, k](const core::DecodedMessage& d, core::Timestamp t) {
                         std::lock_guard<std::mutex> lk(out_mu_);
                         outbox_[k].push_back({t.ns, d});
                     };
                 }) {
        if (replay_) LoadLog(p_.replay_log);
        else if (!p_.log_path.empty()) {
            payload_dir_ = std::filesystem::path(p_.log_path).parent_path() / "collab_payloads";
            std::filesystem::create_directories(payload_dir_);
            log_.open(p_.log_path);
            if (!p_.identity.empty()) Log("{\"e\":\"identity\",\"v\":\"" + p_.identity + "\"}");
        }
        snap_ = std::make_shared<const CollabSnapshot>();
        if (replay_) history_[0] = snap_;
        else thread_ = std::thread([this] { Run(); });
    }
    ~CollabLive() { Shutdown(); }
    CollabLive(const CollabLive&) = delete;
    CollabLive& operator=(const CollabLive&) = delete;

    /// fd connected to agent k's collab worker; a reader thread files its #ckf and #fresp.
    void Attach(ac::AgentId k, int fd) {
        std::lock_guard<std::mutex> lk(in_mu_);
        Peer& peer = peers_[k];
        peer.fd = fd;
        SendFrame(k, "#hello", EncodeHello({"parent", k}));
        peer.reader = std::thread([this, k, fd] { Read(k, fd); });
    }

    // --- the runtime's calls: all non-blocking live ---
    void Step(core::Timestamp t) { Push({Item::kTick, t, 0, {}}); }
    void Send(ac::AgentId k, NodeFn fn, core::Timestamp t) { Push({Item::kNode, t, k, std::move(fn)}); }
    void CutComms(core::Timestamp t, std::vector<std::vector<ac::AgentId>> groups) {
        Push({Item::kFleet, t, 0, [this, g = std::move(groups)](ac::CollabNode&, core::Timestamp tt) { comms_.Cut(tt, g); }});
    }
    void RestoreComms(core::Timestamp t) {
        Push({Item::kFleet, t, 0, [this](ac::CollabNode&, core::Timestamp tt) { comms_.Restore(tt); }});
    }
    /// D178 known entry poses, applied on the fleet thread in call order (before the first Step). Never call
    /// fleet().SetEntryPoses from the runtime thread while the fleet thread runs: that is a data race.
    void SetEntryPoses(std::map<ac::AgentId, core::SE3> entry) {
        Push({Item::kFleet, core::Timestamp{}, 0,
              [this, e = std::move(entry)](ac::CollabNode&, core::Timestamp) { fleet_.SetEntryPoses(e); }});
    }
    void InjectFalse(core::Timestamp t) {
        Push({Item::kFleet, t, 0,
              [this](ac::CollabNode&, core::Timestamp tt) { fleet_.InjectNow(tt, core::Vec3(4.0, 0.0, 0.0), 45.0); }});
    }

    /// Agent k's coordination deliveries produced so far (live) / exactly the logged ones (replay).
    std::vector<core::DecodedMessage> Drain(ac::AgentId k, int64_t T) {
        std::vector<core::DecodedMessage> out;
        std::lock_guard<std::mutex> lk(out_mu_);
        auto& box = outbox_[k];
        if (!replay_) {
            std::ostringstream keys;
            for (size_t i = 0; i < box.size(); ++i) keys << (i ? "," : "") << "\"" << Key(box[i].second) << "\"";
            Log("{\"e\":\"drain\",\"T\":" + std::to_string(T) + ",\"agent\":" + std::to_string(k) + ",\"keys\":[" +
                keys.str() + "]}");
            for (auto& [t, d] : box) out.push_back(std::move(d));
            box.clear();
            return out;
        }
        const auto want = drains_.find({T, k});
        if (want == drains_.end()) return out;
        for (const std::string& key : want->second) {
            const auto it = std::find_if(box.begin(), box.end(), [&](const auto& x) { return Key(x.second) == key; });
            if (it == box.end()) throw std::runtime_error("collab replay: a logged delivery was never produced: " + key);
            out.push_back(std::move(it->second));
            box.erase(it);
        }
        return out;
    }

    /// The published snapshot (live: the latest; replay: the version the live run read at T).
    std::shared_ptr<const CollabSnapshot> Snapshot(int64_t T) {
        if (!replay_) {
            auto s = std::atomic_load(&snap_);
            Log("{\"e\":\"snap\",\"T\":" + std::to_string(T) + ",\"v\":" + std::to_string(s->version) + "}");
            return s;
        }
        const auto v = snaps_read_.find(T);
        const auto h = history_.find(v == snaps_read_.end() ? 0 : v->second);
        if (h == history_.end())
            throw std::runtime_error("collab replay: snapshot version " + std::to_string(v->second) + " read at T " +
                                     std::to_string(T) + " was never published (latest " + std::to_string(version_) + ")");
        return h->second;
    }

    // D174 call-site compatibility until the runtime reads one Snapshot per barrier (B's invariant 2 needs that).
    [[deprecated("D179: read Snapshot(T) once per barrier")]] std::map<ac::AgentId, core::SE3> T_O0_O() {
        return Snapshot(-1)->T_O0_O;
    }
    [[deprecated("D179: read Snapshot(T) once per barrier")]] std::map<ac::AgentId, bool> Linked() { return Snapshot(-1)->linked; }
    [[deprecated("D179: read Snapshot(T) once per barrier")]] std::string Telemetry(core::Timestamp) {
        return Snapshot(-1)->telemetry;
    }

    CollabFleet& fleet() { return fleet_; }   // fleet thread, or after Shutdown

    void Shutdown() {
        {
            std::lock_guard<std::mutex> lk(q_mu_);
            if (stopping_) return;
            stopping_ = true;
        }
        q_cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        {
            std::lock_guard<std::mutex> lk(in_mu_);
            for (auto& [k, peer] : peers_)
                if (peer.fd >= 0) ::shutdown(peer.fd, SHUT_RDWR);
        }
        in_cv_.notify_all();
        for (auto& [k, peer] : peers_) {
            if (peer.reader.joinable()) peer.reader.join();
            if (peer.fd >= 0) ::close(peer.fd), peer.fd = -1;
        }
        if (log_.is_open()) log_.flush();
    }

private:
    struct Item {
        enum Kind { kTick, kNode, kFleet } kind = kTick;
        core::Timestamp t;
        ac::AgentId k = 0;
        NodeFn fn;
    };
    struct Peer {
        int fd = -1;
        std::thread reader;
        std::deque<std::pair<CollabStoreRecord, uint32_t>> records;        // arrived, not yet committed
        std::map<uint64_t, std::optional<CollabLocalFeatures>> replies;   // arrived, not yet committed
        std::map<uint64_t, std::vector<uint8_t>> record_wire, reply_wire;
        bool closed = false;
        std::string error;                                                // why the reader stopped, if it threw
        std::chrono::steady_clock::time_point last_frame{};               // replay: the worker's last sign of life
    };
    using Commit = std::tuple<ac::AgentId, std::string, uint64_t>;   // agent, ckf|fresp, keyframe

    void Push(Item it) {
        if (replay_) return Process(it);
        {
            std::lock_guard<std::mutex> lk(q_mu_);
            q_.push_back(std::move(it));
        }
        q_cv_.notify_one();
    }

    void Run() {
        for (;;) {
            Item it;
            {
                std::unique_lock<std::mutex> lk(q_mu_);
                q_cv_.wait(lk, [&] { return stopping_ || !q_.empty(); });
                if (q_.empty()) return;   // stopping, drained
                it = std::move(q_.front());
                q_.pop_front();
            }
            Process(it);
        }
    }

    void Process(Item& it) {
        if (it.kind == Item::kNode) return it.fn(fleet_.node(it.k), it.t);
        if (it.kind == Item::kFleet) return it.fn(fleet_.node(fleet_ids_.front()), it.t);
        tick_ns_ = it.t.ns;   // the sim time a #freq asked at this tick carries (worker's gone rule)
        for (const Commit& c : TakeArrivals(it.t.ns)) Apply(c, it.t);
        fleet_.Deliver(it.t);
        fleet_.Tick(it.t);
        Publish(it.t);
    }

    /// Live: everything the readers filed since the last tick (agent order, then arrival order), logged with payload.
    /// Replay: apply exactly the logged payloads for this tick; legacy logs fall back to worker reproduction.
    std::vector<Commit> TakeArrivals(int64_t t_ns) {
        std::vector<Commit> out;
        std::unique_lock<std::mutex> lk(in_mu_);
        if (!replay_) {
            for (auto& [k, peer] : peers_) {
                for (auto& [r, n] : peer.records) {
                    out.emplace_back(k, "ckf", r.id);
                    staged_records_[{k, r.id}] = {std::move(r), n};
                    auto wire = peer.record_wire.find(std::get<2>(out.back()));
                    if (wire == peer.record_wire.end()) throw std::logic_error("collab log: missing #ckf payload");
                    LogCommit(t_ns, k, "ckf", std::get<2>(out.back()), wire->second);
                }
                peer.records.clear();
                peer.record_wire.clear();
                for (auto& [kf, f] : peer.replies) {
                    out.emplace_back(k, "fresp", kf);
                    staged_replies_[{k, kf}] = std::move(f);
                    auto wire = peer.reply_wire.find(kf);
                    if (wire == peer.reply_wire.end()) throw std::logic_error("collab log: missing #fresp payload");
                    LogCommit(t_ns, k, "fresp", kf, wire->second);
                }
                peer.replies.clear();
                peer.reply_wire.clear();
            }
            return out;
        }
        const auto want = commits_.find(t_ns);
        if (want == commits_.end()) return out;
        for (const Commit& c : want->second) {
            const auto& [k, kind, kf] = c;
            const auto payload = replay_payloads_.find({t_ns, k, kind, kf});
            if (payload != replay_payloads_.end()) {
                if (kind == "ckf") {
                    uint32_t n = 0;
                    CollabStoreRecord r = DecodeRecord(payload->second, &n);
                    if (r.id != kf) throw std::runtime_error("collab replay: #ckf payload ID mismatch");
                    staged_records_[{k, kf}] = {std::move(r), n};
                } else if (kind == "fresp") {
                    CollabFeatureReply r = DecodeFeatureReply(payload->second);
                    if (r.kf != kf) throw std::runtime_error("collab replay: #fresp payload ID mismatch");
                    staged_replies_[{k, kf}] = std::move(r.features);
                } else {
                    throw std::runtime_error("collab replay: unknown commit kind " + kind);
                }
                out.push_back(c);
                continue;
            }
            const auto has = [&] {
                Peer& peer = peers_[k];
                if (kind == "ckf")
                    return std::any_of(peer.records.begin(), peer.records.end(), [&](const auto& x) { return x.first.id == kf; });
                return peer.replies.count(kf) > 0;
            };
            // The timeout is worker SILENCE, not total wait: a replay worker may be busy far longer than live (its
            // estimator is not paced by the sim, so keyframes queue ahead of #freq), and that is not an error.
            const auto t0 = std::chrono::steady_clock::now();
            const auto silent_for = [&] {
                const auto last = std::max(t0, peers_[k].last_frame);
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count();
            };
            while (!has()) {
                Peer& peer = peers_[k];
                const auto why = [&](const std::string& what) {
                    std::ostringstream o;
                    o << "collab replay: logged arrival agent " << k << " " << kind << " kf " << kf << " at t_ns "
                      << t_ns << " " << what << " after "
                      << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count()
                      << " s (worker holds " << peer.records.size() << " records, " << peer.replies.size()
                      << " replies" << (peer.error.empty() ? "" : "; reader error: " + peer.error) << ")";
                    return o.str();
                };
                if (peer.closed) throw std::runtime_error(why("never came: the worker closed its socket"));
                if (silent_for() > p_.wait_timeout_s) throw std::runtime_error(why("never came: the worker went silent"));
                in_cv_.wait_for(lk, std::chrono::seconds(1));
            }
            Peer& peer = peers_[k];
            if (kind == "ckf") {
                auto rit = std::find_if(peer.records.begin(), peer.records.end(), [&](const auto& x) { return x.first.id == kf; });
                staged_records_[{k, kf}] = std::move(*rit);
                peer.records.erase(rit);
            } else {
                staged_replies_[{k, kf}] = std::move(peer.replies.at(kf));
                peer.replies.erase(kf);
            }
            out.push_back(c);
        }
        return out;
    }

    void Apply(const Commit& c, core::Timestamp t) {
        const auto& [k, kind, kf] = c;
        const auto key = std::make_pair(k, kf);
        if (kind == "ckf") {
            auto [r, n] = std::move(staged_records_.at(key));
            staged_records_.erase(key);
            gt_[key] = r.has_gt ? std::optional<core::SE3>(r.gt_T_W_B * r.T_B_C) : std::nullopt;
            camera_[key] = r.camera;
            fleet_.Release(k, r, t, n);
        } else {
            ready_[key] = std::move(staged_replies_.at(key));
            staged_replies_.erase(key);
        }
    }

    /// The FeatureSource (fleet thread): a committed reply, else ask once and answer "not ready".
    std::optional<core::FeatureResponseValue> Features(ac::AgentId k, uint64_t kf) {
        const auto key = std::make_pair(k, kf);
        const auto it = ready_.find(key);
        if (it == ready_.end()) {
            if ((!replay_ || !replay_payloads_complete_) && asked_.insert(key).second) {
                std::lock_guard<std::mutex> lk(in_mu_);
                if (peers_.count(k)) SendFrame(k, "#freq", EncodeFeatureRequest(kf, tick_ns_));
            }
            return std::nullopt;
        }
        std::optional<CollabLocalFeatures> f = std::move(it->second);
        ready_.erase(it);
        asked_.erase(key);
        const auto cam = camera_.count(key) ? camera_.at(key) : std::array<float, 5>{};
        if (!f) return CollabFleet::Response(kf, {}, {}, {}, cam, enc_);   // gone: the verifier rejects it
        return CollabFleet::Response(kf, f->keypoints, f->descriptors, f->points3d, cam, enc_);
    }

    std::optional<core::SE3> GtCamera(const ac::KeyframeRef& r) const {
        const auto it = gt_.find({r.agent, r.keyframe});
        return it == gt_.end() ? std::nullopt : it->second;
    }

    void Publish(core::Timestamp t) {
        auto s = std::make_shared<CollabSnapshot>();
        s->version = ++version_;
        s->t_ns = t.ns;
        s->CaptureBytes(comms_.ledger());
        const auto& n0 = fleet_.node(fleet_ids_.front());
        if (const auto T0 = n0.T_G_W())
            for (ac::AgentId k : fleet_ids_) {
                const auto& n = fleet_.node(k);
                if (const auto T = n.T_G_W(); T && n.T_G_W_anchor() == n0.T_G_W_anchor()) s->T_O0_O[k] = T0->inverse() * *T;
            }
        const auto members = n0.Component().members;
        for (ac::AgentId k : fleet_ids_) s->linked[k] = std::find(members.begin(), members.end(), k) != members.end();
        for (ac::AgentId k : fleet_ids_) {
            auto& n = fleet_.node(k);
            CollabAgentView& v = s->agent[k];
            v.component = n.Component();
            for (ac::AgentId j : fleet_ids_) {
                if (j == k) continue;
                if (auto h = n.LastHeartbeat(j)) v.heartbeat.emplace(j, std::move(*h));
                v.acked_patch[j] = n.AckedPatchVersion(j);
            }
        }
        s->telemetry = BuildTelemetry(t);
        std::shared_ptr<const CollabSnapshot> c = s;
        if (replay_) history_[c->version] = c;
        std::atomic_store(&snap_, c);
    }

    std::string BuildTelemetry(core::Timestamp now) const {
        std::ostringstream o;
        o << "{\"t_ns\":" << now.ns << ",\"agents\":[";
        bool first = true;
        for (ac::AgentId k : fleet_ids_) {
            const auto& n = fleet_.node(k);
            const auto& s = n.stats();
            o << (first ? "" : ",") << "{\"agent\":" << k << ",\"broker\":" << n.Component().broker
              << ",\"merged\":" << (n.T_G_W() ? "true" : "false") << ",\"anchor\":" << n.T_G_W_anchor()
              << ",\"edges_verified\":" << s.edges_verified << ",\"solves\":" << s.solves
              << ",\"answers_deferred\":" << s.answers_deferred << "}";
            first = false;
        }
        const auto& vs = fleet_.verify_stats();
        const auto& gs = fleet_.entry_gate_stats();
        size_t kept = 0;
        for (const auto& [key, v] : fleet_.kept_edges()) kept += v.last_weight >= 0.5;
        o << "],\"verify\":{\"calls\":" << vs.calls << ",\"gated\":" << vs.gated << "},\"entry_gate\":{\"tested\":"
          << gs.tested << ",\"rejected\":" << gs.rejected << "},\"kept_inter_edges\":" << kept << ",\"cut_active\":"
          << (comms_.cut_active() ? "true" : "false") << ",\"injections\":[";
        for (size_t i = 0; i < fleet_.injections().size(); ++i) {
            const auto& st = fleet_.injections()[i];
            o << (i ? "," : "") << "{\"planted\":" << (st.planted ? "true" : "false") << ",\"kept_solves\":"
              << st.kept_solves << ",\"weight\":" << (std::isfinite(st.weight) ? st.weight : -1.0) << "}";
        }
        o << "]}";
        return o.str();
    }

    void Read(ac::AgentId k, int fd) {
        LinkFrame f;
        try {
            while (ReadFrame(fd, f)) {
                std::lock_guard<std::mutex> lk(in_mu_);
                Peer& peer = peers_.at(k);
                peer.last_frame = std::chrono::steady_clock::now();   // #cwm included: a busy worker is alive
                if (replay_ && replay_payloads_complete_) {
                    in_cv_.notify_all();  // payload-backed replay never consumes worker output
                    continue;
                }
                if (f.topic == "#ckf") {
                    uint32_t n = 0;
                    CollabStoreRecord r = DecodeRecord(f.payload, &n);
                    peer.record_wire[r.id] = std::move(f.payload);
                    peer.records.emplace_back(std::move(r), n);
                } else if (f.topic == "#fresp") {
                    CollabFeatureReply rep = DecodeFeatureReply(f.payload);
                    peer.reply_wire[rep.kf] = std::move(f.payload);
                    peer.replies[rep.kf] = std::move(rep.features);
                }   // #cwm: D174's lockstep watermark, unused asynchronously
                in_cv_.notify_all();
            }
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(in_mu_);
            peers_.at(k).error = e.what();   // a decode or socket error must reach the replay's error message
        }
        std::lock_guard<std::mutex> lk(in_mu_);
        peers_.at(k).closed = true;
        in_cv_.notify_all();
    }

    void SendFrame(ac::AgentId k, const char* topic, const std::vector<uint8_t>& b) {   // under in_mu_
        Peer& peer = peers_.at(k);
        if (!peer.closed) peer.closed = !WriteFrame(peer.fd, topic, b.data(), b.size());
    }

    static std::string Key(const core::DecodedMessage& d) {   // a delivery's identity in the log
        return std::to_string(d.sender) + ":" + std::to_string(int(d.cls)) + ":" + std::to_string(d.stamp_ns);
    }

    void Log(const std::string& line) {
        if (!log_.is_open()) return;
        std::lock_guard<std::mutex> lk(log_mu_);
        log_ << line << '\n';
    }

    void LogCommit(int64_t t_ns, ac::AgentId agent, const std::string& kind, uint64_t kf,
                   const std::vector<uint8_t>& payload) {
        if (!log_.is_open()) return;
        const std::string name = "a" + std::to_string(agent) + "_" + kind + "_kf" + std::to_string(kf) + "_t" +
                                 std::to_string(t_ns) + ".bin";
        const auto path = payload_dir_ / name;
        std::ofstream out(path, std::ios::binary);
        if (!out) throw std::runtime_error("collab log: cannot write payload " + path.string());
        out.write(reinterpret_cast<const char*>(payload.data()), std::streamsize(payload.size()));
        out.close();
        if (!out) throw std::runtime_error("collab log: failed writing payload " + path.string());
        Log("{\"e\":\"commit\",\"t\":" + std::to_string(t_ns) + ",\"agent\":" + std::to_string(agent) +
            ",\"kind\":\"" + kind + "\",\"kf\":" + std::to_string(kf) +
            ",\"payload\":\"collab_payloads/" + name + "\"}");
    }

    void LoadLog(const std::string& path) {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("collab replay: cannot read " + path);
        payload_dir_ = std::filesystem::path(path).parent_path() / "collab_payloads";
        std::string line, logged_identity;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            const YAML::Node n = YAML::Load(line);
            const std::string e = n["e"].as<std::string>();
            if (e == "commit") {
                const int64_t t = n["t"].as<int64_t>();
                const ac::AgentId agent = n["agent"].as<ac::AgentId>();
                const std::string kind = n["kind"].as<std::string>();
                const uint64_t kf = n["kf"].as<uint64_t>();
                commits_[t].emplace_back(agent, kind, kf);
                if (n["payload"]) {
                    const std::filesystem::path rel(n["payload"].as<std::string>());
                    if (rel.parent_path() != "collab_payloads" || rel.filename().empty())
                        throw std::runtime_error("collab replay: invalid payload path");
                    std::ifstream in(payload_dir_ / rel.filename(), std::ios::binary);
                    if (!in) throw std::runtime_error("collab replay: missing payload " + rel.filename().string());
                    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
                    replay_payloads_[{t, agent, kind, kf}] = std::move(bytes);
                } else replay_payloads_complete_ = false;
            }
            else if (e == "drain")
                for (const auto& k : n["keys"]) drains_[{n["T"].as<int64_t>(), n["agent"].as<ac::AgentId>()}].push_back(k.as<std::string>());
            else if (e == "snap")
                snaps_read_[n["T"].as<int64_t>()] = n["v"].as<uint64_t>();
            else if (e == "identity")
                logged_identity = n["v"].as<std::string>();
        }
        if (!p_.identity.empty() && !logged_identity.empty() && logged_identity != p_.identity)
            throw std::runtime_error("collab replay: " + path + " was written under identity " + logged_identity +
                                     ", this replay runs under " + p_.identity +
                                     " (config or build changed since the live run: it cannot be reproduced)");
    }

    CollabLiveParams p_;
    std::vector<ac::AgentId> fleet_ids_;
    ac::LocalEncoding enc_;
    ac::CommsNode& comms_;
    bool replay_;
    // runtime -> fleet thread
    std::mutex q_mu_;
    std::condition_variable q_cv_;
    std::deque<Item> q_;
    bool stopping_ = false;
    // workers -> fleet thread
    std::mutex in_mu_;
    std::condition_variable in_cv_;
    std::map<ac::AgentId, Peer> peers_;
    // fleet thread only
    std::map<std::pair<ac::AgentId, uint64_t>, std::pair<CollabStoreRecord, uint32_t>> staged_records_;
    std::map<std::pair<ac::AgentId, uint64_t>, std::optional<CollabLocalFeatures>> staged_replies_, ready_;
    std::set<std::pair<ac::AgentId, uint64_t>> asked_;
    std::map<std::pair<ac::AgentId, uint64_t>, std::optional<core::SE3>> gt_;
    std::map<std::pair<ac::AgentId, uint64_t>, std::array<float, 5>> camera_;
    uint64_t version_ = 0;
    int64_t tick_ns_ = 0;
    // fleet thread -> runtime
    std::mutex out_mu_;
    std::map<ac::AgentId, std::deque<std::pair<int64_t, core::DecodedMessage>>> outbox_;
    std::shared_ptr<const CollabSnapshot> snap_;
    // log / replay
    std::mutex log_mu_;
    std::ofstream log_;
    std::map<int64_t, std::vector<Commit>> commits_;
    std::map<std::tuple<int64_t, ac::AgentId, std::string, uint64_t>, std::vector<uint8_t>> replay_payloads_;
    bool replay_payloads_complete_ = true;
    std::filesystem::path payload_dir_;
    std::map<std::pair<int64_t, ac::AgentId>, std::vector<std::string>> drains_;
    std::map<int64_t, uint64_t> snaps_read_;
    std::map<uint64_t, std::shared_ptr<const CollabSnapshot>> history_;
    CollabFleet fleet_;   // its hooks capture `this`: constructed after every member above
    std::thread thread_;
};

}  // namespace ariadne::runtime::live
