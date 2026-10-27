// ---------------------------------------------------------------------------
// ariadne_live: the live closed-loop runtime (design LIVE_RUNTIME_L1, D150-D154). ROS-free.
//
//   ariadne_live --scene DIR --perception-config P --slam-config S --mission-config M --live-config L --out DIR
//                (--link SOCKET | --replay INPUTS.mcap)
//
// --link: listens on SOCKET for the ROS edge (ros2/ariadne_bridge RuntimeLink), takes (topic, CDR) frames, reorders
//   them by stamp behind a watermark, records exactly what it consumed (inputs.mcap) and sends commands back.
// --replay: feeds a previous run's inputs.mcap through the same code in the recorded order; no command leaves.
//
// Per agent, one thread runs the replay estimator path unchanged (RunStereoVo over a LiveSimSource: the same
// decoder, lens, gray conversion and pairing as SimSource). The mission thread integrates keyframe depth into the
// 1.8 map (planning/mission/frontier_mission.hpp) and replans every replan_period_s of SIM time, only after BOTH
// agents' estimators have passed that time, so a replan sees the same inputs live and in replay. At each replan the
// deterministic layer lists the reachable frontier clusters (the candidates), the chooser picks one (classical =
// nearest), and the choice becomes a follow() command along the A* path (D151). Every decision is a trace record;
// live records are executed: true and carry the command id (agent, stamp) the mover answers on mover_state.
//
// Outputs (run dir): trace.jsonl, commands.jsonl, mover_state.jsonl, telemetry.jsonl (console), snapshots/,
// raw/agent_<k>/... (the runner's own outputs, estimate_body.tum included), inputs.mcap (live), results.yaml,
// manifest.yaml.
// ---------------------------------------------------------------------------
#include "planning/mission/frontier_mission.hpp"
#include "runtime/live/link_frame.hpp"
#include "runtime/live/live_source.hpp"
#include "collaboration/comms/comms_node.hpp"
#include "collaboration/protocol/collab_node.hpp"
#include "runtime/live/a1_planner.hpp"
#include "runtime/live/agent_worker.hpp"
#include "runtime/live/cpu_affinity.hpp"
#include "runtime/live/collab_live.hpp"
#include "runtime/live/lockstep_gate.hpp"
#include "runtime/live/sensor_horizon.hpp"
#include "runtime/live/collab_tee.hpp"
#include "runtime/live/multiagent_glue.hpp"
#include "runtime/live/planner.hpp"
#include "runtime/manifest/build_identity.hpp"
#include "runtime/manifest/determinism.hpp"
#include "runtime/manifest/opencv_identity.hpp"
#include "runtime/manifest/run_manifest.hpp"
#include "runtime/replay/ros_cdr_writer.hpp"
#include "runtime/tools/stereo_vo_runner.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace ariadne;
using planning::Cell;
namespace live = runtime::live;
namespace mis = planning::mission;

namespace {

struct LiveConfig {
    std::string name, chooser;
    double end_s = 0, max_command_m = 0, eps_cells = 1, telemetry_hz = 5, min_goal_m = 0, escape_m = 0,
           max_standoff_m = 0, refused_radius_m = 0;
    double autonomous_threshold = 1.01;   // §23 risk gate (AGENTIC_7 D158); > 1 = no gate
    double llm_timeout_s = 0;              // chooser llm: wall seconds to wait for the orchestrator
    double command_max_age_s = 0;          // live demo: a decision older than this (sim s, clock at send - T) is not
                                           // sent; the agent keeps its current motion. 0 = off. A safety cap, not a
                                           // latency fix: the age itself comes from decision time
    bool face_frontier = true;
    bool lockstep = false;                 // D165: gz paused at every barrier until the commands are applied
    int lockstep_substep_ms = 500;         // D166: gz paced toward each barrier in sub-steps of this much sim time,
                                           // gated on consumption, so no queue between gz and the estimators can
                                           // hold more than one sub-step (uncapped RTF would otherwise overflow the
                                           // intra-process rings, which drop their oldest when full - rclcpp jazzy
                                           // ring_buffer_implementation.hpp L83). <= 0 restores the single 5 s jump.
    bool estimator_processes = false;      // D168: one estimator PROCESS per agent (its own OpenCV pool) instead of
                                           // the D152a turn lock (which costs 109-126 ms/frame of waiting at 2
                                           // agents). false until the acceptance (live + 3 equal replays) passes.
    bool lens = false;                     // D227 (the D41 amendment, maintainer-directed): the read-time synthetic
                                           // lens. OFF by default for every live/demo/POC run - A2 measured it at
                                           // ~58 ms/frame, about half the lane - and the calibration then reports
                                           // the render pinhole with D = 0. `lens: true` restores the D41 operator.
    bool demo = false;                     // D169 (AGENTIC_7 §4): judges-live mode. No lockstep, keyframes integrate
                                           // on a background thread, no per-replan PNGs, and the LLM is supervisory
                                           // and NON-BLOCKING: the deterministic pick is commanded immediately, the
                                           // answer applies at the next horizon or expires. Not an evidence mode.
    int hold_back_ms = 3000;
    // D175 item 4 (MULTIAGENT_PIPELINE): per-agent planners fed only through the radio. false = the centralised
    // loop, byte-identical to before this feature existed (the oracle row).
    struct Multiagent {
        bool enabled = false;
        std::string scenario;                 // comms scenario yaml (3.3); live runs need mode: ideal (RF needs tracks)
        std::string broker_mode = "plain";    // plain | discount | overlap (A1 R1/R2)
        double consensus_timeout_s = 10.0;    // older consensus than this = uncoordinated
        double discount_range_m = 4.0;        // R2 P(d) range (broker_mode discount)
        double discount_tau_s = 10.0;         // R2 cut-off age constant [ASSUMPTION, logged]
        int max_bid_options = 8;              // top-M options per bid (D175 wire table cap)
    } multiagent;
    // D174 (LIVE_COLLAB): the live SLAM collaboration. The DNN work runs in per-agent ariadne_collab_worker
    // processes (cv::ALIKED output depends on the thread count; inside the estimator it would break the D168
    // digests); the estimator only TEES. false = D168 digests unchanged. Requires multiagent.enabled: SLAM and
    // coordination share one radio and one ledger.
    struct Collab {
        bool enabled = false;
        std::string config;                 // CollabConfig yaml (arm, budget, retrieval, verify, local_encoding)
        double feature_latency_s = 3.0;     // sim-time model of one ALIKED extraction (lockstep; manifest)
        int worker_threads = 4;             // OpenCV threads per collab worker, pinned (output depends on it)
        double keep_s = 120;
        std::string place_onnx, place_size, aliked_onnx, lightglue_onnx;
        std::string dnn_backend = "cv_dnn";   // ALIKED + LightGlue: cv_dnn | ort_cuda | ort_cpu (HardPairParams)
    } collab;
};

LiveConfig LoadLive(const std::string& path) {
    const YAML::Node n = YAML::LoadFile(path);
    LiveConfig c;
    c.name = n["name"].as<std::string>();
    c.end_s = n["end_s"].as<double>();
    c.chooser = n["chooser"].as<std::string>();
    c.max_command_m = n["command"]["max_length_m"].as<double>();
    c.eps_cells = n["command"]["simplify_eps_cells"].as<double>();
    c.face_frontier = n["command"]["face_frontier"].as<bool>();
    c.min_goal_m = n["command"]["min_goal_m"].as<double>();
    c.escape_m = n["command"]["escape_m"].as<double>();
    c.max_standoff_m = n["command"]["max_standoff_m"].as<double>();
    c.refused_radius_m = n["command"]["refused_radius_m"].as<double>();
    if (n["command"]["max_age_s"]) c.command_max_age_s = n["command"]["max_age_s"].as<double>();
    if (c.command_max_age_s < 0) throw std::invalid_argument("live config: command.max_age_s >= 0");
    c.hold_back_ms = n["reorder"]["hold_back_ms"].as<int>();
    c.telemetry_hz = n["telemetry_hz"].as<double>();
    if (n["lockstep"]) c.lockstep = n["lockstep"].as<bool>();
    if (n["lockstep_substep_ms"]) c.lockstep_substep_ms = n["lockstep_substep_ms"].as<int>();
    if (n["estimator_processes"]) c.estimator_processes = n["estimator_processes"].as<bool>();
    if (n["lens"]) c.lens = n["lens"].as<bool>();   // absent = off (D227)
    if (n["mode"]) c.demo = n["mode"].as<std::string>() == "demo";
    if (c.demo) c.lockstep = false;   // demo never pauses the world, whatever the lockstep key says
    if (c.chooser != "classical" && c.chooser != "llm") throw std::invalid_argument("live config: chooser classical|llm");
    c.autonomous_threshold = n["risk_gate"]["autonomous_threshold"].as<double>();
    if (c.chooser == "llm") c.llm_timeout_s = n["llm"]["timeout_s"].as<double>();
    if (!(c.end_s > 0) || !(c.max_command_m > 0) || !(c.telemetry_hz > 0)) throw std::invalid_argument("live config: bad values");
    if (n["multiagent"]) {
        const YAML::Node m = n["multiagent"];
        c.multiagent.enabled = m["enabled"].as<bool>();
        if (c.multiagent.enabled) {
            c.multiagent.scenario = m["scenario"].as<std::string>();
            if (m["broker_mode"]) c.multiagent.broker_mode = m["broker_mode"].as<std::string>();
            if (m["consensus_timeout_s"]) c.multiagent.consensus_timeout_s = m["consensus_timeout_s"].as<double>();
            if (m["discount_range_m"]) c.multiagent.discount_range_m = m["discount_range_m"].as<double>();
            if (m["discount_tau_s"]) c.multiagent.discount_tau_s = m["discount_tau_s"].as<double>();
            if (m["max_bid_options"]) c.multiagent.max_bid_options = m["max_bid_options"].as<int>();
            if (c.multiagent.broker_mode != "plain" && c.multiagent.broker_mode != "discount" &&
                c.multiagent.broker_mode != "overlap")
                throw std::invalid_argument("live config: multiagent.broker_mode plain|discount|overlap");
        }
    }
    if (n["collab"]) {
        const YAML::Node m = n["collab"];
        c.collab.enabled = m["enabled"].as<bool>();
        if (c.collab.enabled) {
            if (!c.multiagent.enabled) throw std::invalid_argument("live config: collab.enabled requires multiagent.enabled");
            c.collab.config = m["config"].as<std::string>();
            if (m["feature_latency_s"]) c.collab.feature_latency_s = m["feature_latency_s"].as<double>();
            if (m["worker_threads"]) c.collab.worker_threads = m["worker_threads"].as<int>();
            if (m["keep_s"]) c.collab.keep_s = m["keep_s"].as<double>();
            const YAML::Node mo = m["models"];
            c.collab.place_onnx = mo["place"].as<std::string>();
            c.collab.place_size = mo["place_size"].as<std::string>();
            c.collab.aliked_onnx = mo["aliked"].as<std::string>();
            c.collab.lightglue_onnx = mo["lightglue"].as<std::string>();
            if (m["dnn_backend"]) c.collab.dnn_backend = m["dnn_backend"].as<std::string>();
        }
    }
    return c;
}

/// One keyframe for the map, at the spool's scale (the 1.8 carving values assume 640x360 depth, D153).
struct KfRec {
    int agent = 0;
    uint64_t id = 0;
    int64_t t_ns = 0;
    core::SE3 T_O_C;
    cv::Mat Z, S;   // CV_32F, half resolution, fp16-rounded exactly as the spool stores them
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::vector<cv::Point2f> track_px;   // the tracker's left-pixel observations (full-res; console overlay)
    bool mapped = false;   // demo: already integrated by the agent's worker at delivery (Z/S moved there)
};

cv::Mat HalfAsSpool(const cv::Mat& f32) {   // KeyframeSpool::WriteHalf + ReadHalf, without the PNG round trip
    cv::Mat d, h, back;
    cv::resize(f32, d, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    d.convertTo(h, CV_16F);
    h.convertTo(back, CV_32F);
    return back;
}

// --- A2's live splat tee (D220; service contract D231) -----------------------------------------------------------
// ariadne_live CONNECTS to the recon service's Unix socket (env ARIADNE_RECON_SOCK) and tees every delivered
// keyframe: topic "#kf" = u32 LE json_len | JSON | Z fp16 | S fp16 | RGB rgb8 (JSON carries both intrinsics: K
// full-res for the RGB, Kz spool-scale matching Z/S; invalid depth is 0, the spool convention), topic "#t_w_o" =
// {"agent","t_s","xyzquat"} with qw LAST (Eigen::Quaterniond - a naive quaternion breaks at the 180-degree
// entrance), sent once per agent before any keyframe. The splat is a VIEWER: a bounded queue (8 per agent, drop
// the oldest, drops counted) on one sender thread; absent or dying socket = silent no-op. It can never
// back-pressure VO or the mission.
class SplatTee {
public:
    SplatTee(const char* path, const std::vector<core::SE3>& T_W_O) {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return;
        sockaddr_un a{};
        a.sun_family = AF_UNIX;
        if (std::strlen(path) >= sizeof(a.sun_path)) { ::close(fd_), fd_ = -1; return; }
        std::strcpy(a.sun_path, path);
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { ::close(fd_), fd_ = -1; return; }
        pending_.assign(T_W_O.size(), 0);
        for (size_t k = 0; k < T_W_O.size(); ++k) {
            const core::SE3& T = T_W_O[k];
            const Eigen::Quaterniond q(T.R);
            std::ostringstream o;
            o.precision(17);
            o << "{\"agent\":" << k << ",\"t_s\":0,\"xyzquat\":[" << T.t.x() << "," << T.t.y() << "," << T.t.z()
              << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "]}";
            const std::string j = o.str();
            Msg m;
            m.topic = "#t_w_o";
            m.bytes.assign(j.begin(), j.end());
            queue_.push_back(std::move(m));
        }
        th_ = std::thread([this] { Run(); });
    }
    ~SplatTee() {
        if (fd_ < 0) return;
        {
            std::lock_guard<std::mutex> g(mu_);
            stop_ = true;
        }
        cv_.notify_one();
        th_.join();
        ::close(fd_);
    }
    bool on() const { return fd_ >= 0 && !dead_.load(std::memory_order_relaxed); }
    uint64_t drops() const { return drops_.load(); }
    uint64_t sends() const { return sends_; }           // sender-thread counters: read after join (D224 profiling)
    uint64_t sent_bytes() const { return sent_bytes_; }
    double busy_ms() const { return busy_ms_; }
    void OnKf(const KfRec& r, const cv::Mat& rgb) {
        if (!on()) return;
        cv::Mat z16, s16;
        r.Z.convertTo(z16, CV_16F);
        r.S.convertTo(s16, CV_16F);
        const cv::Mat c = rgb.isContinuous() ? rgb : rgb.clone();
        std::ostringstream o;
        o.precision(17);
        o << "{\"agent\":" << r.agent << ",\"id\":" << r.id << ",\"t_ns\":" << r.t_ns << ",\"pose_version\":0,"
          << "\"T_O_C\":[";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                o << ((i || j) ? "," : "") << (j < 3 ? r.T_O_C.R(i, j) : r.T_O_C.t(i));
        // Spool camera -> full resolution: u = 2 u' + 0.5 (the inverse of HalfAsSpool's mapping).
        o << "],\"K\":[" << 2 * r.fx << "," << 2 * r.fy << "," << 2 * (r.cx + 0.5) - 0.5 << ","
          << 2 * (r.cy + 0.5) - 0.5 << "],\"Kz\":[" << r.fx << "," << r.fy << "," << r.cx << "," << r.cy
          << "],\"w\":" << c.cols << ",\"h\":" << c.rows << ",\"zw\":" << r.Z.cols << ",\"zh\":" << r.Z.rows
          << ",\"training_image\":\"rgb\"}";
        const std::string j = o.str();
        Msg m;
        m.topic = "#kf";
        m.agent = r.agent;
        m.bytes.reserve(4 + j.size() + z16.total() * 2 + s16.total() * 2 + c.total() * 3);
        const uint32_t jl = uint32_t(j.size());   // u32 LE (PutPod lives further down this file)
        m.bytes.insert(m.bytes.end(), reinterpret_cast<const uint8_t*>(&jl), reinterpret_cast<const uint8_t*>(&jl) + 4);
        m.bytes.insert(m.bytes.end(), j.begin(), j.end());
        const auto put = [&m](const cv::Mat& x) {
            const auto* q = x.ptr<uint8_t>();
            m.bytes.insert(m.bytes.end(), q, q + x.total() * x.elemSize());
        };
        put(z16), put(s16), put(c);
        {
            std::lock_guard<std::mutex> g(mu_);
            auto& n = pending_[size_t(r.agent)];
            if (n >= 8) {   // the viewer lags: drop ITS oldest frame, never block
                for (auto it = queue_.begin(); it != queue_.end(); ++it) {
                    if (it->topic == "#kf" && it->agent == r.agent) {
                        queue_.erase(it), --n, ++drops_;
                        break;
                    }
                }
            }
            queue_.push_back(std::move(m)), ++n;
        }
        cv_.notify_one();
    }

private:
    struct Msg {
        std::string topic;
        int agent = -1;
        std::vector<uint8_t> bytes;
    };
    void Run() {
        std::unique_lock<std::mutex> lk(mu_);
        for (;;) {
            cv_.wait(lk, [&] { return stop_ || !queue_.empty(); });
            if (queue_.empty()) return;   // stop_ and drained
            Msg m = std::move(queue_.front());
            queue_.pop_front();
            if (m.agent >= 0) --pending_[size_t(m.agent)];
            lk.unlock();
            if (!dead_.load(std::memory_order_relaxed)) {
                const auto w0 = std::chrono::steady_clock::now();
                if (!live::WriteFrame(fd_, m.topic, m.bytes.data(), m.bytes.size()))
                    dead_.store(true, std::memory_order_relaxed);   // service gone: drain silently, never retry mid-run
                busy_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
                ++sends_;
                sent_bytes_ += m.bytes.size();
            }
            lk.lock();
            if (stop_ && queue_.empty()) return;
        }
    }
    int fd_ = -1;
    std::atomic<bool> dead_{false};
    std::atomic<uint64_t> drops_{0};
    uint64_t sends_ = 0, sent_bytes_ = 0;   // sender thread only
    double busy_ms_ = 0;
    bool stop_ = false;
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Msg> queue_;
    std::vector<int> pending_;
    std::thread th_;
};

struct AgentLane {
    live::MessageQueue images{64};   // ~64 x 2.8 MB per agent at most (bounded: see MessageQueue)
    live::MessageQueue imus{512};    // 2.5 s at 200 Hz; preserves timestamp order and bounds backlog
    live::MessageQueue worker_inputs{128};  // bounded release-order FIFO keeps image/IMU transport ordered
    live::GtStore gt;
    std::mutex mu;
    std::deque<KfRec> kfs;                               // delivered by the VO thread, consumed by the mission
    std::vector<std::pair<int64_t, core::SE3>> cams;     // every frame's T_O_C (rectified left camera)
    std::vector<int64_t> kf_times;                       // every keyframe's stamp (estimator facts for the chooser)
    cv::Mat last_rgb;                                    // the latest keyframe's rectified RGB (observe_image)
    int64_t progress = -1;                               // stamp of the last frame the estimator finished
    bool done = false;
    size_t frames = 0, keyframes = 0, unpaired = 0;
    int64_t last_telemetry_ns = -1;
};

class Lines {   // a thread-safe JSONL file
public:
    explicit Lines(const fs::path& p) : f_(p) { f_.precision(10); }
    void Put(const std::string& s) {
        std::lock_guard<std::mutex> lk(mu_);
        f_ << s << "\n";
        f_.flush();
    }

private:
    std::mutex mu_;
    std::ofstream f_;
};

std::string Xy(double x, double y) {
    std::ostringstream o;
    o.precision(10);
    o << "[" << x << "," << y << "]";
    return o.str();
}

double YawOfCamera(const core::SE3& T_W_C) {   // heading of the optical axis (z) in the horizontal plane
    return std::atan2(T_W_C.R(1, 2), T_W_C.R(0, 2));
}

// --- D168: one estimator per PROCESS ------------------------------------------------------------------------
// D152a serialized the two estimators with a turn lock because they shared OpenCV's one parallel_for pool (a busy
// pool changes stripe partitioning, so the order of floating-point reductions). At 2 agents that lock is 109-126 ms
// of waiting per 140 ms frame. A worker process per agent gives each estimator its own pool: no lock, and the
// partitioning depends only on that process's own load. Wire = LinkFrame on a Unix socket:
//   parent -> worker: (topic, CDR) for this agent's left/right images and gt pose, EOF at end of input;
//   worker -> parent: #frame (t_ns, T_O_C), #kf (id, t_ns, T_O_C, spool intrinsics, Z, S, rgb), #timing
//                     (t_ns, wait/lock/track), #done (unpaired), then EOF.
template <typename T>
void PutPod(std::vector<uint8_t>& b, const T& v) {
    const auto* q = reinterpret_cast<const uint8_t*>(&v);
    b.insert(b.end(), q, q + sizeof v);
}
template <typename T>
T GetPod(const uint8_t*& q) {
    T v;
    std::memcpy(&v, q, sizeof v);
    q += sizeof v;
    return v;
}
void PutMat(std::vector<uint8_t>& b, const cv::Mat& m0) {
    const cv::Mat m = m0.isContinuous() ? m0 : m0.clone();
    PutPod(b, int32_t(m.rows)), PutPod(b, int32_t(m.cols)), PutPod(b, int32_t(m.type()));
    const auto* q = m.ptr<uint8_t>();
    b.insert(b.end(), q, q + m.total() * m.elemSize());
}
cv::Mat GetMat(const uint8_t*& q) {
    const int rows = GetPod<int32_t>(q), cols = GetPod<int32_t>(q), type = GetPod<int32_t>(q);
    cv::Mat m(rows, cols, type);
    std::memcpy(m.ptr<uint8_t>(), q, m.total() * m.elemSize());
    q += m.total() * m.elemSize();
    return m;
}
void PutSe3(std::vector<uint8_t>& b, const core::SE3& T) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) PutPod(b, double(T.R(i, j)));
    for (int i = 0; i < 3; ++i) PutPod(b, double(T.t(i)));
}
core::SE3 GetSe3(const uint8_t*& q) {
    core::SE3 T;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) T.R(i, j) = GetPod<double>(q);
    for (int i = 0; i < 3; ++i) T.t(i) = GetPod<double>(q);
    return T;
}

int RunEstimatorWorker(const std::string& scene, int agent, const std::string& socket_path,
                       const std::string& collab_socket, bool lens_on, bool latest_stereo, tools::StereoVoArgs va, const std::string& out,
                       int argc, char** argv) {
    live::PinFromEnv(("ARIADNE_CPUSET_ESTIMATOR_" + std::to_string(agent)).c_str(), "estimator");   // per agent first,
    if (!std::getenv(("ARIADNE_CPUSET_ESTIMATOR_" + std::to_string(agent)).c_str()))                // else the shared set
        live::PinFromEnv("ARIADNE_CPUSET_ESTIMATOR", "estimator");
    // D228: 5 OpenCV threads per worker. The estimator is thread-count-invariant (A2: 16/5/2 bit-identical) and a
    // 3-worker fleet at the default pool thrashes the box (cpu PSI 26 % -> 1.2 % at 5 on c7i.4xlarge, job 46).
    {
        int n = 5;
        if (const char* e = std::getenv("ARIADNE_WORKER_CV_THREADS")) n = std::max(1, std::atoi(e));
        cv::setNumThreads(n);
    }
    const int fd = live::Connect(socket_path, std::chrono::seconds(60));
    // D174: the collab tee - serialization only, NO DNN in this process (cv::ALIKED output depends on the thread
    // count; here it would break the D168 digests). A dead collab worker quiets the tee, never the estimator.
    std::optional<live::CollabTee> tee;
    if (!collab_socket.empty()) tee.emplace(live::Connect(collab_socket, std::chrono::seconds(60)), uint32_t(agent));
    live::MessageQueue images{64};
    live::MessageQueue imus{512};
    live::GtStore gt;
    std::mutex turn;   // never contended: one estimator per process is the point
    // D229: the lane ingest pipeline - measured NO RTF gain lens-off (A2 job 15: 0.833 vs 0.839; the lane is
    // track/keyframe compute, not ingest), so the default is OFF (M: an unmeasured extra thread does not belong
    // on the demo path). ARIADNE_LANE_PIPELINE=1 enables it; its payoff case is a live path with heavy ingest
    // (lens-on or degradation rows).
    const char* lp = std::getenv("ARIADNE_LANE_PIPELINE");
    const bool pipelined = lp && std::string(lp) == "1";
    live::LiveSimSource src(scene, agent, images, imus, gt, turn, "live", lens_on, pipelined);
    images.ConfigureSensor(512ULL * 1024 * 1024,
                           latest_stereo ? src.left_topic() : "", latest_stereo ? src.right_topic() : "");
    imus.ConfigureSensor(64ULL * 1024 * 1024);
    Lines ingest_log(fs::path(out) / ("sensor_ingest_agent" + std::to_string(agent) + ".jsonl"));
    if (latest_stereo) images.Observe([&](const char* kind, int64_t stamp) {
        ingest_log.Put("{\"stage\":\"worker\",\"event\":\"" + std::string(kind) +
                       "\",\"stamp_ns\":" + std::to_string(stamp) + "}");
    });
    std::exception_ptr sensor_error;
    std::mutex send_mu;
    auto send = [&](const char* topic, const std::vector<uint8_t>& b) {
        std::lock_guard<std::mutex> lk(send_mu);
        if (!live::WriteFrame(fd, topic, b.data(), b.size())) std::_Exit(3);   // parent gone: nothing left to save
    };
    std::thread reader([&] {
        try {
        live::LinkFrame f;
        while (live::ReadFrame(fd, f)) {
            if (f.topic == src.left_topic() || f.topic == src.right_topic()) {
                live::StreamMessage m;
                m.topic = f.topic;
                m.stamp_ns = live::PeekStamp("sensor_msgs/msg/Image", f.payload);
                m.data = std::make_shared<const std::vector<uint8_t>>(std::move(f.payload));
                images.Push(std::move(m));
            } else if (f.topic == src.imu_topic()) {
                live::StreamMessage m;
                m.topic = f.topic;
                m.data = std::make_shared<const std::vector<uint8_t>>(std::move(f.payload));
                imus.Push(std::move(m));
            } else {   // this agent's gt pose, raw CDR exactly as the link delivered it
                const auto g = runtime::ros_cdr::DecodePoseStamped(f.payload.data(), f.payload.size());
                runtime::TimedPose tp{g.header.stamp.ns(), {}};
                tp.T.R = Eigen::Quaterniond(g.orientation.w, g.orientation.x, g.orientation.y, g.orientation.z)
                             .normalized()
                             .toRotationMatrix();
                tp.T.t = core::Vec3(g.position.x, g.position.y, g.position.z);
                gt.Add(tp);
            }
        }
        }
        catch (...) { sensor_error = std::current_exception(); }
        images.Close();
        imus.Close();
    });
    src.on_timing = [&](int64_t t_ns, double w, double l, double tr) {
        std::vector<uint8_t> b;
        PutPod(b, t_ns), PutPod(b, w), PutPod(b, l), PutPod(b, tr);
        send("#timing", b);
    };
    tools::StereoVoArgs a = va;
    a.out = out;
    a.tool = std::string("ariadne_live/agent") + std::to_string(agent);
    a.late_ground_truth = [&gt] { return gt.Snapshot(); };
    a.on_keyframe = [&](const tools::KeyframeView& kv) {
        const runtime::KeyframeBundle& b = kv.bundle;
        std::vector<uint8_t> o;
        PutPod(o, uint64_t(kv.keyframe.id)), PutPod(o, int64_t(kv.keyframe.t_ns));
        PutSe3(o, b.T_WiC);
        // Spool camera: u' + 0.5 = 0.5 (u + 0.5).
        PutPod(o, double(b.K.fx * 0.5)), PutPod(o, double(b.K.fy * 0.5));
        PutPod(o, double(0.5 * (b.K.cx + 0.5) - 0.5)), PutPod(o, double(0.5 * (b.K.cy + 0.5) - 0.5));
        PutMat(o, HalfAsSpool(runtime::MaskedDepth32(b.depth, b.mask)));
        PutMat(o, HalfAsSpool(runtime::MaskedDepth32(b.depth_sigma, b.mask)));
        PutMat(o, b.rgb);
        // Phase 6 training image (RECON_6, A2/M 2026-10-23): a tag then, when not "none", albedo (half-res CV_32F,
        // fp16-rounded) + albedo_valid (half-res CV_8U) from KeyframeFalloff::Normalize. That code lives on
        // feat/ph5-dve (FramePhoto, falloff_glue) and is wired here once it reaches main; until then every record
        // says "none" and the recon service refuses to train live unless the run is the RGB ablation - nothing
        // silently falls back.
        const std::string ti = "none";
        PutPod(o, uint32_t(ti.size()));
        o.insert(o.end(), ti.begin(), ti.end());
        if (tee) {   // D174: #ckin + the releases whose odometry became final at this keyframe (gt EVAL ONLY)
            std::optional<core::SE3> gt_T_W_B;
            const auto snap = gt.Snapshot();   // ~20 kf/run: the copy is noise
            for (const auto& tp : snap)
                if (std::llabs(tp.t_ns - kv.keyframe.t_ns) <= 10000000 &&
                    (!gt_T_W_B || std::llabs(tp.t_ns - kv.keyframe.t_ns) < 5000000))
                    gt_T_W_B = tp.T;
            tee->OnKeyframe(kv, gt_T_W_B);
        }
        // Console overlay (P8): the tracked observations, one CV_32FC2 row. Appended last so older readers
        // (there are none - worker and parent are the same binary) would simply stop before it.
        PutMat(o, kv.track_px.empty() ? cv::Mat(0, 1, CV_32FC2) : cv::Mat(kv.track_px).reshape(2, 1));
        send("#kf", o);
    };
    a.on_frame = [&](const tools::FrameView& fv) {
        if (tee) tee->OnFrame(fv.t_ns);   // D174: the lockstep watermark source (#cprog)
        std::vector<uint8_t> o;
        PutPod(o, int64_t(fv.t_ns));
        PutSe3(o, fv.T_O_C);
        send("#frame", o);
    };
    int rc = 0;
    try {
        tools::RunStereoVo(src, a, argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ariadne_live worker %d: %s\n", agent, e.what());
        rc = 1;
    }
    std::vector<uint8_t> o;
    PutPod(o, uint64_t(src.unpaired()));
    send("#done", o);
    ::shutdown(fd, SHUT_RDWR);
    images.Close(); imus.Close();
    reader.join();
    if (sensor_error) {
        try { std::rethrow_exception(sensor_error); }
        catch (const std::exception& e) { std::fprintf(stderr, "sensor reader: %s\n", e.what()); }
        rc = 1;
    }
    std::fprintf(stderr, "sensor queues agent=%d dropped_pairs=%zu dropped_incomplete=%zu late_images=%zu image_peak_bytes=%zu imu_peak_bytes=%zu\n",
                 agent, images.dropped_pairs(), images.dropped_incomplete(), images.late_images(), images.max_bytes(), imus.max_bytes());
    ::close(fd);
    return rc;
}

/// chooser: llm, live (AGENTIC_7 D156): one JSON line per decision request to the orchestrator's Unix socket; answers
/// come back as JSON lines keyed by decision_id, in any order.
class ChooserLink {
public:
    explicit ChooserLink(const std::string& path) : fd_(live::Connect(path, std::chrono::seconds(60))) {
        reader_ = std::thread([this] {
            std::string buf;
            char tmp[4096];
            for (;;) {
                const ssize_t n = ::recv(fd_, tmp, sizeof tmp, 0);
                if (n <= 0) break;
                buf.append(tmp, size_t(n));
                size_t nl;
                while ((nl = buf.find('\n')) != std::string::npos) {
                    std::string line = buf.substr(0, nl);
                    buf.erase(0, nl + 1);
                    try {
                        const std::string id = YAML::Load(line)["decision_id"].as<std::string>();
                        std::lock_guard<std::mutex> lk(mu_);
                        answers_[id] = std::move(line);
                    } catch (const std::exception&) {
                    }
                    cv_.notify_all();
                }
            }
            std::lock_guard<std::mutex> lk(mu_);
            closed_ = true;
            cv_.notify_all();
        });
    }
    ~ChooserLink() {
        ::shutdown(fd_, SHUT_RDWR);
        reader_.join();
        ::close(fd_);
    }
    void Send(const std::string& line) {
        const std::string l = line + "\n";
        if (::send(fd_, l.data(), l.size(), MSG_NOSIGNAL) != ssize_t(l.size())) throw std::runtime_error("chooser: send");
    }
    /// The raw answer line, or "" on timeout / closed link.
    std::string Wait(const std::string& id, std::chrono::steady_clock::time_point until) {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait_until(lk, until, [&] { return answers_.count(id) || closed_; });
        const auto it = answers_.find(id);
        return it == answers_.end() ? "" : it->second;
    }

private:
    int fd_;
    std::thread reader_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<std::string, std::string> answers_;
    bool closed_ = false;
};

/// chooser: llm, replay (AGENTIC_7 D161): the live trace's choices, keyed by (t_s text, agent): the chosen index and the
/// raw `"choice":{...}` tail, re-emitted verbatim. The model is never called in replay.
std::map<std::pair<std::string, int>, std::pair<int, std::string>> LoadRecordedChoices(const std::string& trace) {
    std::map<std::pair<std::string, int>, std::pair<int, std::string>> out;
    std::ifstream f(trace);
    if (!f) throw std::runtime_error("replay: cannot open the live trace " + trace);
    std::string line;
    while (std::getline(f, line)) {
        const YAML::Node n = YAML::Load(line);
        if (!n["choice"]) continue;
        const auto p = line.find(",\"choice\":");
        std::ostringstream t;
        t.precision(10);
        t << n["t_s"].as<double>();
        out[{t.str(), n["agent"].as<int>()}] = {n["chosen"].as<int>(), line.substr(p, line.size() - 1 - p)};
    }
    return out;
}

std::string JsonEscape(const std::string& s) {
    std::string o;
    for (char ch : s) {
        if (ch == '"' || ch == '\\') o += '\\';
        if (ch == '\n') o += "\\n";
        else o += ch;
    }
    return o;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scene, mission_cfg_path, live_cfg_path, out, link_path, replay_path, chooser_socket, decisions_path,
            directives_path, planner_cfg_path, shadow_cfg_path, worker_socket, collab_socket;
        int worker_agent = -1;
        std::string ingest_arg = "lossless";
        std::string lens_arg = "off";   // D227: the D41 lens is off unless the parent (LiveConfig.lens) says on
        tools::StereoVoArgs va;
        for (int i = 1; i < argc; ++i) {
            const std::string k = argv[i];
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + k);
            const std::string v = argv[++i];
            if (k == "--scene") scene = v;
            else if (k == "--perception-config") va.perception_config = v;
            else if (k == "--slam-config") va.slam_config = v;
            else if (k == "--imu-noise-config") va.imu_noise_config = v;
            else if (k == "--mission-config") mission_cfg_path = v;
            else if (k == "--live-config") live_cfg_path = v;
            else if (k == "--out") out = v;
            else if (k == "--link") link_path = v;
            else if (k == "--replay") replay_path = v;
            else if (k == "--chooser-socket") chooser_socket = v;   // chooser llm, live
            else if (k == "--decisions") decisions_path = v;        // chooser llm, replay: the live trace.jsonl
            else if (k == "--directives") directives_path = v;      // replay: the live run's directives.jsonl
            else if (k == "--planner-config") planner_cfg_path = v;  // A1's Phase 4 planner (mission_v2.yaml); else 1.8
            else if (k == "--shadow-map-config") shadow_cfg_path = v;   // one map for every row: p4slab snapshots
            else if (k == "--seed") va.seed = std::stoull(v);
            else if (k == "--estimator-worker") worker_agent = std::stoi(v);   // D168: internal, spawned by the parent
            else if (k == "--worker-socket") worker_socket = v;
            else if (k == "--collab-socket") collab_socket = v;   // D174: this agent's ariadne_collab_worker
            else if (k == "--ingest-policy") ingest_arg = v;
            else if (k == "--lens") lens_arg = v;                 // worker mode (D227): on|off, from LiveConfig.lens
            else throw std::invalid_argument("unknown argument " + k);
        }
        if (lens_arg != "on" && lens_arg != "off") throw std::invalid_argument("--lens on|off");
        if (ingest_arg != "lossless" && ingest_arg != "latest_stereo") throw std::invalid_argument("--ingest-policy lossless|latest_stereo");
        if (worker_agent >= 0) {
            if (scene.empty() || va.perception_config.empty() || va.slam_config.empty() || out.empty() || worker_socket.empty())
                throw std::invalid_argument("worker mode: --scene --perception-config --slam-config --out --worker-socket");
            return RunEstimatorWorker(scene, worker_agent, worker_socket, collab_socket, lens_arg == "on", ingest_arg == "latest_stereo", va, out,
                                      argc, argv);
        }
        if (scene.empty() || mission_cfg_path.empty() || live_cfg_path.empty() || out.empty() ||
            va.perception_config.empty() || va.slam_config.empty() || link_path.empty() == replay_path.empty()) {
            throw std::invalid_argument("usage: ariadne_live --scene S --perception-config P --slam-config S "
                                        "--mission-config M --live-config L --out DIR (--link SOCK | --replay MCAP)");
        }
        // Live tuning knobs (cpu_affinity.hpp): pin the main process before any thread exists, cap its OpenCV pool.
        live::PinFromEnv("ARIADNE_CPUSET_MAIN", "ariadne_live main");
        if (const char* e = std::getenv("ARIADNE_RT_CV_THREADS")) {
            cv::setNumThreads(std::max(1, std::atoi(e)));
            std::fprintf(stderr, "[threads] ariadne_live main: OpenCV pool %d (ARIADNE_RT_CV_THREADS)\n", cv::getNumThreads());
        }
        const bool is_live = !link_path.empty();
        const LiveConfig lc = LoadLive(live_cfg_path);
        if (lc.chooser == "llm" && is_live && chooser_socket.empty()) throw std::invalid_argument("chooser llm needs --chooser-socket");
        if (lc.chooser == "llm" && !is_live && decisions_path.empty() == chooser_socket.empty())
            throw std::invalid_argument("chooser llm replay needs --decisions (the live trace) or --chooser-socket (a test)");
        // TEST ONLY (D225 guard): ARIADNE_TEST_LANE_DELAY="k:ms" wall-delays lane k's parent-side delivery by ms
        // per frame. It changes NO data, only timing, so a correct barrier yields identical traces with and
        // without it; the pre-D225 hardcoded wait trips it. Never set outside infra/scripts/test_barrier_n3.sh.
        int test_delay_lane = -1, test_delay_ms = 0;
        if (const char* td = std::getenv("ARIADNE_TEST_LANE_DELAY")) {
            if (std::sscanf(td, "%d:%d", &test_delay_lane, &test_delay_ms) != 2 || test_delay_ms < 0)
                throw std::invalid_argument("ARIADNE_TEST_LANE_DELAY: k:ms");
            std::fprintf(stderr, "ariadne_live: TEST lane delay %d ms on lane %d\n", test_delay_ms, test_delay_lane);
        }
        const auto recorded = lc.chooser == "llm" && !decisions_path.empty() ? LoadRecordedChoices(decisions_path)
                                                              : std::map<std::pair<std::string, int>, std::pair<int, std::string>>{};
        const mis::MissionConfig mc = mis::LoadMissionConfig(mission_cfg_path);
        const int NA = int(YAML::LoadFile(scene + "/mission_live.json")["agents"].size());   // P8 D213: N agents
        // Non-const since D174: the collaboration's adopted frames move T_W_O[k] at barriers (the "known entry
        // pose" is the INITIAL value, refined by fleet estimates); everything else in it stays fixed.
        mis::Briefing brief = mis::LoadBriefing(scene, mc, NA, "live");
        live::PlaceholderParams pp;   // the placeholder planner's knobs (live config), not part of the seam
        pp.min_goal_m = lc.min_goal_m, pp.escape_m = lc.escape_m, pp.max_standoff_m = lc.max_standoff_m;
        pp.refused_radius_m = lc.refused_radius_m, pp.autonomous_threshold = lc.autonomous_threshold;
        pp.eps_cells = lc.eps_cells, pp.max_command_m = lc.max_command_m;
        // The mission planner behind the seam: A1's Phase 4 planner with --planner-config, else the 1.8 placeholder
        // (the ablation's row). The scene's worldgen seed is the run seed mixed into the random policy's seed.
        std::unique_ptr<live::MissionPlanner> planner;
        std::string planner_name = "placeholder_1.8";
        if (planner_cfg_path.empty()) {
            planner = std::make_unique<live::PlaceholderPlanner>(mc, brief, pp);
        } else {
            const uint64_t run_seed = YAML::LoadFile(scene + "/scene.json")["seed"].as<uint64_t>();
            std::string policy;
            const auto p4 = live::LoadPhase4Params(planner_cfg_path, mc, lc.refused_radius_m, lc.max_command_m, lc.escape_m,
                                                   run_seed, &policy);
            planner = std::make_unique<live::A1Planner>(mc, brief, p4, policy, lc.autonomous_threshold, NA);
            planner_name = "a1_phase4:" + policy;
        }
        // --- D175 item 4: the per-agent pipeline (MULTIAGENT_PIPELINE) -------------------------------------------
        // One planner, exporter and received-layers set per agent plus the operator node, all share-nothing; one
        // CollabNode per participant (coordination-only: no SLAM hooks until LIVE_COLLAB) over a seeded CommsNode.
        // Nothing here exists unless multiagent.enabled - the centralised loop above is D175's oracle row, untouched.
        namespace ac = ariadne::collaboration;
        namespace multi = planning::multi;
        struct MaState {
            ac::CommsScenario scenario;
            std::unique_ptr<ac::LinkModel> link;
            std::unique_ptr<ac::CommsNode> comms;
            std::deque<live::A1Planner> planners;                     // planner_k: agent k's OWN map and NBV
            std::deque<multi::SlabExporter> exporters;
            std::deque<multi::ReceivedLayers> layers;                 // per agent; last entry = the operator's
            std::vector<std::unique_ptr<ac::CollabNode>> nodes;       // agents 0..NA-1, operator = NA
            std::vector<std::vector<core::DecodedMessage>> mailbox;   // filled by the coordination hook during ticks
            std::vector<std::map<int, multi::TaskBid>> bids_rx;       // per node: newest bid per sender
            std::vector<std::optional<multi::TaskConsensus>> cons_rx; // per agent: newest consensus received
            std::vector<int64_t> cons_rx_t_ns;                        // and when it arrived (sim time)
            std::vector<multi::Rigid2> T_G_W;                         // known entry poses: the 2D part of T_W_O[k]
            std::vector<uint64_t> kf_count, bid_seq, patch_seq, cons_seq;   // per agent counters
            std::vector<std::optional<std::pair<uint64_t, core::SE3>>> last_kf;  // for the odometry edge
            std::unique_ptr<multi::OperatorEgress> op_egress;   // D* Lite on what the operator RECEIVED (A1 e82307f)
            // D174 (collab: true): the fleet OWNS the radio nodes; `nodes` stays empty then. The DNN runs in
            // per-agent ariadne_collab_worker processes; LightGlue verification runs in the parent (collab.Step).
            std::unique_ptr<perception::HardPairMatcher> lightglue;
            std::unique_ptr<live::CollabLive> collab;
            std::vector<pid_t> collab_pids;
            std::vector<std::string> collab_socks;
            std::ofstream edges_csv;
            std::map<ac::AgentId, bool> linked;   // the fleet's comms state, for A1's linked[k]
            std::mutex two_mu;                    // brief.T_W_O: barrier writes vs the telemetry readers
            multi::BrokerParams broker;
            bool discounted = false;                                  // broker_mode discount: R2 ApplyConsensus
            double tau_s = 10.0, timeout_s = 10.0;
            int max_options = 8;
            core::FrameTag g_tag{0, 0};                               // G = the world frame, agreed at the briefing
            // planner_k's map is touched ONLY on workers[k] (agent_worker.hpp): integration and the plan. Route reads the
            // finished plan only, so the mission thread calls it directly.
            // eager_map (demo, no shadow map): keyframes are integrated at delivery, not at the next decision.
            bool eager_map = false;
            std::atomic<size_t> rays_async{0};
            std::deque<live::AgentWorker> workers;   // LAST: destroyed (drained, joined) before the planners
        };
        std::unique_ptr<MaState> ma;
        if (lc.multiagent.enabled) {
            if (planner_cfg_path.empty()) throw std::invalid_argument("multiagent needs --planner-config (A1's planner)");
            ma = std::make_unique<MaState>();
            ma->scenario = ac::LoadCommsScenario(lc.multiagent.scenario);
            if (is_live && ma->scenario.mode != "ideal")
                throw std::invalid_argument("multiagent live: scenario mode must be ideal (RF link models need "
                                            "recorded tracks; D175 SS5 - the RF rows run on recordings)");
            ma->link = ac::MakeLinkModel(ma->scenario, {}, {});
            const uint64_t run_seed = YAML::LoadFile(scene + "/scene.json")["seed"].as<uint64_t>();
            ma->comms = std::make_unique<ac::CommsNode>(*ma->link, run_seed, core::Timestamp{0});
            ma->comms->Apply(ma->scenario);
            std::string policy;
            const auto p4 = live::LoadPhase4Params(planner_cfg_path, mc, lc.refused_radius_m, lc.max_command_m,
                                                   lc.escape_m, run_seed, &policy);
            const auto p4brief = live::Phase4BriefFrom(mc, brief);
            std::vector<ac::AgentId> fleet;
            for (int k = 0; k <= NA; ++k) fleet.push_back(ac::AgentId(k));   // NA = the operator (no vote, no bids)
            ma->mailbox.resize(size_t(NA) + 1), ma->bids_rx.resize(size_t(NA) + 1);
            ma->cons_rx.resize(size_t(NA)), ma->cons_rx_t_ns.assign(size_t(NA), INT64_MIN);
            ma->kf_count.assign(size_t(NA), 0), ma->bid_seq.assign(size_t(NA), 0), ma->patch_seq.assign(size_t(NA), 0);
            ma->cons_seq.assign(size_t(NA), 0);
            ma->last_kf.resize(size_t(NA));
            for (int k = 0; k < NA; ++k) {
                ma->planners.emplace_back(mc, brief, p4, policy, lc.autonomous_threshold, NA);
                ma->workers.emplace_back();
                ma->exporters.emplace_back(k, p4brief.grid);
                ma->layers.emplace_back(k, NA + 1, p4brief.grid);
                const core::SE3& T = brief.T_W_O[size_t(k)];
                ma->T_G_W.push_back({T.t.x(), T.t.y(), std::atan2(T.R(1, 0), T.R(0, 0))});
            }
            ma->layers.emplace_back(NA, NA + 1, p4brief.grid);   // the operator's received layers (self = NA)
            ma->op_egress = std::make_unique<multi::OperatorEgress>(p4brief.grid, brief.target, brief.exits, mc.rho_r);
            static const core::WireCodec ma_codec(core::Framing::kFragmented);
            if (!lc.collab.enabled) {   // D174: with collab the FLEET owns the radio nodes (one radio, one ledger)
                for (int k = 0; k <= NA; ++k) {
                    ac::NodeHooks h;
                    auto* mb = &ma->mailbox[size_t(k)];
                    h.coordination = [mb](const core::DecodedMessage& d, core::Timestamp) { mb->push_back(d); };
                    ma->nodes.push_back(std::make_unique<ac::CollabNode>(ac::AgentId(k), fleet, ma_codec,
                                                                         ma->comms->ledger(), std::move(h), ac::NodeParams{}));
                }
            }
            ma->broker.r_same = lc.min_goal_m;
            ma->broker.range_m = lc.multiagent.discount_range_m;
            ma->broker.mode = lc.multiagent.broker_mode == "overlap"    ? multi::BrokerMode::kOverlap
                              : lc.multiagent.broker_mode == "discount" ? multi::BrokerMode::kDiscount
                                                                        : multi::BrokerMode::kPlain;
            ma->discounted = lc.multiagent.broker_mode == "discount";
            ma->tau_s = lc.multiagent.discount_tau_s;
            ma->timeout_s = lc.multiagent.consensus_timeout_s;
            ma->max_options = lc.multiagent.max_bid_options;
            planner_name += " +multiagent(" + lc.multiagent.broker_mode + "," + ma->scenario.name + ")";
        }
        // A2's splat tee (D220): on only when the recon service listens (env from live.sh under ARIADNE_SPLAT=1).
        std::unique_ptr<SplatTee> splat;
        if (const char* sp = std::getenv("ARIADNE_RECON_SOCK")) {
            splat = std::make_unique<SplatTee>(sp, brief.T_W_O);
            if (!splat->on()) std::fprintf(stderr, "ariadne_live: splat tee: %s absent or refused; tee off\n", sp);
        }

        // The shadow map (NBV_4.3 §5 amendment, A1): a Phase 4 TSDF fed the SAME keyframes as the planner, Integrate only,
        // whose slab is written at every barrier (snapshots/p4slab_<ms>.png), so every planner row is scored from one
        // mapper, not from its own (the 1.8 row's OccupancyGrid and the TSDF differ, D110). It never decides anything.
        std::unique_ptr<planning::Phase4Planner> shadow;
        planning::TravParams shadow_trav;
        planning::GridFrame shadow_grid;
        if (!shadow_cfg_path.empty()) {
            std::string unused;
            const auto sp = live::LoadPhase4Params(shadow_cfg_path, mc, lc.refused_radius_m, lc.max_command_m, lc.escape_m, 0, &unused);
            const auto sb = live::Phase4BriefFrom(mc, brief);
            shadow = std::make_unique<planning::Phase4Planner>(sb, sp);
            shadow_trav = sp.trav, shadow_grid = sb.grid;
        }
        if (ma) ma->eager_map = lc.demo && !shadow;   // the shadow map needs Z/S at the decision: keep it inline
        const int64_t end_ns = int64_t(std::llround(lc.end_s * 1e9));
        const int64_t replan_ns = int64_t(std::llround(mc.replan_s * 1e9));
        // D166 pacing: the next sub-step is requested once consumption is within grace of the last target.
        const int64_t substep_ns = lc.lockstep_substep_ms > 0
                                       ? std::min<int64_t>(int64_t(lc.lockstep_substep_ms) * 1000000, replan_ns)
                                       : replan_ns;
        const int64_t grace_ns = 100000000;   // one camera period (20 Hz) behind the paused sim is "caught up"
        if (lc.lockstep && end_ns % replan_ns != 0) {   // the last barrier must BE the end (its holds go out paused)
            throw std::invalid_argument("lockstep: end_s must be a multiple of the replan period");
        }

        const fs::path run_dir = fs::path(out) / (brief.scene_id + (is_live ? "-live-" : "-replay-") + tools::UtcNow("%Y%m%dT%H%M%SZ"));
        fs::create_directories(run_dir / "snapshots");
        fs::create_directories(run_dir / "raw");
        Lines trace(run_dir / "trace.jsonl"), commands(run_dir / "commands.jsonl"),
            mover_state(run_dir / "mover_state.jsonl"), telemetry(run_dir / "telemetry.jsonl"),
            link_log(run_dir / "link.jsonl"), lockstep_log(run_dir / "lockstep.jsonl"),
            timing_log(run_dir / "timing.jsonl");
        // Runtime-resolved values supplement config hashes: a copied YAML alone
        // does not show defaults, overrides, or which integration owner is active.
        {
            YAML::Node stack;
            stack["schema"] = 1;
            stack["execution"] = is_live ? "live" : "replay";
            stack["demo"] = lc.demo;
            stack["lockstep"] = lc.lockstep;
            stack["agents"] = NA;
            stack["sensor_policy"] = (is_live && lc.demo && !lc.lockstep) ? "latest_stereo" : "lossless";
            stack["input_order"] = (is_live && lc.demo && !lc.lockstep) ? "arrival" : "timestamp_watermark";
            stack["mapping_owner"] = ma ? (ma->eager_map ? "per_agent_worker_at_delivery" : "per_agent_worker_at_decision")
                                         : (lc.demo ? "central_background" : "central_mission");
            stack["collaboration_enabled"] = bool(ma && lc.collab.enabled);
            stack["allocation_identity"] = "broker_and_members_v1";
            stack["chooser"] = lc.chooser;
            stack["energy_source"] = "estimated_odometry";
            stack["target_mode"] = "known_briefing_coordinate";
            stack["mission_completion_scored"] = false;
            stack["required_evidence"] = "runtime activity and mission outcome checks remain separate from configuration";
            stack["perception"] = YAML::LoadFile(va.perception_config);
            stack["slam"] = YAML::LoadFile(va.slam_config);
            stack["live"] = YAML::LoadFile(live_cfg_path);
            if (!planner_cfg_path.empty()) {
                stack["planner"] = YAML::LoadFile(planner_cfg_path);
                if (!stack["planner"]["nbv"]["cost_time"]) stack["planner"]["nbv"]["cost_time"] = "none";
                if (!stack["planner"]["nbv"]["view_yaw"]) stack["planner"]["nbv"]["view_yaw"] = "target";
            }
            std::ofstream(run_dir / "stack_manifest.yaml") << stack;
        }
        const auto wall_start = std::chrono::steady_clock::now();
        std::atomic<int64_t> live_clock_ns{-1};
        Lines control_age(run_dir / "control_age.jsonl");
        // D224 (A2's profiling ask): ARIADNE_PROFILE=1 writes profile.jsonl - one line per lockstep barrier, a
        // wall-clock decomposition of the barrier's stages. Default off; it only ADDS a file, never changes timing-
        // dependent behaviour, so digests are untouched either way.
        const bool profiling = std::getenv("ARIADNE_PROFILE") && std::string(std::getenv("ARIADNE_PROFILE")) == "1";
        std::optional<Lines> profile;
        if (profiling) profile.emplace(run_dir / "profile.jsonl");
        std::array<std::chrono::steady_clock::time_point, 9> pm{};
        const auto pmark = [&](size_t i) {
            if (profile) {
                pm[i] = std::chrono::steady_clock::now();
                for (size_t j = i + 1; j < pm.size(); ++j) pm[j] = pm[i];   // unset later stages read as 0 ms
            }
        };

        std::string profile_extra;   // more ,"key":value pairs for the next profile line (backlog, plan stages)
        const auto write_profile = [&](int64_t T, const char* kind) {
            if (!profile) return;
            pmark(8);
            const auto ms = [&](size_t i, size_t j) {
                return std::chrono::duration<double, std::milli>(pm[j] - pm[i]).count();
            };
            std::ostringstream po;
            po.precision(8);
            po << "{\"type\":\"" << kind << "\",\"t_s\":" << T * 1e-9
               << ",\"clock_ns_at_end\":" << live_clock_ns.load()
               << ",\"ms\":{\"integrate\":" << ms(0, 1)
               << ",\"radio\":" << ms(1, 2) << ",\"drain\":" << ms(2, 3) << ",\"broker\":" << ms(3, 4)
               << ",\"plan_fork\":" << ms(4, 5) << ",\"sends\":" << ms(5, 6) << ",\"merge\":" << ms(6, 7)
               << ",\"decide_command\":" << ms(7, 8) << ",\"total\":" << ms(0, 8) << "}" << profile_extra << "}";
            profile_extra.clear();
            profile->Put(po.str());
        };

        // --- topics (mission_live.json) -------------------------------------------------------------------
        const YAML::Node mj = YAML::LoadFile(scene + "/mission_live.json");
        std::map<std::string, std::pair<int, std::string>> route;   // topic -> (agent, role)
        std::map<int, std::string> cmd_topic;
        std::set<std::string> timed, left_image_topics;
        for (const auto& a : mj["agents"]) {
            const int k = a["agent"].as<int>();
            const auto t = a["topics"];
            left_image_topics.insert(t["left_image"].as<std::string>());
            route[t["left_image"].as<std::string>()] = {k, "image"};
            route[t["right_image"].as<std::string>()] = {k, "image"};
            route[t["gt_pose"].as<std::string>()] = {k, "gt"};
            route[t["imu"].as<std::string>()] = {k, "imu"};
            route[t["left_info"].as<std::string>()] = {k, "info"};
            route[t["right_info"].as<std::string>()] = {k, "info"};
            route[a["live_topics"]["mover_state"].as<std::string>()] = {k, "mover_state"};
            cmd_topic[k] = a["live_topics"]["cmd"].as<std::string>();
            // every stamped sensor stream is timed (D165): a message released on arrival (watermark already past it)
            // would land in ARRIVAL order against a same-stamp message of another topic (camera_info vs image_raw)
            for (const char* r : {"left_image", "right_image", "left_info", "right_info", "gt_pose", "imu"}) timed.insert(t[r].as<std::string>());
        }
        if (int(cmd_topic.size()) != NA) throw std::runtime_error("live: mission_live.json agents and command topics disagree");

        std::deque<AgentLane> lanes(static_cast<size_t>(NA));
        const bool latest_stereo = is_live && lc.demo && !lc.lockstep;
        // Live demo with keyframes integrated at delivery: the decision does not wait for the integration backlog and
        // its plans run ahead of it (agent_worker.hpp PostUrgent). Replay and lockstep keep the post-order barrier.
        const bool plan_ahead = latest_stereo && ma && ma->eager_map;
        Lines ingest_log(run_dir / "sensor_ingest.jsonl");
        for (const auto& a : mj["agents"]) {
            const int k = a["agent"].as<int>();
            const std::string left = latest_stereo ? a["topics"]["left_image"].as<std::string>() : "";
            const std::string right = latest_stereo ? a["topics"]["right_image"].as<std::string>() : "";
            lanes[k].images.ConfigureSensor(512ULL * 1024 * 1024, left, right);
            lanes[k].imus.ConfigureSensor(64ULL * 1024 * 1024);
            lanes[k].worker_inputs.ConfigureSensor(512ULL * 1024 * 1024, left, right);
            if (latest_stereo) {
                auto observer = [&, k](const char* kind, int64_t stamp) {
                    ingest_log.Put("{\"stage\":\"parent\",\"agent\":" + std::to_string(k) +
                        ",\"event\":\"" + std::string(kind) + "\",\"stamp_ns\":" + std::to_string(stamp) + "}");
                };
                (lc.estimator_processes ? lanes[k].worker_inputs : lanes[k].images).Observe(observer);
            }
        }
        // The mover's answers (D153b): (agent, command stamp) -> accepted. Part of the recorded input, so the wait for
        // them below is deterministic in replay.
        std::map<std::pair<int, int64_t>, bool> answers;
        std::map<std::pair<int, int64_t>, int64_t> applied_at;   // the mover's applied_ns per answer (lockstep log)
        std::set<std::pair<int, int64_t>> arrived;   // answers seen on the link (lockstep waits on these: the reorderer
                                                     // holds an answer until the world resumes, so `answers` would never fill)
        bool input_done = false;
        std::mutex progress_mu;
        std::condition_variable progress_cv;

        // --- the link (live) ------------------------------------------------------------------------------
        int fd = -1;
        std::mutex send_mu;
        if (is_live) {
            std::printf("ariadne_live: waiting for the ROS edge on %s\n", link_path.c_str());
            std::fflush(stdout);
            fd = live::ListenAndAccept(link_path);
            std::printf("ariadne_live: link up\n");
        }
        // D165 lockstep: ask gz to run to sim time t and pause there (RuntimeLink -> ControlWorld.run_to_sim_time).
        // resume_gen tells the reader the coming silence-then-burst is a requested pause, not a stalled topic.
        std::atomic<uint64_t> resume_gen{0};
        auto RunTo = [&](int64_t t) {
            if (!is_live || !lc.lockstep) return;
            ++resume_gen;
            std::lock_guard<std::mutex> lk(send_mu);
            if (!live::WriteFrame(fd, "#run_to", reinterpret_cast<const uint8_t*>(&t), sizeof t)) {
                throw std::runtime_error("live: the ROS edge closed the link");
            }
        };
        auto SendCommand = [&](int agent, const runtime::ros_cdr::PoseArray& m) {
            if (!is_live) return;
            const auto bytes = runtime::ros_cdr::EncodePoseArray(m);
            if (latest_stereo) {
                control_age.Put("{\"agent\":" + std::to_string(agent) +
                    ",\"decision_ns\":" + std::to_string(m.header.stamp.ns()) +
                    ",\"clock_ns_at_send\":" + std::to_string(live_clock_ns.load()) +
                    ",\"wall_s\":" + std::to_string(std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - wall_start).count()) + "}");
            }
            std::lock_guard<std::mutex> lk(send_mu);
            if (!live::WriteFrame(fd, cmd_topic.at(agent), bytes.data(), bytes.size())) {
                throw std::runtime_error("live: the ROS edge closed the link");
            }
        };

        // --- dispatch: every consumed message, in release order ----------------------------------------------
        live::ConsumedRecorder rec;
        if (is_live) rec.Open((run_dir / "inputs.mcap").string());
        std::atomic<bool> past_end{false};
        live::SensorHorizon sensor_horizon(size_t(NA), end_ns);
        uint64_t seq = 0;
        long dropped_untyped = 0;
        std::vector<int> worker_fd(static_cast<size_t>(NA), -1);   // D168 estimator processes; -1 = in-process
        std::vector<int64_t> last_gt_tel_ns(static_cast<size_t>(NA), -1);   // P8: GT ghost arrows, at telemetry_hz
        std::deque<std::mutex> worker_mu(static_cast<size_t>(NA));
        std::vector<pid_t> worker_pid(static_cast<size_t>(NA), 0);
        std::vector<std::thread> feeders(static_cast<size_t>(NA));
        auto Dispatch = [&](live::StreamMessage m) {
            if (past_end) return;
            const auto it = route.find(m.topic);
            if (it == route.end()) {
                ++dropped_untyped;
                return;
            }
            const auto [agent, role] = it->second;
            if (lc.demo && !lc.lockstep && (role == "image" || role == "imu")) {
                const auto stream = role == "imu" ? live::SensorHorizon::Imu
                    : left_image_topics.count(m.topic) != 0
                        ? live::SensorHorizon::Left : live::SensorHorizon::Right;
                const bool admit = sensor_horizon.Admit(size_t(agent), stream, m.stamp_ns);
                // Dispatch the last required IMU before closing queues. Images
                // past end are discarded, but a lagging agent keeps receiving.
                if (!admit) {
                    if (sensor_horizon.Complete()) {
                        past_end = true;
                        for (auto& l : lanes) l.images.Close(), l.imus.Close(), l.worker_inputs.Close();
                    }
                    return;
                }
            } else if ((role == "image" || role == "imu") && m.stamp_ns > end_ns) {
                past_end = true;
                for (auto& l : lanes) l.images.Close(), l.imus.Close(), l.worker_inputs.Close();
                return;
            }
            m.seq = ++seq;
            rec.Write(m);
            if (role == "image") {
                if (lc.estimator_processes) lanes[agent].worker_inputs.Push(std::move(m));
                else lanes[agent].images.Push(std::move(m));
            } else if (role == "imu") {
                if (lc.estimator_processes) lanes[agent].worker_inputs.Push(std::move(m));
                else lanes[agent].imus.Push(std::move(m));
            } else if (role == "gt") {
                const auto p = runtime::ros_cdr::DecodePoseStamped(m.data->data(), m.data->size());
                runtime::TimedPose tp{p.header.stamp.ns(), {}};
                tp.T.R = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                             .normalized()
                             .toRotationMatrix();
                tp.T.t = core::Vec3(p.position.x, p.position.y, p.position.z);
                lanes[agent].gt.Add(tp);
                if (last_gt_tel_ns[size_t(agent)] < 0 ||
                    tp.t_ns - last_gt_tel_ns[size_t(agent)] >= int64_t(1e9 / lc.telemetry_hz)) {
                    last_gt_tel_ns[size_t(agent)] = tp.t_ns;
                    std::ostringstream g;
                    g.precision(8);
                    g << "{\"type\":\"gt_pose\",\"t_s\":" << tp.t_ns * 1e-9 << ",\"agent\":" << agent << ",\"xy\":"
                      << Xy(tp.T.t.x(), tp.T.t.y()) << ",\"yaw\":" << std::atan2(tp.T.R(1, 0), tp.T.R(0, 0)) << "}";
                    telemetry.Put(g.str());
                }
                if (worker_fd[agent] >= 0) {   // D168: the worker keeps its own GtStore for the raw outputs
                    std::lock_guard<std::mutex> lk(worker_mu[agent]);
                    live::WriteFrame(worker_fd[agent], m.topic, m.data->data(), m.data->size());
                }
            } else if (role == "mover_state") {
                const std::string js = runtime::ros_cdr::DecodeString(m.data->data(), m.data->size());
                const auto num = [&js](const std::string& key) -> long long {
                    const auto p = js.find("\"" + key + "\":");
                    return p == std::string::npos ? -1 : std::stoll(js.substr(p + key.size() + 3));
                };
                {
                    std::lock_guard<std::mutex> lk(progress_mu);
                    answers[{agent, int64_t(num("cmd_stamp_ns"))}] = js.find("\"accepted\":true") != std::string::npos;
                    applied_at[{agent, int64_t(num("cmd_stamp_ns"))}] = int64_t(num("applied_ns"));
                }
                progress_cv.notify_all();
                mover_state.Put("{\"agent\":" + std::to_string(agent) + ",\"seq\":" + std::to_string(m.seq) +
                                ",\"state\":" + js + "}");
                telemetry.Put("{\"type\":\"mover_state\",\"agent\":" + std::to_string(agent) + ",\"state\":" + js + "}");
            }
            if (lc.demo && !lc.lockstep && sensor_horizon.Complete()) {
                past_end = true;
                for (auto& l : lanes) l.images.Close(), l.imus.Close(), l.worker_inputs.Close();
            }
        };

        std::thread reader;
        // Keep failures from the replay producer on the owning thread. An
        // exception escaping a std::thread entry point terminates the process
        // and hides the MCAP/dispatch error that caused it.
        std::exception_ptr replay_reader_error;
        long reorder_timeouts = 0, reorder_order_breaks = 0;
        // Start ingestion only after model/config validation and worker sockets
        // exist. Replay can otherwise race startup and lose early GT forwarding.
        auto start_reader = [&] {
        if (is_live) {
            reader = std::thread([&] {
                // D166: the wall-clock timeout skip made release order hardware-dependent under lockstep
                // (~80 skips per 200 s run, at the barriers). Lockstep guarantees every timed topic reaches the
                // watermark, so the hold is effectively infinite there; free-run keeps the configured hold.
                live::Reorderer ro(timed, std::chrono::milliseconds(lc.lockstep ? 86400000 : lc.hold_back_ms), latest_stereo);
                live::LinkFrame f;
                uint64_t seen_gen = 0;
                try {
                    while (live::ReadFrame(fd, f)) {
                        if (f.topic[0] == '#') {
                            const std::string payload(f.payload.begin(), f.payload.end());
                            link_log.Put("{\"control\":\"" + f.topic + "\",\"payload\":" + payload + "}");
                            if (f.topic == "#clock") live_clock_ns = YAML::Load(payload)["sim_ns"].as<int64_t>();
                            if (f.topic == "#ready") RunTo(substep_ns);   // lockstep: the world starts at the runtime's word (D166: one sub-step)
                            if (f.topic == "#run_to_ack") lockstep_log.Put("{\"ack\":" + payload + "}");
                            continue;
                        }
                        if (const uint64_t g = resume_gen.load(); g != seen_gen) {
                            seen_gen = g;
                            ro.Refresh();   // the silence before this burst was a requested pause, not a stall
                        }
                        live::StreamMessage m;
                        m.topic = f.topic;
                        const auto it = route.find(f.topic);
                        const std::string role = it == route.end() ? "" : it->second.second;
                        if (role == "mover_state" && lc.lockstep) {
                            const std::string js = runtime::ros_cdr::DecodeString(f.payload.data(), f.payload.size());
                            const auto q = js.find("\"cmd_stamp_ns\":");
                            if (q != std::string::npos) {
                                std::lock_guard<std::mutex> lk(progress_mu);
                                arrived.insert({it->second.first, std::stoll(js.substr(q + 15))});
                                progress_cv.notify_all();
                            }
                        }
                        m.type = role == "image" ? "sensor_msgs/msg/Image"
                                 : role == "gt" ? "geometry_msgs/msg/PoseStamped"
                                 : role == "imu" ? "sensor_msgs/msg/Imu"
                                 : role == "info" ? "sensor_msgs/msg/CameraInfo"
                                 : role == "mover_state" ? "std_msgs/msg/String" : "unknown";
                        auto data = std::make_shared<std::vector<uint8_t>>(std::move(f.payload));
                        m.stamp_ns = live::PeekStamp(m.type, *data);
                        m.data = std::move(data);
                        ro.Push(std::move(m), Dispatch);
                        if (past_end) break;
                    }
                    ro.Flush(Dispatch);
                } catch (const std::exception& e) {
                    replay_reader_error = std::current_exception();
                    std::fprintf(stderr, "ariadne_live: link reader: %s\n", e.what());
                }
                reorder_timeouts = ro.released_on_timeout();
                reorder_order_breaks = ro.released_out_of_order();
                for (auto& l : lanes) l.images.Close(), l.imus.Close(), l.worker_inputs.Close();
                {
                    std::lock_guard<std::mutex> lk(progress_mu);
                    input_done = true;
                }
                progress_cv.notify_all();
            });
        } else {
            reader = std::thread([&] {
                try {
                    runtime::RosbagReader bag;
                    bag.Open(replay_path);
                    const auto types = bag.topics();
                    bag.Replay([&](const runtime::RosMessage& msg) {
                        live::StreamMessage m;
                        m.topic = std::string(msg.topic);
                        const auto type = types.find(m.topic);
                        if (type == types.end()) throw std::runtime_error("rosbag: missing topic schema for " + m.topic);
                        m.type = type->second;
                        auto data = std::make_shared<std::vector<uint8_t>>(msg.data, msg.data + msg.size);
                        m.stamp_ns = live::PeekStamp(m.type, *data);
                        m.data = std::move(data);
                        Dispatch(std::move(m));
                    });
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "ariadne_live: replay input reader: %s\n", e.what());
                    replay_reader_error = std::current_exception();
                } catch (...) {
                    std::fprintf(stderr, "ariadne_live: replay input reader: unknown exception\n");
                    replay_reader_error = std::current_exception();
                }
                for (auto& l : lanes) l.images.Close(), l.imus.Close(), l.worker_inputs.Close();
                {
                    std::lock_guard<std::mutex> lk(progress_mu);
                    input_done = true;
                }
                progress_cv.notify_all();
            });
        }

        };

        // --- per-agent estimators: the replay path, unchanged ---------------------------------------------------
        std::vector<std::thread> vo(static_cast<size_t>(NA));
        std::vector<std::unique_ptr<live::LiveSimSource>> sources(static_cast<size_t>(NA));
        std::mutex estimator_turn;   // D152a: one frame at a time through the estimators (in-process mode only)
        struct TimeAgg {   // D167: per-agent frame timing, for results.yaml (the raw series is timing.jsonl)
            std::mutex mu;
            long n = 0;
            double wait = 0, lock = 0, track = 0, max_track = 0, max_lock = 0;
        };
        std::deque<TimeAgg> tagg(static_cast<size_t>(NA));
        // Lane plumbing shared by both estimator arrangements (D168): in-process threads behind the D152a turn lock,
        // or one worker process per agent, each with its own OpenCV pool.
        auto deliver_timing = [&](int k, int64_t t_ns, double w, double l, double tr) {
            {
                std::lock_guard<std::mutex> g(tagg[k].mu);
                ++tagg[k].n, tagg[k].wait += w, tagg[k].lock += l, tagg[k].track += tr;
                tagg[k].max_track = std::max(tagg[k].max_track, tr);
                tagg[k].max_lock = std::max(tagg[k].max_lock, l);
            }
            std::ostringstream o;
            o.precision(6);
            o << "{\"agent\":" << k << ",\"t_s\":" << t_ns * 1e-9 << ",\"wait_s\":" << w << ",\"lock_s\":" << l
              << ",\"track_s\":" << tr << ",\"clock_ns_at_receive\":" << live_clock_ns.load()
              << ",\"wall_s\":" << std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count() << "}";
            timing_log.Put(o.str());
        };
        auto deliver_kf = [&](int k, KfRec r, const cv::Mat& rgb) {
            {   // P8: the camera window - the latest keyframe as /snapshots/cam_<k>.jpg (the console polls it)
                cv::Mat bgr;
                if (rgb.channels() == 3) cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                else bgr = rgb;
                cv::imwrite((run_dir / "snapshots" / ("cam_" + std::to_string(k) + ".jpg")).string(), bgr,
                            {cv::IMWRITE_JPEG_QUALITY, 80});
                // The feature overlay: the observations the estimator recorded at this keyframe, as pixel
                // coordinates in the image just written (full-res rectified left). The console draws them.
                std::ostringstream j;
                j << "{\"t_ns\":" << r.t_ns << ",\"w\":" << bgr.cols << ",\"h\":" << bgr.rows << ",\"pts\":[";
                j.setf(std::ios::fixed), j.precision(1);
                for (size_t i = 0; i < r.track_px.size(); ++i)
                    j << (i ? "," : "") << '[' << r.track_px[i].x << ',' << r.track_px[i].y << ']';
                j << "]}";
                std::ofstream((run_dir / "snapshots" / ("cam_" + std::to_string(k) + ".json")).string()) << j.str();
            }
            if (splat) splat->OnKf(r, rgb);   // D220: bounded, drop-oldest, never blocks this thread
            if (ma && ma->eager_map) {   // demo: integrate now on worker_k; the mission keeps only the bookkeeping
                core::SE3 T_W_O;
                {
                    std::lock_guard<std::mutex> lk2(ma->two_mu);
                    T_W_O = brief.T_W_O[size_t(k)];
                }
                const live::KeyframeInput ki{r.agent, r.id, r.t_ns, T_W_O * r.T_O_C, std::move(r.Z), std::move(r.S),
                                             r.fx, r.fy, r.cx, r.cy};
                ma->workers[size_t(k)].Post([m = ma.get(), k, ki] {
                    m->rays_async += m->planners[size_t(k)].Integrate(ki);
                });
                r.mapped = true;
            }
            AgentLane& L = lanes[k];
            std::lock_guard<std::mutex> lk(L.mu);
            L.kf_times.push_back(r.t_ns);
            L.last_rgb = rgb.clone();
            L.kfs.push_back(std::move(r));
            ++L.keyframes;
        };
        auto deliver_frame = [&](int k, int64_t t_ns, const core::SE3& T_O_C) {
            AgentLane& L = lanes[k];
            {
                std::lock_guard<std::mutex> lk(L.mu);
                L.cams.emplace_back(t_ns, T_O_C);
                ++L.frames;
            }
            {
                std::lock_guard<std::mutex> lk(progress_mu);
                L.progress = t_ns;
            }
            progress_cv.notify_all();
            if (L.last_telemetry_ns < 0 || t_ns - L.last_telemetry_ns >= int64_t(1e9 / lc.telemetry_hz)) {
                L.last_telemetry_ns = t_ns;
                core::SE3 Two = brief.T_W_O[size_t(k)];
                if (ma && ma->collab) {   // D174 feedback moves it at barriers; this thread only READS
                    std::lock_guard<std::mutex> g(ma->two_mu);
                    Two = brief.T_W_O[size_t(k)];
                }
                const core::SE3 T = Two * T_O_C;
                std::ostringstream o;
                o.precision(8);
                o << "{\"type\":\"pose\",\"t_s\":" << t_ns * 1e-9 << ",\"agent\":" << k << ",\"xy\":"
                  << Xy(T.t.x(), T.t.y()) << ",\"yaw\":" << YawOfCamera(T) << "}";
                telemetry.Put(o.str());
            }
        };
        auto lane_done = [&](int k, size_t unpaired) {
            {
                std::lock_guard<std::mutex> lk(progress_mu);
                lanes[k].done = true;
                lanes[k].unpaired = unpaired;
            }
            progress_cv.notify_all();
        };
        // --- D174: the live collaboration (collab: true) ----------------------------------------------------------
        // One ariadne_collab_worker per agent (the DNN: EigenPlaces + lazy ALIKED, its own pinned OpenCV pool);
        // the parent runs CollabLive: the fleet owns the radio nodes (customize installs the D175 mailboxes),
        // LightGlue verification runs inside Step. The estimator workers tee (#ckin/#crel/#cprog) and never touch
        // a DNN. collab: false = none of this exists (D168 digests unchanged).
        uint64_t collab_config_digest = 0;              // the consumed collab config BYTES (collab60d lesson)
        std::string collab_config_path_used;
        if (ma && lc.collab.enabled) {
            if (!lc.estimator_processes)
                throw std::invalid_argument("collab: true requires estimator_processes: true (the tee lives in the worker)");
            // The collab config is consumed AS BYTES from the run dir, never from the repo at replay time
            // (collab60d's 2b81c1 lesson: the repo file had drifted - the D178 gate was enabled after the
            // recording - so the replay could never match its live run). Live: snapshot the file beside the
            // recording. Replay: the recording's snapshot is authoritative when present (older recordings
            // predate it and still read the configured path, recorded as such in the manifest).
            std::string ccfg_path = lc.collab.config;
            if (is_live) {
                const fs::path snap = run_dir / "collab_config.yaml";
                std::ofstream(snap, std::ios::binary) << std::ifstream(lc.collab.config, std::ios::binary).rdbuf();
                ccfg_path = snap.string();
            } else {
                const fs::path snap = fs::path(replay_path).parent_path() / "collab_config.yaml";
                if (fs::exists(snap)) ccfg_path = snap.string();
            }
            {
                std::ifstream cf(ccfg_path, std::ios::binary);
                std::ostringstream cb;
                cb << cf.rdbuf();
                core::Fnv1a64 ch;
                for (const char c : cb.str()) ch.AbsorbByte(uint8_t(c));
                collab_config_digest = ch.value();
                collab_config_path_used = ccfg_path;
            }
            const auto ccfg = runtime::LoadCollabConfig(ccfg_path);
            if (ccfg.comms_scenario != lc.multiagent.scenario)
                throw std::invalid_argument("collab config comms_scenario (" + ccfg.comms_scenario +
                                            ") != multiagent.scenario (" + lc.multiagent.scenario + "): ONE radio");
            perception::HardPairParams hp;
            hp.aliked_onnx = lc.collab.aliked_onnx;
            hp.lightglue_onnx = lc.collab.lightglue_onnx;
            hp.width = 1280, hp.height = 720;
            hp.backend = lc.collab.dnn_backend;
            ma->lightglue = std::make_unique<perception::HardPairMatcher>(hp);
            for (const auto& r : ma->lightglue->Records())   // the backend that actually runs (log which ran)
                std::fprintf(stderr, "ariadne_live: hard-pair %s via %s\n", r.model.c_str(), r.engine.c_str());
            ma->edges_csv.open((run_dir / "collab_edges.csv").string());
            ma->edges_csv << "agent_a,kf_a,agent_b,kf_b,score,matches,essential_inliers,pnp_inliers,reason,"
                             "gt_err_t_m,gt_err_r_deg,gt_baseline_m,pose_err_deg\n";
            std::vector<ac::AgentId> fleet_ids;
            for (int k2 = 0; k2 <= NA; ++k2) fleet_ids.push_back(ac::AgentId(k2));
            live::CollabLiveParams cp;   // D179: async; the event log is the reproducibility substrate
            if (is_live) cp.log_path = (run_dir / "collab_events.jsonl").string();
            else cp.replay_log = (fs::path(replay_path).parent_path() / "collab_events.jsonl").string();
            {   // 7aa8a67: the log's identity line - replay refuses a log whose config bytes or build differ
                char hx[17];
                std::snprintf(hx, sizeof hx, "%016llx", (unsigned long long)collab_config_digest);
                const char* gc = std::getenv("ARIADNE_GIT_COMMIT");
                cp.identity = std::string(hx) + ":" + (gc && *gc ? gc : "unknown");
            }
            const uint64_t cseed = YAML::LoadFile(scene + "/scene.json")["seed"].as<uint64_t>();
            ma->collab = std::make_unique<live::CollabLive>(
                ccfg, fleet_ids, *ma->comms, *ma->lightglue, cseed, /*t_first_ns=*/0, cp, &ma->edges_csv,
                [&](ac::AgentId ck, ac::NodeHooks& h) {
                    auto* mb = &ma->mailbox[size_t(ck)];
                    h.coordination = [mb](const core::DecodedMessage& d, core::Timestamp) { mb->push_back(d); };
                });
            const std::string wt = std::to_string(lc.collab.worker_threads);
            const std::string ks = std::to_string(int(lc.collab.keep_s));
            const std::string cbin = (fs::path(argv[0]).parent_path() / "ariadne_collab_worker").string();
            for (int k2 = 0; k2 < NA; ++k2) {
                // /tmp, not the run dir: sun_path is 108 bytes (D168's replay lesson)
                const std::string sock =
                    "/tmp/ariadne-collab-" + std::to_string(::getpid()) + "-" + std::to_string(k2) + ".sock";
                ma->collab_socks.push_back(sock);
                const std::string ka2 = std::to_string(k2);
                // collab60d lesson: a worker's death reason must land on disk (live AND replay), not in a lost
                // pipe - its stderr goes to the run dir. Opened in the parent; only the dup2 sits between fork
                // and exec (async-signal-safe).
                const int werr = ::open((run_dir / ("collab_worker_" + ka2 + ".err")).c_str(),
                                        O_CREAT | O_WRONLY | O_APPEND, 0644);
                const pid_t pid = ::fork();
                if (pid == 0) {   // child: only the stderr dup between fork and exec
                    if (werr >= 0) {
                        ::dup2(werr, 2);
                        ::close(werr);
                    }
                    const char* cas[] = {cbin.c_str(),           "--socket",         sock.c_str(),
                                         "--agent",              ka2.c_str(),        "--place-onnx",
                                         lc.collab.place_onnx.c_str(),               "--place-size",
                                         lc.collab.place_size.c_str(),               "--aliked-onnx",
                                         lc.collab.aliked_onnx.c_str(),              "--lightglue-onnx",
                                         lc.collab.lightglue_onnx.c_str(),           "--threads",
                                         wt.c_str(),             "--keep-s",         ks.c_str(),
                                         "--dnn-backend",        lc.collab.dnn_backend.c_str(),
                                         nullptr};
                    ::execv(cas[0], const_cast<char**>(cas));
                    std::_Exit(127);
                }
                if (werr >= 0) ::close(werr);
                ma->collab_pids.push_back(pid);
            }
            for (int k2 = 0; k2 < NA; ++k2)
                ma->collab->Attach(ac::AgentId(k2), live::Connect(ma->collab_socks[size_t(k2)], std::chrono::seconds(60)));
            {   // D178 entry gate: the known entry poses, set before the fleet thread processes anything
                std::map<ac::AgentId, core::SE3> ep;
                for (int k2 = 0; k2 < NA; ++k2) ep[ac::AgentId(k2)] = brief.T_W_O[size_t(k2)];
                ma->collab->SetEntryPoses(ep);   // queued onto the fleet thread (the direct fleet() call raced it)
            }
        }
        for (int k = 0; k < NA; ++k) {
            if (lc.estimator_processes) {   // D168: spawn the worker, feed its images in consumed order, read back
                // /tmp, not the run dir: a nested replay run dir overflows sun_path (108 bytes; replay1, D168)
                const std::string wsock = "/tmp/ariadne-worker-" + std::to_string(::getpid()) + "-" + std::to_string(k) + ".sock";
                const std::string raw_out = (run_dir / "raw").string();
                const std::string ka = std::to_string(k), sa = std::to_string(va.seed);
                const std::string lensa = lc.lens ? "on" : "off";   // D227: the toggle reaches the worker's source
                const std::string csock = ma && ma->collab ? ma->collab_socks[size_t(k)] : "";   // D174: the tee's target
                const pid_t pid = ::fork();
                if (pid == 0) {   // child: exec ourselves in worker mode, nothing else between fork and exec
                    std::vector<const char*> cas = {argv[0],
                                                    "--scene", scene.c_str(),
                                                    "--perception-config", va.perception_config.c_str(),
                                                    "--slam-config", va.slam_config.c_str(),
                                                    "--out", raw_out.c_str(),
                                                    "--seed", sa.c_str(),
                                                    "--estimator-worker", ka.c_str(),
                                                    "--worker-socket", wsock.c_str(),
                                                    "--lens", lensa.c_str(),
                                                    "--ingest-policy", latest_stereo ? "latest_stereo" : "lossless"};
                    if (!va.imu_noise_config.empty()) cas.insert(cas.end(), {"--imu-noise-config", va.imu_noise_config.c_str()});
                    if (!csock.empty()) cas.insert(cas.end(), {"--collab-socket", csock.c_str()});
                    cas.push_back(nullptr);
                    ::execv(cas[0], const_cast<char**>(cas.data()));
                    std::_Exit(127);
                }
                worker_pid[k] = pid;
                worker_fd[k] = live::ListenAndAccept(wsock);
                ::unlink(wsock.c_str());
                feeders[k] = std::thread([&, k] {
                    while (auto m = lanes[k].worker_inputs.Pop()) {
                        std::lock_guard<std::mutex> lk(worker_mu[k]);
                        if (!live::WriteFrame(worker_fd[k], m->topic, m->data->data(), m->data->size())) break;
                    }
                    ::shutdown(worker_fd[k], SHUT_WR);   // end of input: the worker finishes and answers #done
                });
                vo[k] = std::thread([&, k] {
                    live::LinkFrame f;
                    size_t unpaired = 0;
                    while (live::ReadFrame(worker_fd[k], f)) {
                        const uint8_t* q = f.payload.data();
                        if (f.topic == "#frame") {
                            if (k == test_delay_lane && test_delay_ms > 0)   // D225 guard: timing only, never data
                                std::this_thread::sleep_for(std::chrono::milliseconds(test_delay_ms));
                            const int64_t t = GetPod<int64_t>(q);
                            deliver_frame(k, t, GetSe3(q));
                        } else if (f.topic == "#kf") {
                            KfRec r;
                            r.agent = k;
                            r.id = GetPod<uint64_t>(q), r.t_ns = GetPod<int64_t>(q);
                            r.T_O_C = GetSe3(q);
                            r.fx = GetPod<double>(q), r.fy = GetPod<double>(q);
                            r.cx = GetPod<double>(q), r.cy = GetPod<double>(q);
                            r.Z = GetMat(q), r.S = GetMat(q);
                            const cv::Mat rgb = GetMat(q);
                            const uint32_t tn = GetPod<uint32_t>(q);
                            const std::string training(reinterpret_cast<const char*>(q), tn);   // "none" until ph5-dve
                            q += tn;
                            if (training != "none") { GetMat(q); GetMat(q); }   // albedo + valid: not consumed here yet
                            const cv::Mat px = GetMat(q);   // 1xN CV_32FC2 tracked observations
                            if (!px.empty())
                                r.track_px.assign(px.ptr<cv::Point2f>(), px.ptr<cv::Point2f>() + px.total());
                            deliver_kf(k, std::move(r), rgb);
                        } else if (f.topic == "#timing") {
                            const int64_t t = GetPod<int64_t>(q);
                            const double w = GetPod<double>(q), l = GetPod<double>(q), tr = GetPod<double>(q);
                            deliver_timing(k, t, w, l, tr);
                        } else if (f.topic == "#done") {
                            unpaired = size_t(GetPod<uint64_t>(q));
                        }
                    }
                    lane_done(k, unpaired);
                });
                continue;
            }
            sources[k] = std::make_unique<live::LiveSimSource>(scene, k, lanes[k].images, lanes[k].imus, lanes[k].gt, estimator_turn,
                                                               "live", lc.lens);
            sources[k]->on_timing = [&, k](int64_t t_ns, double w, double l, double tr) { deliver_timing(k, t_ns, w, l, tr); };
            vo[k] = std::thread([&, k] {
                tools::StereoVoArgs a = va;
                a.out = (run_dir / "raw").string();
                a.tool = std::string("ariadne_live/agent") + std::to_string(k);
                a.late_ground_truth = [&L = lanes[k]] { return L.gt.Snapshot(); };
                a.on_keyframe = [&, k](const tools::KeyframeView& kv) {
                    const runtime::KeyframeBundle& b = kv.bundle;
                    KfRec r;
                    r.agent = k;
                    r.id = kv.keyframe.id;
                    r.t_ns = kv.keyframe.t_ns;
                    r.T_O_C = b.T_WiC;
                    r.Z = HalfAsSpool(runtime::MaskedDepth32(b.depth, b.mask));
                    r.S = HalfAsSpool(runtime::MaskedDepth32(b.depth_sigma, b.mask));
                    // Spool camera: u' + 0.5 = 0.5 (u + 0.5).
                    r.fx = b.K.fx * 0.5;
                    r.fy = b.K.fy * 0.5;
                    r.cx = 0.5 * (b.K.cx + 0.5) - 0.5;
                    r.cy = 0.5 * (b.K.cy + 0.5) - 0.5;
                    r.track_px = kv.track_px;
                    deliver_kf(k, std::move(r), b.rgb);
                };
                a.on_frame = [&, k](const tools::FrameView& fv) { deliver_frame(k, fv.t_ns, fv.T_O_C); };
                try {
                    tools::RunStereoVo(*sources[k], a, argc, argv);
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "ariadne_live: agent %d estimator: %s\n", k, e.what());
                }
                lane_done(k, sources[k]->unpaired());
            });
        }

        // --- the mission: barrier -> integrate -> replan -> candidates -> choose -> command ----------------------
        std::unique_ptr<ChooserLink> chooser;
        if (lc.chooser == "llm" && !chooser_socket.empty()) {   // live; or replay against a test orchestrator
            fs::create_directories(run_dir / "approvals");
            chooser = std::make_unique<ChooserLink>(chooser_socket);
        }
        int replans = 0, commands_sent = 0, answer_timeouts = 0, commands_expired = 0;
        // Operator directives (AGENTIC_7 §0b): recorded inputs. Live: <run>/directives_inbox/*.json, taken at the
        // replan barrier in name order, stamped and appended to directives.jsonl. Replay: that file, verbatim.
        std::vector<mis::Directive> directives;
        if (!directives_path.empty()) {
            std::ifstream f(directives_path);
            for (std::string line; std::getline(f, line);) {
                if (!line.empty()) directives.push_back(mis::ParseDirective(YAML::Load(line)));
            }
        }
        if (is_live) fs::create_directories(run_dir / "directives_inbox");
        Lines directives_log(run_dir / "directives.jsonl"), directives_rejected(run_dir / "directives_rejected.jsonl");
        if (!is_live) for (const auto& d : directives) directives_log.Put(mis::DirectiveJson(d));
        std::vector<size_t> integrated(static_cast<size_t>(NA), 0);
        size_t rays = 0;
        std::vector<std::pair<int, std::pair<double, double>>> refused_goals;   // (agent, goal xy): never re-chosen
        std::map<int, std::pair<double, double>> last_goal;                      // per agent, the last commanded goal
        // Per agent, the task its last command serves (AGENTIC_7 D164): the chooser's memory across replans. A new pick
        // whose goal lies within refused_radius_m of it CONTINUES it (since_ns kept); anything else starts a new one.
        struct CurrentTask {
            bool set = false;
            double gx = 0, gy = 0;
            int64_t since_ns = 0;
            std::string reason;   // the chooser's words when it was picked (LLM request only: replay has no answer text)
        };
        std::vector<CurrentTask> current(static_cast<size_t>(NA));
        // D169 demo: keyframes integrate here as they arrive, serialized against Plan by planner_mu. A1
        // (2026-10-23): Plan carries incremental egress state (D* Lite), so there is ONE Plan caller and the call
        // sequence is never reordered; per-agent keyframe order stays time order (each lane already is, and the
        // (t, agent, id) sort keeps it); cross-agent interleaving makes the demo map not bit-reproducible, which is
        // accepted - demo is not the evidence substrate.
        std::mutex planner_mu;
        std::atomic<bool> integ_stop{false};
        std::thread integrator;
        if (lc.demo && !ma) {
            integrator = std::thread([&] {
                while (!integ_stop) {
                    std::vector<KfRec> batch;
                    for (auto& L : lanes) {
                        std::lock_guard<std::mutex> lk(L.mu);
                        while (!L.kfs.empty()) {
                            batch.push_back(std::move(L.kfs.front()));
                            L.kfs.pop_front();
                        }
                    }
                    if (batch.empty()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        continue;
                    }
                    std::sort(batch.begin(), batch.end(), [](const KfRec& a, const KfRec& b) {
                        return std::tie(a.t_ns, a.agent, a.id) < std::tie(b.t_ns, b.agent, b.id);
                    });
                    std::lock_guard<std::mutex> lk(planner_mu);
                    for (const KfRec& r : batch) {
                        const live::KeyframeInput ki{r.agent, r.id, r.t_ns, brief.T_W_O[size_t(r.agent)] * r.T_O_C,
                                                     r.Z, r.S, r.fx, r.fy, r.cx, r.cy};
                        rays += planner->Integrate(ki);
                        ++integrated[r.agent];
                    }
                }
            });
        }
        std::string demo_prev_did;                                 // D169: the horizon whose answer we poll next
        std::vector<std::vector<std::pair<double, double>>> demo_prev_goals(static_cast<size_t>(NA));   // its candidate goals
        int64_t radio_tick_ns = 0;
        int64_t watchdog_progress = -1;
        auto watchdog_wall = std::chrono::steady_clock::now();
        int64_t paced_to = substep_ns;   // the last run_to target issued (the reader sent it at #ready)
        // A mission-loop failure must be LOUD and must not unwind past the joinable lane threads (collab60d:
        // a replay arrival-wait runtime_error reached main, the std::thread members' dtors called terminate
        // 'without an active exception', and e.what() was lost). Caught here, printed, then the one teardown
        // path below runs (joins, Shutdown, rec.Close, results.yaml) and the process still exits nonzero.
        std::exception_ptr mission_err;
        start_reader();
        try {
        for (int64_t T = replan_ns; T <= end_ns; T += replan_ns) {
            {
                std::unique_lock<std::mutex> lk(progress_mu);
                // EVERY lane (D225: these checks were hardcoded to lanes 0 and 1 after the D213 N-lift - the
                // ma200 replay divergence; the predicates live in lockstep_gate.hpp under a unit test now).
                for (;;) {   // evidence mode paces every lane; demo advances on fresh available state
                    if (latest_stereo) {
                        int64_t newest = -1;
                        for (const auto& lane : lanes) newest = std::max(newest, lane.progress);
                        if (newest > watchdog_progress) {
                            watchdog_progress = newest;
                            watchdog_wall = std::chrono::steady_clock::now();
                        } else if (std::chrono::steady_clock::now() - watchdog_wall > std::chrono::seconds(120)) {
                            throw std::runtime_error("demo watchdog: no estimator progress for 120 wall seconds");
                        }
                    }
                    if ((latest_stereo ? live::AnyReached(lanes, T) : live::AllReached(lanes, T)) ||
                        live::AllDoneShort(lanes, T)) break;
                    if (is_live && lc.lockstep && paced_to < T && !input_done) {
                        if (live::MinProgress(lanes) >= paced_to - grace_ns) {
                            const int64_t next = std::min(paced_to + substep_ns, T);
                            lk.unlock();
                            RunTo(next);
                            lk.lock();
                            paced_to = next;
                            continue;
                        }
                    }
                    progress_cv.wait_for(lk, std::chrono::milliseconds(200));
                }
                if (latest_stereo) {
                    int64_t newest = T;
                    for (const auto& lane : lanes) newest = std::max(newest, lane.progress);
                    T = std::min(end_ns, std::max(T, (newest / replan_ns) * replan_ns));
                }
                if (live::AllDoneShort(lanes, T)) break;   // every estimator ended before T: the mission is over
                // The previous decision's outcome is an input to this one: wait for EVERY agent's answer to the commands sent
                // at T - replan (live: bounded wait, a timeout is counted; replay: they are in the recording).
                if (T > replan_ns && !latest_stereo) {
                    const auto have = [&] {
                        if (input_done) return true;
                        for (int k = 0; k < NA; ++k) if (!answers.count({k, T - replan_ns})) return false;
                        return true;
                    };
                    if (is_live) {
                        if (!progress_cv.wait_for(lk, std::chrono::seconds(30), have)) ++answer_timeouts;
                    } else {
                        progress_cv.wait(lk, have);
                    }
                    for (int k = 0; k < NA; ++k) {
                        const auto a = answers.find({k, T - replan_ns});
                        if (a != answers.end() && !a->second && last_goal.count(k)) refused_goals.push_back({k, last_goal[k]});
                    }
                }
            }
            const auto barrier_wall = std::chrono::steady_clock::now();
            pmark(0);
            profile_extra.clear();
            if (live::IntegrateAtDecision(lc.demo, bool(ma))) {   // per-agent maps are owned by this mission thread; demo must feed them too
                std::vector<KfRec> batch;
                for (auto& L : lanes) {
                    std::lock_guard<std::mutex> lk(L.mu);
                    while (!L.kfs.empty() && L.kfs.front().t_ns < T) {
                        batch.push_back(std::move(L.kfs.front()));
                        L.kfs.pop_front();
                    }
                }
                std::sort(batch.begin(), batch.end(), [](const KfRec& a, const KfRec& b) {
                    return std::tie(a.t_ns, a.agent, a.id) < std::tie(b.t_ns, b.agent, b.id);
                });
                for (const KfRec& r : batch) {
                    const live::KeyframeInput ki{r.agent, r.id, r.t_ns, brief.T_W_O[size_t(r.agent)] * r.T_O_C,
                                                 r.Z, r.S, r.fx, r.fy, r.cx, r.cy};
                    if (ma) {
                        // D175: agent k's keyframes reach ONLY planner_k (its own map, in W = G under known entry
                        // poses) and its own CollabNode (radio pose in W_self = O_k; the odometry edge is carried
                        // for the record - coordination-only mode runs no SLAM on it).
                        const auto k = size_t(r.agent);
                        // On worker_k, in batch order: agents integrate in parallel, each in the old sequence.
                        if (!r.mapped)
                            ma->workers[k].Post([m = ma.get(), k, ki] { m->rays_async += m->planners[k].Integrate(ki); });
                        if (!ma->collab) {   // D174: with the fleet, keyframes enter ONLY via Release (#ckf)
                            std::optional<ac::OdometryEdge> odo;
                            if (ma->last_kf[k]) odo = ac::OdometryEdge{ma->last_kf[k]->first,
                                                                       ma->last_kf[k]->second.inverse() * r.T_O_C,
                                                                       core::Mat6::Identity()};
                            ma->nodes[k]->AddKeyframe(r.id, r.t_ns, r.T_O_C, odo, {}, core::Timestamp{r.t_ns});
                        }
                        ma->last_kf[k] = {r.id, r.T_O_C};
                        ++ma->kf_count[k];
                    } else {
                        rays += planner->Integrate(ki);
                    }
                    if (shadow) shadow->Integrate(ki.agent, ki.t_ns, live::ToDepthFrame(ki));
                    ++integrated[r.agent];
                }
                if (ma && plan_ahead) {   // live demo: plan on the map as it stands; the backlog keeps integrating
                    if (profile) {   // keyframes delivered but not yet integrated, per agent
                        std::ostringstream bl;
                        bl << ",\"backlog\":[";
                        for (size_t k = 0; k < ma->workers.size(); ++k) bl << (k ? "," : "") << ma->workers[k].Pending();
                        bl << "]";
                        profile_extra += bl.str();
                    }
                    rays += ma->rays_async.exchange(0);
                } else if (ma) {   // the decision sees every keyframe before T (and, demo, whatever was delivered since)
                    for (auto& w : ma->workers) w.Wait();
                    rays += ma->rays_async.exchange(0);
                }
            }
            pmark(1);   // integrate done
            if (is_live) {
                std::vector<fs::path> inbox;
                for (const auto& e : fs::directory_iterator(run_dir / "directives_inbox")) {
                    if (e.path().extension() == ".json") inbox.push_back(e.path());
                }
                std::sort(inbox.begin(), inbox.end());
                for (const auto& f : inbox) {
                    try {
                        YAML::Node n = YAML::LoadFile(f.string());
                        n["effective_ns"] = T;
                        const mis::Directive d = mis::ParseDirective(n);
                        directives.push_back(d);
                        directives_log.Put(mis::DirectiveJson(d));
                        telemetry.Put("{\"type\":\"directive\",\"t_s\":" + std::to_string(T * 1e-9) + ",\"directive\":" + mis::DirectiveJson(d) + "}");
                        fs::rename(f, fs::path(f.string() + ".applied"));
                    } catch (const std::exception& e) {
                        directives_rejected.Put("{\"file\":\"" + JsonEscape(f.filename().string()) + "\",\"t_ns\":" +
                                                std::to_string(T) + ",\"error\":\"" + JsonEscape(e.what()) + "\"}");
                        fs::rename(f, fs::path(f.string() + ".rejected"));
                    }
                }
            }
            const std::vector<mis::Directive> active = mis::ActiveDirectives(directives, T);
            live::PlanContext ctx(NA);
            ctx.T = T;
            ctx.directives = active;
            ctx.refused_goals = refused_goals;
            for (int k = 0; k < NA; ++k) {
                std::lock_guard<std::mutex> lk(lanes[size_t(k)].mu);
                for (auto it = lanes[k].cams.rbegin(); it != lanes[k].cams.rend(); ++it) {
                    if (it->first < T) {
                        if (latest_stereo && T - it->first > replan_ns) break;
                        const core::SE3 Tw = brief.T_W_O[size_t(k)] * it->second;
                        ctx.active[k] = true;
                        ctx.at[k] = brief.ToCell(Tw.t.x(), Tw.t.y());
                        ctx.position[k] = Tw.t;
                        ctx.yaw[size_t(k)] = YawOfCamera(Tw);   // D117: C_time starts from the current heading
                        break;
                    }
                }
            }
            for (int k = 0; k < NA; ++k) {
                std::lock_guard<std::mutex> lk(lanes[size_t(k)].mu);
                const auto& poses = lanes[size_t(k)].cams;
                double travelled = 0;
                for (size_t j = 1; j < poses.size() && poses[j].first < T; ++j)
                    travelled += (poses[j].second.t - poses[j - 1].second.t).norm();
                ctx.flown_m[size_t(k)] = travelled;
            }
            live::PlanOutput plan_out;
            std::vector<multi::Pick> ma_picks;
            if (ma) {
                // The radio node of participant k: the fleet's with collab (one radio, one ledger), else ours.
                const auto ma_node = [&](int nk) -> ac::CollabNode& {
                    return ma->collab ? ma->collab->fleet().node(ac::AgentId(nk)) : *ma->nodes[size_t(nk)];
                };
                // D179b: under collab, ONE snapshot per barrier (taken right after the ladder) serves every
                // component / heartbeat / acked read, so all five sites see one tick's state - frozen in Publish
                // after Deliver+Tick, logged by version, replay-identical. Without collab the nodes are ours and
                // the direct reads stand.
                std::shared_ptr<const live::CollabSnapshot> csnap;
                // A missing per-agent view (the initial snapshot, before the fleet thread's first Publish, has an
                // EMPTY agent map - found the hard way: .at() from a fork-join thread killed collab60b's live run
                // at barrier 1) means "nothing heard yet": a fresh node's own state. Never .at() into csnap.
                const auto ma_view = [&](int nk) -> const live::CollabAgentView* {
                    if (!csnap) return nullptr;
                    const auto it = csnap->agent.find(ac::AgentId(nk));
                    return it == csnap->agent.end() ? nullptr : &it->second;
                };
                const auto ma_comp = [&](int nk) {
                    if (!csnap) return ma_node(nk).Component();
                    if (const auto* v = ma_view(nk)) return v->component;
                    ac::CollabNode::ComponentView self_only;   // what a fresh node reports: itself, broker itself
                    self_only.broker = ac::AgentId(nk);
                    self_only.members = {ac::AgentId(nk)};
                    return self_only;
                };
                const auto allocation_component = [&](int nk) {
                    const auto comp = ma_comp(nk);
                    multi::ComponentInfo info;
                    info.broker = int32_t(comp.broker);
                    for (auto member : comp.members) if (int(member) < NA) info.members.push_back(int32_t(member));
                    info.epoch = multi::CoordinationToken(info.broker, info.members);
                    return info;
                };
                const auto ma_hb = [&](int nk, int nj) -> std::optional<core::HeartbeatValue> {
                    if (!csnap) return ma_node(nk).LastHeartbeat(ac::AgentId(nj));
                    const auto* v = ma_view(nk);
                    if (!v) return std::nullopt;
                    const auto it = v->heartbeat.find(ac::AgentId(nj));
                    return it == v->heartbeat.end() ? std::nullopt : std::optional<core::HeartbeatValue>(it->second);
                };
                const auto ma_acked = [&](int nk, int nj) -> uint64_t {
                    if (!csnap) return ma_node(nk).AckedPatchVersion(ac::AgentId(nj));
                    const auto* v = ma_view(nk);
                    if (!v) return 0;
                    const auto it = v->acked_patch.find(ac::AgentId(nj));
                    return it == v->acked_patch.end() ? 0 : it->second;
                };
                // --- D175 item 4: the per-agent barrier ---------------------------------------------------------
                // (1) The comms ticks for (T_prev, T] run BEFORE the planners (D175 SS5), nodes in ascending id
                // per rung: the one seeded, single-threaded step. The rung spacing is the heartbeat period
                // (membership needs >= m_in of the last `window` periods mutually heard to form a component - one
                // tick per barrier starves it; found on the first POC smoke: every agent stayed a component of 1),
                // and the endpoint pumps what the previous barrier enqueued. Heartbeat poses are the last keyframe
                // added by the barrier's integrate step, so a pose is at most one replan stale. Deliveries land in
                // the mailboxes via the coordination hook.
                {
                    const int64_t tick_dt = 500000000;   // = MembershipParams.period_s
                    live::AdvanceRadioTicks(radio_tick_ns, T, tick_dt, [&](int64_t tt) {
                        const core::Timestamp t_now{tt};
                        if (ma->collab) {   // D174: the fleet owns delivery, releases and ticks (lockstep watermark)
                            ma->collab->Step(t_now);
                            return;
                        }
                        // The network is a discrete-event queue: arrived datagrams must be ROUTED to their
                        // destination node before the tick (pass B's loop does exactly this; without it nothing
                        // is ever received - found on the second POC smoke: heartbeats sent, none heard).
                        for (const auto& d : ma->comms->network().DeliverUntil(t_now))
                            if (int(d.to) <= NA) ma->nodes[size_t(d.to)]->OnDatagram(d, t_now);
                        for (auto& nd : ma->nodes) nd->Tick(t_now, ma->comms->network());
                    });
                    if (ma->collab) csnap = ma->collab->Snapshot(T);   // D179b: the barrier's one atomic view
                    if (ma->collab) {   // D174 §4 feedback + the panel, at the barrier
                        // (csnap was taken right after the ladder: the same atomic value serves the whole barrier)
                        const auto& snap = csnap;
                        {   // adopted frames move the briefing constants (deliver_frame's telemetry reads them)
                            std::lock_guard<std::mutex> g(ma->two_mu);
                            for (const auto& [ck, Tk] : snap->T_O0_O)
                                if (int(ck) < NA && int(ck) > 0) brief.T_W_O[size_t(ck)] = brief.T_W_O[0] * Tk;
                        }
                        ma->linked = snap->linked;
                        if (!snap->telemetry.empty()) telemetry.Put(snap->telemetry);
                        const fs::path cib = run_dir / "collab_inbox";   // panel buttons, recorded inputs
                        if (fs::exists(cib)) {
                            std::vector<fs::path> files;
                            for (const auto& e : fs::directory_iterator(cib))
                                if (e.path().extension() == ".json") files.push_back(e.path());
                            std::sort(files.begin(), files.end());
                            for (const auto& f : files) {
                                try {
                                    const YAML::Node n2 = YAML::LoadFile(f.string());
                                    const std::string act = n2["action"].as<std::string>();
                                    if (act == "cut_comms") {
                                        std::vector<std::vector<ac::AgentId>> groups;
                                        for (const auto& g2 : n2["groups"]) groups.push_back(g2.as<std::vector<ac::AgentId>>());
                                        ma->collab->CutComms(core::Timestamp{T}, std::move(groups));
                                    } else if (act == "restore_comms") {
                                        ma->collab->RestoreComms(core::Timestamp{T});
                                    } else if (act == "inject_false") {
                                        ma->collab->InjectFalse(core::Timestamp{T});
                                    } else {
                                        throw std::invalid_argument("action " + act);
                                    }
                                    telemetry.Put("{\"type\":\"collab_action\",\"t_s\":" + std::to_string(T * 1e-9) +
                                                  ",\"action\":\"" + act + "\"}");
                                    fs::rename(f, fs::path(f.string() + ".applied"));
                                } catch (const std::exception&) {
                                    fs::rename(f, fs::path(f.string() + ".rejected"));
                                }
                            }
                        }
                    }
                }
                pmark(2);   // radio ladder (+ snapshot) done
                // (2) Drain what arrived by T. Patches integrate at T_G_W(sender) = identity: own maps are kept in
                // W = G under the known entry poses (D175 SS2 Frames; LIVE_COLLAB's adopted estimates re-anchor later).
                for (int k = 0; k <= NA; ++k) {
                    // D179: with the async fleet, deliveries come from its logged drain (my mailboxes stay empty);
                    // ticks in order, FIFO per sender, identical in replay (invariant 1).
                    const std::vector<core::DecodedMessage> drained =
                        ma->collab ? ma->collab->Drain(ac::AgentId(k), T) : std::vector<core::DecodedMessage>{};
                    for (const auto& d : ma->collab ? drained : ma->mailbox[size_t(k)]) {
                        if (d.has_submap_patch && d.submap_patch.sender < NA)
                            ma->layers[size_t(k)].Integrate(live::FromWire(d.submap_patch), {});
                        if (d.has_task_bid) {
                            auto bb = live::FromWire(d.task_bid);
                            ma->bids_rx[size_t(k)][bb.sender] = std::move(bb);
                        }
                        if (d.has_task_consensus && k < NA) {
                            ma->cons_rx[size_t(k)] = live::FromWire(d.task_consensus);
                            ma->cons_rx_t_ns[size_t(k)] = T;
                        }
                    }
                    ma->mailbox[size_t(k)].clear();
                }
                pmark(3);   // drain done
                // (3) Component brokers assign from the bids that ARRIVED (the T - replan lag is the pre-registered
                // coordination cost, D175 SS8.3). Sub-ms, serial. The consensus reaches members at the next tick.
                for (int k = 0; k < NA; ++k) {
                    const auto comp = ma_comp(k);
                    if (int(comp.broker) != k) continue;
                    const multi::ComponentInfo ci = allocation_component(k);
                    std::vector<multi::TaskBid> bids;
                    bids.reserve(ma->bids_rx[size_t(k)].size());
                    for (const auto& [snd, bb] : ma->bids_rx[size_t(k)]) bids.push_back(bb);
                    const auto cons = multi::BrokerAssign(bids, ci, ma->broker, T, ++ma->cons_seq[size_t(k)]);
                    if (ma->collab) {
                        ma->collab->Send(ac::AgentId(k),
                                         [cw = live::ToWire(cons, ma->g_tag)](ac::CollabNode& n, core::Timestamp t) {
                                             n.SendConsensus(cw, t);
                                         },
                                         core::Timestamp{T});
                    } else {
                        ma_node(k).SendConsensus(live::ToWire(cons, ma->g_tag), core::Timestamp{T});
                    }
                }
                pmark(4);   // brokers done
                // (4) Per-agent plans: share-nothing fork-join (user directive: nothing between barriers holds a
                // lock). Thread k touches ONLY planner_k / exporter_k / layers_k / node_k reads; sends wait for (5).
                // (the extra names avoid C++'s most-vexing parse: size_t(NA) as a parameter list - bitten at D213 too)
                std::vector<live::PlanOutput> plans;
                plans.resize(size_t(NA));
                std::vector<multi::TaskBid> bids_out;
                bids_out.resize(size_t(NA));
                std::vector<std::vector<std::pair<int, core::SubmapPatchValue>>> patches_out;
                patches_out.resize(size_t(NA));
                ma_picks.resize(size_t(NA));
                std::vector<std::vector<std::pair<std::string, double>>> plan_stage_ms;
                plan_stage_ms.resize(size_t(NA));
                {
                    // Live demo: each plan jumps its worker's integration backlog (agent_worker.hpp PostUrgent).
                    std::vector<std::future<void>> urgent;
                    const auto fork_t0 = std::chrono::steady_clock::now();
                    const auto post_plan = [&](int k, std::function<void()> job) {
                        if (plan_ahead) urgent.push_back(ma->workers[size_t(k)].PostUrgent(std::move(job)));
                        else ma->workers[size_t(k)].Post(std::move(job));
                    };
                    for (int k = 0; k < NA; ++k) post_plan(k, [&, k] {
                        const auto comp = ma_comp(k);
                        const auto component = allocation_component(k);
                        const uint32_t epoch = component.epoch;
                        const uint32_t map_epoch = uint32_t(comp.epoch.counter);
                        std::vector<planning::PeerKeyframe> peers;
                        for (int j = 0; j < NA; ++j) {
                            if (j == k) continue;
                            if (const auto h = ma_hb(k, j)) {
                                const core::SE3 Twj = brief.T_W_O[size_t(j)] * ac::FromPoseValue(h->pose);
                                peers.push_back({j, int64_t(h->keyframe_watermark), Twj.t});
                            }
                        }
                        const bool lkd = !ma->collab || [&] {   // the fleet's comms state (D174); true pre-collab
                            const auto it = ma->linked.find(ac::AgentId(k));
                            return it != ma->linked.end() && it->second;
                        }();
                        const auto job_t0 = std::chrono::steady_clock::now();
                        const auto since = [](std::chrono::steady_clock::time_point t) {
                            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
                        };
                        plans[size_t(k)] = ma->planners[size_t(k)].PlanOwnFor(ctx, k, peers,
                                                                              &ma->layers[size_t(k)].KnownMask(), lkd);
                        const auto& lp = ma->planners[size_t(k)].LastPlan();
                        if (profile) {
                            plan_stage_ms[size_t(k)] = lp.stage_ms;
                            plan_stage_ms[size_t(k)].emplace_back("plan_own_for", since(job_t0));
                            // the wait for worker_k (the integration in progress; with Post, the whole backlog)
                            plan_stage_ms[size_t(k)].emplace_back(
                                "start_wait", std::chrono::duration<double, std::milli>(job_t0 - fork_t0).count());
                        }
                        const auto export_t0 = std::chrono::steady_clock::now();
                        ma->exporters[size_t(k)].Update(lp.maps);
                        for (int j = 0; j <= NA; ++j) {
                            if (j == k) continue;
                            auto patch = ma->exporters[size_t(k)].Export(
                                ma_acked(k, j), map_epoch, T);
                            if (!patch.blocks.empty())
                                patches_out[size_t(k)].push_back({j, live::ToWire(patch, core::FrameTag{ac::AgentId(k), 0})});
                        }
                        if (profile) plan_stage_ms[size_t(k)].emplace_back("export", since(export_t0));
                        bids_out[size_t(k)] = multi::BuildBid(k, epoch, ++ma->bid_seq[size_t(k)], T,
                                                              lp.agents[size_t(k)].nbv, ma->max_options, {});
                        // (6, in-thread) the pick from the newest consensus RECEIVED by T - reads only this agent's state
                        const multi::TaskConsensus* c = ma->cons_rx[size_t(k)] ? &*ma->cons_rx[size_t(k)] : nullptr;
                        const auto& opts = lp.agents[size_t(k)].nbv.options;
                        if (c && !multi::MatchesComponent(*c, component)) {
                            ma_picks[size_t(k)].option = opts.empty() ? -1 : 0;
                            ma_picks[size_t(k)].coordinated = false;
                            ma_picks[size_t(k)].why = "uncoordinated: component changed";
                            return;
                        }
                        ma_picks[size_t(k)] = ma->discounted
                            ? multi::ApplyConsensusDiscounted(c, k, epoch, T, ma->timeout_s, opts, ma->broker.r_same,
                                                              ma->broker.range_m, ma->tau_s)
                            : multi::ApplyConsensus(c, k, epoch, T, ma->timeout_s, opts, ma->broker.r_same);
                    });
                    if (plan_ahead) {   // every plan finishes before a failure leaves this scope (they capture &)
                        for (auto& f : urgent) f.wait();
                        for (auto& f : urgent) f.get();
                    } else {
                        for (auto& w : ma->workers) w.Wait();
                    }
                }
                if (profile) {   // per agent: Phase4Planner's own stage split of its plan (PlanOwn)
                    std::ostringstream ps;
                    ps.precision(6);
                    ps << ",\"plan_stages\":[";
                    for (int k = 0; k < NA; ++k) {
                        ps << (k ? "," : "") << "{";
                        for (size_t i = 0; i < plan_stage_ms[size_t(k)].size(); ++i)
                            ps << (i ? "," : "") << "\"" << plan_stage_ms[size_t(k)][i].first
                               << "\":" << plan_stage_ms[size_t(k)][i].second;
                        ps << "}";
                    }
                    ps << "]";
                    profile_extra += ps.str();
                }
                pmark(5);   // plan fork-join done
                // (5) Sends, serial in ascending id: the seeded link sees one deterministic order, and CommsNode
                // stays single-writer. Bids self-deliver at the broker immediately (local, no radio hop).
                for (int k = 0; k < NA; ++k) {
                    if (ma->collab) {   // D179: enqueued in call order onto the fleet thread
                        for (auto& jp : patches_out[size_t(k)]) {   // no structured bindings: init-capture needs names
                            ma->collab->Send(ac::AgentId(k),
                                             [j2 = jp.first, pv2 = std::move(jp.second)](ac::CollabNode& n,
                                                                                         core::Timestamp t) {
                                                 n.SendPatch(ac::AgentId(j2), pv2, t);
                                             },
                                             core::Timestamp{T});
                        }
                        ma->collab->Send(ac::AgentId(k),
                                         [bw = live::ToWire(bids_out[size_t(k)], ma->g_tag)](ac::CollabNode& n,
                                                                                             core::Timestamp t) {
                                             n.SendBid(bw, t);
                                         },
                                         core::Timestamp{T});
                    } else {
                        for (auto& [j, pv] : patches_out[size_t(k)])
                            ma_node(k).SendPatch(ac::AgentId(j), pv, core::Timestamp{T});
                        ma_node(k).SendBid(live::ToWire(bids_out[size_t(k)], ma->g_tag), core::Timestamp{T});
                    }
                }
                pmark(6);   // sends enqueued
                // (7) Merge into the one PlanOutput the chooser/validation path reads; the auction pick replaces
                // the centralised joint[k] (A1: "the equivalent of the old joint[k]").
                plan_out = live::PlanOutput(NA);
                std::ostringstream mt;
                mt.precision(10);
                mt << "{\"name\":\"a1_phase4_multiagent\",\"broker_mode\":\"" << lc.multiagent.broker_mode
                   << "\",\"scenario\":\"" << ma->scenario.name << "\",\"agents\":[";
                for (int k = 0; k < NA; ++k) {
                    plan_out.agents[size_t(k)] = plans[size_t(k)].agents[size_t(k)];
                    plan_out.joint[size_t(k)] = ma_picks[size_t(k)].option;
                    // under collab: the component as the agent SAW IT when it picked (the pre-send snapshot);
                    // the post-consensus view appears in the next barrier's line
                    const auto comp = ma_comp(k);
                    mt << (k ? "," : "") << "{\"pick\":" << ma_picks[size_t(k)].option << ",\"why\":\""
                       << JsonEscape(ma_picks[size_t(k)].why) << "\",\"broker\":" << int(comp.broker)
                       << ",\"members\":" << comp.members.size() << ",\"epoch\":" << comp.epoch.counter
                       << ",\"allocation_token\":" << allocation_component(k).epoch << "}";
                }
                const auto charged = [&](core::MessageClass cls) {
                    return csnap ? csnap->Bytes(cls) : ma->comms->ledger().TotalForClass(cls);
                };
                mt << "],\"bytes\":{\"patch\":" << charged(core::MessageClass::SUBMAP_PATCH)
                   << ",\"bid\":" << charged(core::MessageClass::TASK_BID)
                   << ",\"consensus\":" << charged(core::MessageClass::TASK_CONSENSUS)
                   << ",\"heartbeat\":" << charged(core::MessageClass::HEARTBEAT) << "}}";
                plan_out.trace_json = mt.str();
                // The console map and the return route are what the OPERATOR received over the radio (D175 SS8.2,
                // the honest behaviour): the merged layer in the occupancy_grid slab convention, and A1's
                // incremental D* Lite over it at rho_responder. Its confidence is NOT computed (support is not on
                // the wire) - the console must not show one.
                const cv::Mat merged = ma->layers[size_t(NA)].Merged();
                cv::Mat slab(merged.size(), CV_8U, cv::Scalar(planning::kUnknown));
                slab.setTo(planning::kOccupied, merged == multi::kCellOccupied);
                slab.setTo(planning::kFree, merged == multi::kCellFree);
                slab.setTo(planning::kFree, merged == multi::kCellFreeResponder);
                plan_out.maps = plans[0].maps;
                plan_out.maps.slab = slab;
                const planning::EgressReplan oe = ma->op_egress->Replan(merged);
                plan_out.maps.egress.found = oe.found;
                plan_out.maps.egress.cells = oe.cells;
                plan_out.maps.egress.goal_index = oe.exit;
                plan_out.maps.egress.length_cells = oe.length_m / brief.gs.cell;
            } else {
                std::lock_guard<std::mutex> plk(planner_mu);   // uncontended outside demo
                plan_out = planner->Plan(ctx);
            }
            pmark(7);   // plan/merge done (both planner paths)
            const mis::ReplanMaps& maps = plan_out.maps;
            const double ts = T * 1e-9;
            const std::string snap = "snapshots/slab_" + std::to_string(int64_t(std::llround(ts * 1000))) + ".png";
            if (!lc.demo) cv::imwrite((run_dir / snap).string(), maps.slab);   // D169: no per-replan PNGs in demo
            if (shadow) {
                const std::string p4 = "snapshots/p4slab_" + std::to_string(int64_t(std::llround(ts * 1000))) + ".png";
                cv::imwrite((run_dir / p4).string(),
                            planning::ComputeTraversability(shadow->Map(), shadow->Distance(), shadow_grid, shadow_trav).slab);
            }
            {
                std::ostringstream o;
                o.precision(6);
                o << "{\"type\":\"snapshot\",\"t_s\":" << ts << ",\"png\":\"" << snap << "\",\"egress_xy\":";
                if (maps.egress.found) {   // the egress route at rho_responder (invariant 4), for the console
                    o << "[";
                    for (size_t i = 0; i < maps.egress.cells.size(); ++i) {
                        const auto [x, y] = brief.CellCentre(maps.egress.cells[i]);
                        o << (i ? "," : "") << Xy(x, y);
                    }
                    o << "]";
                } else {
                    o << "null";
                }
                o << "}";
                telemetry.Put(o.str());
            }
            ++replans;
            // Phase 1, per agent: the deterministic layer's candidate set, its risk (§23), the classical choice.
            struct Pending {
                bool active = false;
                std::string head;                  // the trace record up to (not including) "chosen"
                Cell ac;
                cv::Mat plan;
                std::vector<mis::Candidate> cands;   // ELIGIBLE only (path >= min_goal_m), nearest first
                std::vector<double> risk;
                std::vector<bool> exempt;          // supervisor_return: never gated (§21.4, maintainer 2026-10-23)
                int classical = -1;
                std::string agent_json;            // this agent's part of the joint request (without the closing brace)
            };
            std::vector<Pending> P(static_cast<size_t>(NA));
            const double rho_a = mc.rho_a, rho_r = mc.rho_r;
            const int free_cells = cv::countNonZero(maps.slab == planning::kFree),
                      occ_cells = cv::countNonZero(maps.slab == planning::kOccupied),
                      unknown_cells = int(maps.slab.total()) - free_cells - occ_cells;
            for (int k = 0; k < NA; ++k) {
                std::ostringstream o;
                o.precision(10);
                o << "{\"t_s\":" << ts << ",\"agent\":" << k << ",\"mode\":\"" << (is_live ? "live" : "replay")
                  << "\",\"executed\":" << (is_live ? "true" : "false") << ",\"decision\":\"frontier\",\"chooser\":\""
                  << lc.chooser << "\",";
                // The vision-derived inputs of this decision: the map built from keyframe depth, the agent's estimate.
                bool have_pose = false;
                core::SE3 T_O_C;
                size_t frames = 0, kfs = 0;
                int64_t last_kf = -1;
                {
                    std::lock_guard<std::mutex> lk(lanes[k].mu);
                    for (auto it = lanes[k].cams.rbegin(); it != lanes[k].cams.rend(); ++it) {
                        if (it->first < T) {
                            T_O_C = it->second;
                            have_pose = true;
                            break;
                        }
                    }
                    for (const auto& [t, Tc] : lanes[k].cams) frames += t < T;
                    for (int64_t t : lanes[k].kf_times) {
                        if (t < T) ++kfs, last_kf = t;
                    }
                }
                std::ostringstream vision;
                vision.precision(10);
                vision << "{\"keyframes_integrated\":[";
                for (int a2 = 0; a2 < NA; ++a2) vision << (a2 ? "," : "") << integrated[size_t(a2)];
                vision << "],\"rays\":" << rays
                       << ",\"free_cells\":" << free_cells << ",\"occupied_cells\":" << occ_cells << ",\"unknown_cells\":"
                       << unknown_cells << ",\"frontier_clusters\":" << maps.frontiers.clusters.size()
                       << ",\"rejected_small\":" << maps.frontiers.rejected_small << ",\"egress_found\":"
                       << (maps.egress.found ? "true" : "false") << ",\"snapshot_png\":\"" << snap << "\"";
                if (!have_pose) {
                    o << "\"vision\":" << vision.str() << "},\"result\":\"no decision: no frame yet\"}";
                    trace.Put(o.str());
                    continue;
                }
                const core::SE3 T_W_C = brief.T_W_O[size_t(k)] * T_O_C;
                const Cell ac = brief.ToCell(T_W_C.t.x(), T_W_C.t.y());
                vision << ",\"agent_xy_m\":" << Xy(T_W_C.t.x(), T_W_C.t.y()) << ",\"agent_cell\":[" << ac.r << ","
                       << ac.c << "]}";
                o << "\"vision\":" << vision.str() << ",";
                if (plan_out.trace_json != "{}") o << "\"planner\":" << plan_out.trace_json << ",";
                Pending& pd = P[k];
                pd.active = true;
                pd.ac = ac;
                // The planner's options (seam: AGENTIC_7 §0b), rendered into the trace and the LLM request.
                const live::AgentOptions& ao = plan_out.agents[k];
                pd.plan = ao.plan;
                pd.classical = ao.classical;
                const int excluded_refused = ao.excluded_refused, excluded_near = ao.excluded_near;
                std::ostringstream cj;
                cj.precision(6);
                cj << "[";
                // The current task, and the option that continues it (nearest goal within refused_radius_m).
                int continues = -1;
                double cont_d = lc.refused_radius_m;
                for (size_t i = 0; current[k].set && i < ao.options.size(); ++i) {
                    const double d = std::hypot(ao.options[i].goal_xy.first - current[k].gx,
                                                ao.options[i].goal_xy.second - current[k].gy);
                    if (d < cont_d) cont_d = d, continues = int(i);
                }
                std::ostringstream task;
                task.precision(6);
                if (!current[k].set) {
                    task << "null";
                } else {
                    task << "{\"goal_xy_m\":" << Xy(current[k].gx, current[k].gy) << ",\"since_t_s\":"
                         << current[k].since_ns * 1e-9 << ",\"straight_line_m\":"
                         << std::hypot(current[k].gx - T_W_C.t.x(), current[k].gy - T_W_C.t.y())
                         << ",\"candidate\":" << continues;
                }
                for (size_t i = 0; i < ao.options.size(); ++i) {
                    const auto& t = ao.options[i];   // the seam's fields only (A1's TaskCandidate names)
                    const double r = t.risk;
                    pd.cands.push_back(t.c);
                    pd.risk.push_back(r);
                    pd.exempt.push_back(t.supervisor_return);
                    // Operator intent at this goal (§0b): the region directives for this agent covering it, the product
                    // of their priority weights, the distance to the nearest one (0 inside; null if none is active).
                    std::string covering;
                    double weight = 1, to_dir = std::numeric_limits<double>::infinity();
                    for (const auto& d : active) {
                        if (d.kind != "priority_region" && d.kind != "explore" && d.kind != "assign") continue;
                        if (d.agent >= 0 && d.agent != k) continue;
                        const double dd = d.Distance(t.goal_xy.first, t.goal_xy.second);
                        to_dir = std::min(to_dir, dd);
                        if (dd == 0) {
                            covering += (covering.empty() ? "\"" : ",\"") + JsonEscape(d.id) + "\"";
                            if (d.kind == "priority_region") weight *= d.weight;
                        }
                    }
                    cj << (i ? "," : "") << "{\"index\":" << i << ",\"rank\":" << i << ",\"utility\":" << t.utility
                       << ",\"cluster\":" << t.task_id << ",\"path_m\":"
                       << t.path_m << ",\"goal_xy_m\":" << Xy(t.goal_xy.first, t.goal_xy.second) << ",\"standoff_m\":"
                       << t.standoff_m << ",\"centroid_xy_m\":" << Xy(t.centroid_xy.first, t.centroid_xy.second)
                       << ",\"cells\":" << t.cells << ",\"risk\":" << r << ",\"risk_reason\":\"" << t.risk_reason
                       << "\",\"needs_approval\":" << (r > lc.autonomous_threshold && !t.supervisor_return ? "true" : "false")
                       << (t.supervisor_return ? ",\"supervisor_return\":true" : "")
                       << ",\"directives\":[" << covering << "],\"priority_weight\":" << weight << ",\"to_directive_m\":";
                    if (std::isfinite(to_dir)) cj << to_dir; else cj << "null";
                    cj << "}";
                }
                cj << "]";
                o << "\"candidates\":" << cj.str() << ",\"candidates_total\":" << pd.cands.size()
                  << ",\"excluded_refused\":" << excluded_refused << ",\"excluded_near\":" << excluded_near
                  << ",\"excluded_directive\":" << ao.excluded_directive << ",\"directives\":[";
                for (size_t i = 0; i < ao.applied.size(); ++i) o << (i ? "," : "") << "\"" << JsonEscape(ao.applied[i]) << "\"";
                o << "],\"current_task\":" << task.str() << (current[k].set ? "}" : "")
                  << ",\"autonomous_threshold\":" << lc.autonomous_threshold << ",\"classical_choice\":" << pd.classical
                  << ",";
                pd.head = o.str();
                {
                    std::ostringstream q;
                    q.precision(10);
                    q << "{\"agent\":" << k << ",\"vision\":" << vision.str() << ",\"estimator\":{\"frames\":"
                      << frames << ",\"keyframes\":" << kfs << ",\"since_last_keyframe_s\":"
                      << (last_kf < 0 ? -1.0 : (T - last_kf) * 1e-9) << "},\"candidates\":" << cj.str()
                      << ",\"classical_choice\":" << pd.classical << ",\"current_task\":" << task.str();
                    if (current[k].set) q << ",\"reason\":\"" << JsonEscape(current[k].reason) << "\"}";
                    pd.agent_json = q.str();
                }
            }
            // Phase 2: choose, jointly for both agents (AGENTIC_7 §0: Co-NavGPT's joint assignment; exactly-once).
            std::vector<int> picks(plan_out.joint.begin(), plan_out.joint.end());   // the planner's allocation
            std::vector<std::string> choice(static_cast<size_t>(NA)), pick_reason(static_cast<size_t>(NA), "classical choice");
            const auto other_cluster = [&](int k, int64_t taken) { return live::OtherTask(plan_out, k, taken, lc.autonomous_threshold); };
            const auto cluster_of = [&](int k, int i) { return live::TaskOf(plan_out, k, i); };
            if (lc.chooser == "classical") {
                for (auto& c : choice) c = ",\"choice\":{\"source\":\"classical\"}";
            } else if (!is_live && chooser_socket.empty()) {
                std::ostringstream t;
                t.precision(10);
                t << ts;
                for (int k = 0; k < NA; ++k) {
                    if (!P[k].active) continue;
                    const auto it = recorded.find({t.str(), k});
                    if (it == recorded.end()) throw std::runtime_error("replay: no recorded choice at t=" + t.str());
                    picks[k] = it->second.first;
                    choice[k] = it->second.second;
                }
            } else {
                // The joint request: both agents' facts and candidates, plus the images the model may look at, all
                // rendered here with OpenCV (the orchestrator image has no OpenCV).
                const std::string did = "t" + std::to_string(T / 1000000) + "ms";
                const fs::path ddir = run_dir / "decisions";
                fs::create_directories(ddir);
                cv::Mat img;
                cv::Mat lut(1, 256, CV_8UC3, cv::Scalar(128, 128, 128));
                lut.at<cv::Vec3b>(planning::kFree) = {255, 255, 255};
                lut.at<cv::Vec3b>(planning::kOccupied) = {0, 0, 0};
                cv::Mat slab3;
                cv::cvtColor(maps.slab, slab3, cv::COLOR_GRAY2BGR);
                cv::LUT(slab3, lut, img);
                cv::flip(img, img, 0);   // row 0 = south in the grid; north up in the picture
                const int scale = 3;
                cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
                const auto px = [&](const Cell& c) { return cv::Point(c.c * scale + scale / 2, (maps.slab.rows - 1 - c.r) * scale + scale / 2); };
                for (size_t i = 1; i < maps.egress.cells.size(); ++i)
                    cv::line(img, px(maps.egress.cells[i - 1]), px(maps.egress.cells[i]), {255, 80, 0}, 2);
                const cv::Scalar col[] = {{0, 0, 230}, {0, 160, 0}, {200, 80, 0}};   // index mod 3
                for (int k = 0; k < NA; ++k) {
                    if (!P[k].active) continue;
                    for (size_t i = 0; i < P[k].cands.size(); ++i) {
                        const cv::Point g = px(P[k].cands[i].goal);
                        cv::drawMarker(img, g, col[k % 3], cv::MARKER_TILTED_CROSS, 10, 2);
                        cv::putText(img, "a" + std::to_string(k) + ":c" + std::to_string(i), g + cv::Point(5, -5),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.4, col[k % 3], 1, cv::LINE_AA);
                    }
                    cv::circle(img, px(P[k].ac), 7, col[k % 3], cv::FILLED);
                    cv::putText(img, "agent " + std::to_string(k), px(P[k].ac) + cv::Point(9, 4), cv::FONT_HERSHEY_SIMPLEX,
                                0.45, col[k % 3], 1, cv::LINE_AA);
                }
                const fs::path map_png = fs::absolute(ddir / (did + "_map.png"));
                cv::imwrite(map_png.string(), img);
                std::ostringstream q;
                q.precision(10);
                q << "{\"decision_id\":\"" << did << "\",\"t_s\":" << ts << ",\"map_png\":\"" << map_png.string()
                  << "\",\"egress\":{\"found\":" << (maps.egress.found ? "true" : "false") << ",\"length_m\":"
                  << maps.egress.length_cells * brief.gs.cell << ",\"exit\":" << maps.egress.goal_index
                  << ",\"clearance_m\":" << rho_r << "},\"autonomous_threshold\":" << lc.autonomous_threshold
                  << ",\"approvals_dir\":\"" << fs::absolute(run_dir / "approvals").string() << "\",\"directives\":[";
                for (size_t i = 0; i < active.size(); ++i) q << (i ? "," : "") << mis::DirectiveJson(active[i]);
                q << "],\"agents\":[";
                bool first = true;
                for (int k = 0; k < NA; ++k) {
                    if (!P[k].active) continue;
                    std::string kf_png;
                    {
                        std::lock_guard<std::mutex> lk(lanes[k].mu);
                        if (!lanes[k].last_rgb.empty()) {
                            kf_png = fs::absolute(ddir / (did + "_a" + std::to_string(k) + "_kf.png")).string();
                            cv::Mat half, bgr;
                            cv::resize(lanes[k].last_rgb, half, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                            if (half.channels() == 3) cv::cvtColor(half, bgr, cv::COLOR_RGB2BGR);
                            else bgr = half;
                            cv::imwrite(kf_png, bgr);
                        }
                    }
                    q << (first ? "" : ",") << P[k].agent_json << ",\"keyframe_png\":"
                      << (kf_png.empty() ? "null" : "\"" + kf_png + "\"") << "}";
                    first = false;
                }
                q << "]}";
                std::string ans, top_problem;
                std::string vdid = did;   // the request the processed answer belongs to (demo: the PREVIOUS horizon)
                std::vector<std::vector<std::pair<double, double>>> prev_goals(static_cast<size_t>(NA));
                try {   // run 4: the orchestrator's handler died and Send threw out of main; the unwind destroyed a
                        // joinable thread -> std::terminate. A dead chooser link degrades like a timeout, never a crash.
                    chooser->Send(q.str());
                    if (lc.demo) {   // D169: never wait on the control path - poll the previous horizon's answer
                        vdid = demo_prev_did;
                        for (int k = 0; k < NA; ++k) prev_goals[k] = std::move(demo_prev_goals[k]);
                        demo_prev_did = did;
                        for (int k = 0; k < NA; ++k) {
                            demo_prev_goals[k].clear();
                            for (const auto& op : plan_out.agents[k].options) demo_prev_goals[k].push_back(op.goal_xy);
                        }
                        if (vdid.empty()) {
                            top_problem = "first horizon: no previous request";
                        } else {
                            ans = chooser->Wait(vdid, std::chrono::steady_clock::now());
                            if (ans.empty()) top_problem = "expired: no answer within one horizon";
                        }
                    } else {
                        ans = chooser->Wait(did, std::chrono::steady_clock::now() +
                                                     std::chrono::milliseconds(int64_t(lc.llm_timeout_s * 1000)));
                    }
                } catch (const std::exception& e) {
                    top_problem = std::string("chooser link down: ") + e.what();
                }
                YAML::Node assigned;
                if (top_problem.empty() && ans.empty()) {
                    top_problem = "timeout or no orchestrator";
                } else if (top_problem.empty()) {
                    try {
                        assigned = YAML::Load(ans)["assignments"];
                        if (!assigned || assigned.IsNull()) top_problem = "no assignment recorded";
                    } catch (const std::exception& e) {
                        top_problem = std::string("malformed answer: ") + e.what();
                    }
                }
                // Per agent: a bad pick for one agent does not discard the other's.
                std::vector<int> want(static_cast<size_t>(NA), -2);
                std::vector<std::string> approval_id(static_cast<size_t>(NA)), validation(static_cast<size_t>(NA)), source(static_cast<size_t>(NA)),
                    approval_raw(static_cast<size_t>(NA));
                for (int k = 0; k < NA; ++k) {
                    if (!P[k].active) continue;
                    source[k] = "orchestrator", validation[k] = "ok";
                    if (!top_problem.empty()) {
                        source[k] = "fallback", validation[k] = top_problem;
                        continue;
                    }
                    for (const auto& e : assigned) {
                        if (e["agent"] && e["agent"].as<int>() == k) {
                            want[k] = e["candidate"] ? e["candidate"].as<int>() : -2;
                            approval_id[k] = e["approval_id"] ? e["approval_id"].as<std::string>() : "";
                            if (e["reason"]) pick_reason[k] = e["reason"].as<std::string>();
                        }
                    }
                    if (lc.demo && want[k] >= 0) {   // D169: the answer names the PREVIOUS horizon's candidate;
                        int j = -2;                  // map it by goal, the same rule that continues current_task
                        if (size_t(want[k]) < prev_goals[k].size()) {
                            const auto g = prev_goals[k][size_t(want[k])];
                            double best = lc.refused_radius_m;
                            for (size_t i = 0; i < plan_out.agents[k].options.size(); ++i) {
                                const auto& o2 = plan_out.agents[k].options[i].goal_xy;
                                const double d = std::hypot(o2.first - g.first, o2.second - g.second);
                                if (d < best) best = d, j = int(i);
                            }
                        }
                        want[k] = j;   // -2 = the override's goal is gone: the generic fallback below says so
                    }
                    if (want[k] < -1 || want[k] >= int(P[k].cands.size())) {
                        source[k] = "fallback", validation[k] = "missing or out of the candidate set";
                    } else if (want[k] >= 0 && P[k].risk[size_t(want[k])] > lc.autonomous_threshold &&
                               !P[k].exempt[size_t(want[k])]) {
                        const std::string expect = vdid + "-a" + std::to_string(k) + "-c" + std::to_string(want[k]);
                        std::ifstream f(run_dir / "approvals" / (expect + ".json"));
                        approval_raw[k].assign(std::istreambuf_iterator<char>(f), {});
                        bool approved = false;
                        try {
                            approved = !approval_raw[k].empty() && approval_id[k] == expect &&
                                       YAML::Load(approval_raw[k])["verdict"].as<std::string>() == "approved";
                        } catch (const std::exception&) {
                        }
                        if (!approved) source[k] = "fallback", validation[k] = "above the risk threshold without an approval";
                    }
                    if (source[k] == "orchestrator") picks[k] = want[k];
                    else pick_reason[k] = "fallback: " + validation[k];
                }
                {   // exactly-once across agents, ascending index (P8 D213; at N=2 the old agent-1-yields rule)
                    std::set<int64_t> taken2;
                    for (int k = 0; k < NA; ++k) {
                        if (!P[size_t(k)].active || picks[size_t(k)] < 0) continue;
                        const int64_t cl = cluster_of(k, picks[size_t(k)]);
                        if (taken2.count(cl) && source[size_t(k)] == "orchestrator") {
                            const int alt = other_cluster(k, cl);
                            if (alt >= 0) {
                                picks[size_t(k)] = alt;
                                source[size_t(k)] = "fallback";
                                validation[size_t(k)] = "same cluster as an earlier agent while another was eligible";
                            }
                        }
                        if (picks[size_t(k)] >= 0) taken2.insert(cluster_of(k, picks[size_t(k)]));
                    }
                }
                for (int k = 0; k < NA; ++k) {
                    if (!P[k].active) continue;
                    std::ostringstream c;
                    c << ",\"choice\":{\"source\":\""
                      << (lc.demo && source[k] == "orchestrator" ? "orchestrator_delayed" : source[k])
                      << "\",\"decision_id\":\"" << (lc.demo ? vdid : did)
                      << "\",\"validation\":\"" << JsonEscape(validation[k]) << "\",\"approval\":"
                      << (approval_raw[k].empty() ? "null" : "\"" + JsonEscape(approval_raw[k]) + "\"")
                      << (k == 0 ? std::string(",\"answer\":") + (ans.empty() ? "null" : ans) : std::string(",\"answer\":\"see agent 0's record\""))
                      << "}";
                    choice[k] = c.str();
                }
            }
            // Operator authority (§23): hold / goto override whatever was chosen; recorded like every directive.
            std::vector<const mis::Directive*> direct(static_cast<size_t>(NA), nullptr);
            for (const auto& d : active) {
                if ((d.kind == "hold" || d.kind == "goto") && d.agent >= 0 && d.agent < NA) direct[d.agent] = &d;
            }
            for (int k = 0; k < NA; ++k) {
                if (!P[k].active || !direct[k]) continue;
                picks[k] = -1;
                choice[k] = ",\"choice\":{\"source\":\"operator\",\"directive\":\"" + JsonEscape(direct[k]->id) + "\",\"kind\":\"" +
                            direct[k]->kind + "\"}";
            }
            // Phase 3: the command for each pick (the same code for every chooser).
            std::vector<char> sent_at_T(static_cast<size_t>(NA), false);
            for (int k = 0; k < NA; ++k) {
                Pending& pd = P[k];
                if (!pd.active) {
                    if (latest_stereo) {
                        runtime::ros_cdr::PoseArray hold;
                        hold.header.stamp.sec = int32_t(T / 1000000000);
                        hold.header.stamp.nanosec = uint32_t(T % 1000000000);
                        hold.header.frame_id = "world";
                        SendCommand(k, hold);
                        commands.Put("{\"agent\":" + std::to_string(k) + ",\"stamp_ns\":" +
                                     std::to_string(T) + ",\"poses\":0,\"reason\":\"stale_or_uninitialized_pose\"}");
                    }
                    continue;
                }
                const int pick = picks[k];
                std::ostringstream o;
                o.precision(10);
                o << pd.head;
                runtime::ros_cdr::PoseArray cmd;
                bool turn_here = false;   // D125: the command is one in-place turn (trace kind "turn")
                double turn_yaw = 0;
                cmd.header.stamp.sec = int32_t(T / 1000000000);
                cmd.header.stamp.nanosec = uint32_t(T % 1000000000);
                cmd.header.frame_id = "world";
                if (direct[k] && direct[k]->kind == "goto") {   // last resort (CERBERUS): the mover's envelope still checks it
                    runtime::ros_cdr::Pose p;
                    p.position = {direct[k]->x, direct[k]->y, brief.entry_z[size_t(k)]};
                    p.orientation = {0, 0, std::sin(0.5 * direct[k]->yaw), std::cos(0.5 * direct[k]->yaw)};
                    cmd.poses.push_back(p);
                    last_goal[k] = {direct[k]->x, direct[k]->y};
                    current[k] = {true, direct[k]->x, direct[k]->y, T, "operator goto " + direct[k]->id};
                } else if (pick >= 0) {
                    const auto& po = plan_out.agents[k].options[size_t(pick)];
                    if (po.supervisor_return && po.risk > lc.autonomous_threshold) {   // taken ungated: say so loudly
                        std::ostringstream al;
                        al.precision(3);
                        al << "{\"type\":\"alert\",\"t_s\":" << ts << ",\"agent\":" << k << ",\"text\":\"agent " << k
                           << ": energy-reserve RETURN taken without approval (risk " << po.risk << " > " << lc.autonomous_threshold
                           << "; reserve returns are exempt)\"}";
                        telemetry.Put(al.str());
                    }
                    const auto g = po.goal_xy;
                    const bool same = current[k].set && std::hypot(g.first - current[k].gx, g.second - current[k].gy) < lc.refused_radius_m;
                    current[k] = {true, g.first, g.second, same ? current[k].since_ns : T, pick_reason[k]};
                    // Route reads only the finished plan (plan fork joined above) and the briefing, never the map
                    // that worker_k may be integrating into: no need to queue behind its keyframe backlog.
                    auto [path, face_yaw] = ma ? ma->planners[size_t(k)].Route(plan_out, k, pick, pd.ac)
                                               : planner->Route(plan_out, k, pick, pd.ac);
                    if (lc.demo && path.size() > 1) {   // D169: never backtrack to where the agent was at plan time
                        core::SE3 latest;
                        bool have_latest = false;
                        {
                            std::lock_guard<std::mutex> lk2(lanes[k].mu);
                            if (!lanes[k].cams.empty()) latest = lanes[k].cams.back().second, have_latest = true;
                        }
                        if (have_latest) {
                            const core::SE3 Tw = brief.T_W_O[size_t(k)] * latest;
                            size_t nearest = 0;
                            double best = std::numeric_limits<double>::max();
                            for (size_t i = 0; i < path.size(); ++i) {
                                const double d = std::hypot(path[i].first - Tw.t.x(), path[i].second - Tw.t.y());
                                if (d < best) best = d, nearest = i;
                            }
                            path.erase(path.begin(), path.begin() + long(std::min(nearest, path.size() - 1)));
                        }
                    }
                    if (path.size() == 1 && std::isnan(path[0].first)) {   // D125: an at-cell view, a turn here
                        if (lc.face_frontier && std::isfinite(face_yaw)) {     // without face_frontier: nothing to do, hold
                            runtime::ros_cdr::Pose p;   // NaN x, y = the rig's own position (D151 amendment)
                            p.position = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                          brief.entry_z[size_t(k)]};
                            p.orientation = {0, 0, std::sin(0.5 * face_yaw), std::cos(0.5 * face_yaw)};
                            cmd.poses.push_back(p);
                            turn_here = true, turn_yaw = face_yaw;
                            last_goal[k] = g;   // the view's eye: a refusal excludes it, as for any goal
                        }
                    } else {
                        if (!path.empty()) last_goal[k] = path.back();
                        for (const auto& [x, y] : path) {
                            runtime::ros_cdr::Pose p;
                            p.position = {x, y, brief.entry_z[size_t(k)]};
                            p.orientation = {0, 0, 0, 0};   // no yaw: face the next waypoint (D151)
                            cmd.poses.push_back(p);
                        }
                        // D119: a NaN yaw = a cut command, which ends short of the view: no final turn.
                        if (!cmd.poses.empty() && lc.face_frontier && std::isfinite(face_yaw)) {
                            cmd.poses.back().orientation = {0, 0, std::sin(0.5 * face_yaw), std::cos(0.5 * face_yaw)};
                        }
                    }
                }
                const std::string result = cmd.poses.empty() ? "hold" : "go";
                o << "\"chosen\":" << pick << ",\"min_goal_m\":" << lc.min_goal_m << ",\"result\":\"" << result
                  << "\",\"command\":{\"stamp_ns\":" << T << ",\"topic\":\"" << cmd_topic.at(k) << "\",\"kind\":\""
                  << (turn_here ? "turn" : cmd.poses.empty() ? "hold" : cmd.poses.size() == 1 ? "goto" : "follow")
                  << "\",\"waypoints_xy_m\":[";
                if (!turn_here)   // the turn's NaN position stays in the PoseArray only: the trace is JSON (B's rule)
                    for (size_t i = 0; i < cmd.poses.size(); ++i) o << (i ? "," : "") << Xy(cmd.poses[i].position.x, cmd.poses[i].position.y);
                o << "]";
                if (turn_here) o << ",\"turn_yaw\":" << turn_yaw;
                o << "}" << choice[k] << "}";
                trace.Put(o.str());
                telemetry.Put("{\"type\":\"decision\",\"record\":" + o.str() + "}");
                // Command expiry (live demo only): too old by the sim clock at send time -> not sent, logged.
                const int64_t clock_now = live_clock_ns.load();
                const bool expired = latest_stereo && lc.command_max_age_s > 0 && clock_now >= 0 &&
                                     double(clock_now - T) * 1e-9 > lc.command_max_age_s;
                commands.Put("{\"agent\":" + std::to_string(k) + ",\"stamp_ns\":" + std::to_string(T) + ",\"poses\":" +
                             std::to_string(cmd.poses.size()) +
                             (expired ? ",\"expired\":true,\"clock_ns_at_send\":" + std::to_string(clock_now) : "") + "}");
                if (expired) {
                    ++commands_expired;
                    continue;
                }
                SendCommand(k, cmd);
                ++commands_sent;
                sent_at_T[k] = true;
            }
            // D165 lockstep: the world is paused at T; the commands are applied at T (the mover takes them while
            // paused) and answered BEFORE the world resumes. Every barrier resumes to T + replan, the last one too
            // (frames past end_s close the lanes).
            if (latest_stereo) write_profile(T, "demo_decision");
            else if (!is_live) write_profile(T, "replay_decision");   // the same stage split, offline (in-region replay)
            if (is_live && lc.lockstep) {
                std::vector<std::pair<int, int64_t>> wait_for;   // (agent, command stamp) answers to have before resuming
                for (int k = 0; k < NA; ++k) if (sent_at_T[k]) wait_for.push_back({k, T});
                if (T == end_ns) {   // the end of the run: every agent holds, applied at end_s while still paused
                    for (int k = 0; k < NA; ++k) {
                        runtime::ros_cdr::PoseArray hold;   // stamp end + 1 ns: a separate answer from this barrier's
                        hold.header.stamp.sec = int32_t((end_ns + 1) / 1000000000);
                        hold.header.stamp.nanosec = uint32_t((end_ns + 1) % 1000000000);
                        hold.header.frame_id = "world";
                        SendCommand(k, hold);
                        wait_for.push_back({k, end_ns + 1});
                    }
                }
                std::vector<int64_t> at(static_cast<size_t>(NA), -1);
                bool all = false;
                {
                    std::unique_lock<std::mutex> lk(progress_mu);
                    all = progress_cv.wait_for(lk, std::chrono::seconds(30), [&] {
                        if (input_done) return true;
                        for (const auto& w : wait_for) if (!arrived.count(w)) return false;
                        return true;
                    });
                    for (int k = 0; k < NA; ++k) {
                        const auto a = applied_at.find({k, T});
                        if (a != applied_at.end()) at[k] = a->second;
                    }
                }
                if (!all) ++answer_timeouts;
                const double paused_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - barrier_wall).count();
                write_profile(T, "barrier");
                std::string at_js;
                for (int k = 0; k < NA; ++k) at_js += (k ? "," : "") + std::to_string(at[size_t(k)]);
                lockstep_log.Put("{\"T_ns\":" + std::to_string(T) + ",\"applied_ns\":[" + at_js +
                                 "],\"answered\":" + (all ? "true" : "false") +
                                 ",\"wall_paused_s\":" + std::to_string(paused_s) + ",\"resume_to_ns\":" +
                                 std::to_string(std::min(T + substep_ns, end_ns + substep_ns)) + "}");
                RunTo(T + substep_ns);   // D166: one sub-step; the next barrier's wait loop paces the rest
                paced_to = T + substep_ns;
            }
        }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "ariadne_live: mission loop failed: %s\n", e.what());
            mission_err = std::current_exception();
        } catch (...) {
            std::fprintf(stderr, "ariadne_live: mission loop failed: unknown exception\n");
            mission_err = std::current_exception();
        }
        if (mission_err && is_live) ::shutdown(fd, SHUT_RDWR);   // stop the reader: the lanes drain, the joins return
        // End of the run: every agent holds (lockstep sent them at the last barrier, before resuming).
        if (is_live && !lc.lockstep && !mission_err) {
            for (int k = 0; k < NA; ++k) {
                runtime::ros_cdr::PoseArray hold;
                hold.header.stamp.sec = int32_t(end_ns / 1000000000);
                hold.header.stamp.nanosec = uint32_t(end_ns % 1000000000);
                hold.header.frame_id = "world";
                SendCommand(k, hold);
            }
        }
        // The estimators stop when the READER closes their queues (the first image past end_s, or the end of the
        // input): never earlier, or live and replay would consume different tails.
        for (auto& t : vo) t.join();
        integ_stop = true;
        if (integrator.joinable()) integrator.join();
        for (int k = 0; k < NA; ++k) {   // D168: reap the estimator workers
            if (feeders[k].joinable()) feeders[k].join();
            if (worker_fd[k] >= 0) ::close(worker_fd[k]);
            if (worker_pid[k] > 0) {
                int st = 0;
                ::waitpid(worker_pid[k], &st, 0);
                if ((!WIFEXITED(st) || WEXITSTATUS(st) != 0) && !mission_err)
                    mission_err = std::make_exception_ptr(std::runtime_error("estimator worker " +
                        std::to_string(k) + " failed (wait status " + std::to_string(st) + ")"));
            }
        }
        if (ma && ma->collab) {   // D174: the estimator tees are closed; the fleet drains, then the workers exit
            ma->collab->Shutdown();
            for (const pid_t cp : ma->collab_pids) {
                int st = 0;
                ::waitpid(cp, &st, 0);
            }
            for (const auto& sp : ma->collab_socks) ::unlink(sp.c_str());
        }
        if (is_live) ::shutdown(fd, SHUT_RDWR);
        reader.join();
        if (replay_reader_error && !mission_err) mission_err = replay_reader_error;
        rec.Close();
        if (is_live) ::close(fd);

        std::ofstream res(run_dir / "results.yaml");
        if (mission_err) res << "mission_error: true   # the loop failed; see stderr - artifacts above the failure are valid\n";
        res << "# ariadne_live results (design LIVE_RUNTIME_L1). Generated; do not hand-edit.\n"
            << "mode: " << (is_live ? (lc.demo ? "live-demo" : "live") : "replay") << "\nscene_id: " << brief.scene_id
            << "\nlive_config: " << lc.name << "\nmission_config: " << mc.name << "\nend_s: " << lc.end_s
            << "\nconsumed: {messages: " << rec.count() << ", digest_fnv1a64: '" << std::hex << rec.digest() << std::dec
            << "'}\nreplans: " << replans << "\ncommands_sent: " << (is_live ? commands_sent : 0)
            << "\ncommands_decided: " << commands_sent + commands_expired << "\ncommands_expired: " << commands_expired << "\nlockstep: " << (lc.lockstep ? "true" : "false")
            << "\nestimator_processes: " << (lc.estimator_processes ? "true" : "false")
            << "\nplanner: " << planner_name << "\nshadow_map: " << (shadow ? "p4slab (Phase 4 TSDF)" : "none")
            << "\nreorder_released_on_timeout: " << reorder_timeouts
            << "\nreorder_released_out_of_order: " << reorder_order_breaks
            << "\nunrouted_messages: " << dropped_untyped << "\nanswer_timeouts: " << answer_timeouts
            << "\nrefused_goals: " << refused_goals.size() << "\n";
        const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
        res << "wall_s: " << wall_s << "\nend_to_end_rtf: " << (wall_s > 0 ? lc.end_s / wall_s : 0)
            << "   # sim seconds per wall second, whole run incl. startup; gz's own view is gz_stats.jsonl\n"
            << "agents:\n";
        for (int k = 0; k < NA; ++k) {
            const long n = std::max(1L, tagg[k].n);
            res << "  - {agent: " << k << ", frames: " << lanes[k].frames << ", keyframes: " << lanes[k].keyframes
                << ", unpaired_frames_dropped: " << lanes[k].unpaired << ", keyframes_integrated: " << integrated[k]
                << ", image_queue_max_depth: " << lanes[k].images.max_depth()
                << ", track_mean_ms: " << 1000 * tagg[k].track / n << ", track_max_ms: " << 1000 * tagg[k].max_track
                << ", lock_mean_ms: " << 1000 * tagg[k].lock / n << ", lock_max_ms: " << 1000 * tagg[k].max_lock
                << ", queue_wait_mean_ms: " << 1000 * tagg[k].wait / n << "}\n";
        }
        res << "rays: " << rays << "\n";
        res.close();

        runtime::RunManifest man;
        man.run_id = run_dir.filename().string();
        man.created_utc = tools::UtcNow("%Y-%m-%dT%H:%M:%SZ");
        man.git_commit = std::getenv("ARIADNE_GIT_COMMIT") ? std::getenv("ARIADNE_GIT_COMMIT") : runtime::kUnknownField;
        man.git_dirty = std::getenv("ARIADNE_GIT_DIRTY") && std::string(std::getenv("ARIADNE_GIT_DIRTY")) == "true";
        man.image_digest = std::getenv("ARIADNE_IMAGE_DIGEST") ? std::getenv("ARIADNE_IMAGE_DIGEST") : runtime::kUnknownField;
        man.scene = "sim-live/" + brief.scene_id;
        man.seed_recorded = true;
        man.seed = va.seed;
        man.agent_count = 2;
        man.execution_mode = is_live ? "live" : "replay";
        man.planner_config = mc.name;
        man.extra["planner"] = "frontier candidates (1.8) + chooser " + lc.chooser;
        man.extra["live_config"] = live_cfg_path;
        if (lc.multiagent.enabled) {
            man.extra["multiagent"] = "broker_mode=" + lc.multiagent.broker_mode + " scenario=" + lc.multiagent.scenario;
        }
        man.extra["collab"] = lc.collab.enabled ? "true" : "false";
        if (lc.collab.enabled) {
            man.extra["collab_config"] = collab_config_path_used;
            {
                char hx[17];
                std::snprintf(hx, sizeof hx, "%016llx", (unsigned long long)collab_config_digest);
                man.extra["collab_config_fnv1a64"] = hx;
            }
            man.extra["collab_feature_latency_s"] = std::to_string(lc.collab.feature_latency_s);
            man.extra["collab_worker_threads"] = std::to_string(lc.collab.worker_threads);
            man.extra["collab_parent_threads"] = std::to_string(cv::getNumThreads());
            man.extra["collab_models"] = lc.collab.place_onnx + " + " + lc.collab.aliked_onnx;
        }
        man.extra["perception_config"] = va.perception_config;
        man.extra["slam_config"] = va.slam_config;
        if (!replay_path.empty()) man.extra["replay_of"] = replay_path;
        std::string cmd;
        for (int i = 0; i < argc; ++i) cmd += (i ? " " : "") + std::string(argv[i]);
        man.extra["command"] = cmd;
        runtime::RecordBuildIdentity(man);
        runtime::RecordOpenCvIdentity(man);
        runtime::RecordDeterminismDigests(man);
        std::ofstream(run_dir / "manifest.yaml") << runtime::ToYaml(man);
        if (splat && splat->on())
            std::printf("ariadne_live: splat tee %llu sends, %llu bytes, %.1f ms socket-busy (%llu dropped under "
                        "the viewer bound)\n",
                        (unsigned long long)splat->sends(), (unsigned long long)splat->sent_bytes(), splat->busy_ms(),
                        (unsigned long long)splat->drops());
        std::printf("ariadne_live[%s]: %d replans, %d decisions, consumed %llu messages (digest %016llx) -> %s\n",
                    is_live ? "live" : "replay", replans, commands_sent, (unsigned long long)rec.count(),
                    (unsigned long long)rec.digest(), run_dir.c_str());
        if (mission_err) return 4;   // the teardown above ran and the artifacts are closed; the run still FAILED
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ariadne_live: %s\n", e.what());
        return 1;
    }
}
