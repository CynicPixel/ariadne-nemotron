#pragma once
// ---------------------------------------------------------------------------
// The live runtime's input side (design live-runtime.md D152). ROS-free: messages arrive as (topic, CDR bytes)
// from the link (link_frame.hpp) or from a recording of a previous live run, and go through the SAME decoder and
// frame assembly as the bag replay (SimFrameAssembler).
//
//   StreamMessage     one ROS message: topic, type, header stamp, CDR bytes, release sequence number
//   MessageQueue      unbounded blocking FIFO between the reader and an agent's estimator thread
//   Reorderer         holds messages back until every timed input topic has delivered a stamp >= theirs
//                     (a watermark), then releases them in (stamp, topic, arrival) order: the order the
//                     estimator consumes and the recording stores. A topic silent past the hold-back timeout is
//                     skipped by the watermark, and that is logged (released_on_timeout).
//   ConsumedRecorder  appends every released message to a rosbag2-layout MCAP (RosbagReader reads it) with
//                     log time = release sequence number, strictly increasing, so a replay sees exactly the
//                     consumed order even where stamps tie (left/right images share a stamp). Also an FNV-1a digest
//                     over (topic, stamp, payload) in that order: the first replay check.
//   LiveSimSource     a StereoSource over one agent's image queue. Calibration and extrinsics from the mission
//                     JSON (a live run has no canonical bag and no /tf_static); lenient pairing (drops counted).
//                     Agents take turns per frame through a shared lock (D152a): with two estimators calling into
//                     OpenCV's one parallel_for_ pool at once, one pose in 501 differed between two replays of the same
//                     recording (PnP inliers 323 vs 322; same tracks and correspondences). [UNVERIFIED] cause: a busy
//                     pool runs the other caller's job serially, changing stripe partitioning and so the order of
//                     floating-point reductions. Turn-taking removes the contention whatever the cause.
// ---------------------------------------------------------------------------
#include "core/determinism/digests.hpp"
#include "runtime/live/message_queue.hpp"
#include "runtime/datasets/stereo_source.hpp"

#include <mcap/writer.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace ariadne::runtime::live {

/// The header stamp of a CDR message that starts with std_msgs/Header (Image, CameraInfo, Imu, PoseStamped):
/// bytes 4..12 after the encapsulation header. -1 for untimed types.
/// The mover's answer (std_msgs/String JSON): its "applied_ns" + 1, else -1 (untimed). The +1 orders an answer after
/// the sensor data of the instant it was applied, so the reorderer HOLDS every agent's answers until the watermark
/// passes and releases them in (stamp, topic) order: canonical, whichever arrived first (D165: at a paused barrier the
/// answers arrived in either order, and two EGL lockstep runs recorded them swapped, 2026-10-23).
inline int64_t AnswerStamp(const std::vector<uint8_t>& b) {
    if (b.size() < 8) return -1;
    uint32_t n;
    std::memcpy(&n, b.data() + 4, 4);
    if (b.size() < 8 + size_t(n)) return -1;
    const std::string js(reinterpret_cast<const char*>(b.data() + 8), n ? n - 1 : 0);
    const auto p = js.find("\"applied_ns\":");
    if (p == std::string::npos) return -1;
    return std::stoll(js.substr(p + 13)) + 1;
}

inline int64_t PeekStamp(const std::string& type, const std::vector<uint8_t>& b) {
    if (type == "std_msgs/msg/String") return AnswerStamp(b);
    if (b.size() < 12 || b[1] != 0x01) throw std::runtime_error("live: short or non-little-endian CDR message");
    int32_t sec;
    uint32_t nsec;
    std::memcpy(&sec, b.data() + 4, 4);
    std::memcpy(&nsec, b.data() + 8, 4);
    return int64_t(sec) * 1000000000 + nsec;
}

class Reorderer {
public:
    explicit Reorderer(std::set<std::string> timed_topics, std::chrono::milliseconds hold_back = std::chrono::seconds(3),
                       bool arrival_order = false)
        : timed_(std::move(timed_topics)), hold_back_(hold_back), arrival_order_(arrival_order) {}

    /// Push one arrival; calls release(m) for every message now safe to release, in release order.
    template <typename F>
    void Push(StreamMessage m, F&& release) {
        if (arrival_order_) {
            // Demo sensors have independent consumers: a slow camera must not
            // hold every agent's IMU behind a fleet-wide timestamp watermark.
            Count(m.stamp_ns);
            release(std::move(m));
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (m.stamp_ns < 0) {   // untimed: released at once, in arrival order
            release(std::move(m));
            return;
        }
        if (timed_.count(m.topic)) {
            auto& t = latest_[m.topic];
            t.stamp = std::max(t.stamp, m.stamp_ns);
            t.wall = now;
        }
        held_.emplace(std::make_tuple(m.stamp_ns, m.topic, arrival_++), std::move(m));
        Drain(now, release);
    }

    /// End of input: release everything still held, in order.
    template <typename F>
    void Flush(F&& release) {
        for (auto& [k, m] : held_) {
            Count(std::get<0>(k));
            release(std::move(m));
        }
        held_.clear();
    }

    long released_on_timeout() const { return timeouts_; }
    /// Releases whose stamp went BACKWARDS (a timeout skip let newer stamps out first, then the stragglers arrived):
    /// the consumed order was not canonical. 0 means the stream order was canonical end to end (2026-10-23).
    long released_out_of_order() const { return order_breaks_; }

    /// Every timed topic counts as heard from now (D165 lockstep): after a pause the runtime asked for, the silence was
    /// requested, not a stall, so the hold-back (wall time) restarts instead of releasing the first burst on timeout.
    void Refresh() {
        const auto now = std::chrono::steady_clock::now();
        for (auto& [t, l] : latest_) l.wall = now;
    }

private:
    template <typename F>
    void Drain(std::chrono::steady_clock::time_point now, F&& release) {
        int64_t wm = std::numeric_limits<int64_t>::max();
        bool timed_out = false;
        for (const auto& t : timed_) {
            const auto it = latest_.find(t);
            const bool silent = it == latest_.end() || now - it->second.wall > hold_back_;
            if (silent) {
                // Never heard from (or stalled): the watermark waits for it up to hold_back from the first arrival.
                if (it == latest_.end() && now - first_wall(now) <= hold_back_) return;
                timed_out = true;
                continue;
            }
            wm = std::min(wm, it->second.stamp);
        }
        while (!held_.empty() && std::get<0>(held_.begin()->first) <= wm) {
            if (timed_out) ++timeouts_;
            Count(std::get<0>(held_.begin()->first));
            release(std::move(held_.begin()->second));
            held_.erase(held_.begin());
        }
    }
    void Count(int64_t stamp) {
        if (stamp < 0) return;
        if (stamp < last_released_) ++order_breaks_;
        last_released_ = std::max(last_released_, stamp);
    }
    std::chrono::steady_clock::time_point first_wall(std::chrono::steady_clock::time_point now) {
        if (!first_) first_ = now;
        return *first_;
    }

    struct Latest {
        int64_t stamp = -1;
        std::chrono::steady_clock::time_point wall;
    };
    std::set<std::string> timed_;
    std::chrono::milliseconds hold_back_;
    bool arrival_order_ = false;
    std::map<std::string, Latest> latest_;
    std::map<std::tuple<int64_t, std::string, uint64_t>, StreamMessage> held_;
    uint64_t arrival_ = 0;
    long timeouts_ = 0, order_breaks_ = 0;
    int64_t last_released_ = std::numeric_limits<int64_t>::min();
    std::optional<std::chrono::steady_clock::time_point> first_;
};

class ConsumedRecorder {
public:
    void Open(const std::string& path) {
        mcap::McapWriterOptions o("ros2");
        o.compression = mcap::Compression::Zstd;
        const auto s = w_.open(path, o);
        if (!s.ok()) throw std::runtime_error("live: cannot open recording " + path + ": " + s.message);
        open_ = true;
    }
    /// Digest only (replay: the consumed sequence is re-derived and compared, nothing is written).
    void Absorb(const StreamMessage& m) {
        for (char c : m.topic) h_.AbsorbByte(uint8_t(c));
        h_.AbsorbU64(uint64_t(m.stamp_ns));
        h_.AbsorbU64(m.data->size());
        for (uint8_t b : *m.data) h_.AbsorbByte(b);
        ++count_;
    }
    void Write(const StreamMessage& m) {
        Absorb(m);
        if (!open_) return;
        auto it = channels_.find(m.topic);
        if (it == channels_.end()) {
            mcap::Schema sc(m.type, "ros2msg", "");
            w_.addSchema(sc);
            mcap::Channel ch(m.topic, "cdr", sc.id);
            w_.addChannel(ch);
            it = channels_.emplace(m.topic, ch.id).first;
        }
        mcap::Message r;
        r.channelId = it->second;
        r.sequence = uint32_t(m.seq);
        r.logTime = mcap::Timestamp(m.seq);   // the consumed order, not a clock (D152)
        r.publishTime = mcap::Timestamp(m.stamp_ns < 0 ? 0 : m.stamp_ns);
        r.dataSize = m.data->size();
        r.data = reinterpret_cast<const std::byte*>(m.data->data());
        const auto s = w_.write(r);
        if (!s.ok()) throw std::runtime_error("live: recording write failed: " + s.message);
    }
    void Close() {
        if (open_) w_.close();
        open_ = false;
    }
    ~ConsumedRecorder() { Close(); }
    uint64_t digest() const { return h_.value(); }
    uint64_t count() const { return count_; }

private:
    mcap::McapWriter w_;
    bool open_ = false;
    std::map<std::string, mcap::ChannelId> channels_;
    core::Fnv1a64 h_;
    uint64_t count_ = 0;
};

/// Thread-safe ground-truth store (scoring only; the estimator never reads it).
class GtStore {
public:
    void Add(const TimedPose& p) {
        std::lock_guard<std::mutex> lk(mu_);
        v_.push_back(p);
    }
    std::vector<TimedPose> Snapshot() const {
        std::lock_guard<std::mutex> lk(mu_);
        auto v = v_;
        std::stable_sort(v.begin(), v.end(), [](const TimedPose& a, const TimedPose& b) { return a.t_ns < b.t_ns; });
        return v;
    }
    /// Ground-truth path length (m, 3D) over the poses stamped at or before `until_ns`: the rig's executed motion.
    double PathLength(int64_t until_ns) const {
        const auto v = Snapshot();
        double L = 0;
        for (size_t i = 1; i < v.size() && v[i].t_ns <= until_ns; ++i) L += (v[i].T.t - v[i - 1].T.t).norm();
        return L;
    }

private:
    mutable std::mutex mu_;
    std::vector<TimedPose> v_;
};

class LiveSimSource : public StereoSource {
public:
    /// `turn`: held while one frame runs through the estimator (D152a). Every agent's source shares it, so frames are
    /// processed one at a time, in whatever order, and each OpenCV call sees an idle thread pool.
    /// `lens_on`: apply the read-time D41 lens to every frame. Off (the live runtime's default since the D41
    /// amendment, D227), frames stay the clean pinhole render and Calibration() reports the RENDER intrinsics
    /// with D = 0 - the camera the estimator then actually sees.
    /// `pipelined` (D229, worker mode only): frame n+1's ingest (decode -> lens -> D -> gray) runs on one
    /// dedicated thread, handed to the VO thread through a depth-1 in-order slot. Content is bit-identical to the
    /// single-threaded shape (the unit guard holds the line); only timing overlaps. Depth 1 keeps the D166 pacing
    /// backpressure: at most one frame is pre-ingested ahead of the estimator.
    LiveSimSource(const std::string& scene, int agent, MessageQueue& images, MessageQueue& imus, const GtStore& gt, std::mutex& turn,
                  const std::string& mission = "live", bool lens_on = true, bool pipelined = false)
        : agent_(agent), images_(images), imus_(imus), gt_(gt), turn_(turn), lens_on_(lens_on), pipelined_(pipelined) {
        const YAML::Node sc = YAML::LoadFile(scene + "/scene.json");
        const YAML::Node m = YAML::LoadFile(scene + "/mission_" + mission + ".json");
        scene_id_ = sc["scene_id"].as<std::string>();
        rig_ = m["rig"]["name"].as<std::string>();
        YAML::Node ag;
        for (const auto& x : m["agents"]) {
            if (x["agent"].as<int>() == agent) ag.reset(x);
        }
        if (!ag) throw std::runtime_error("live: agent " + std::to_string(agent) + " not in the mission");
        t_left_ = ag["topics"]["left_image"].as<std::string>();
        t_right_ = ag["topics"]["right_image"].as<std::string>();
        t_imu_ = ag["topics"]["imu"].as<std::string>();
        const auto L = m["lens"];
        const auto R = m["render_intrinsics_pixel_centre"];
        lens_.width = m["rig"]["camera"]["width"].as<int>();
        lens_.height = m["rig"]["camera"]["height"].as<int>();
        lens_.fx = L["fx"].as<double>();
        lens_.fy = L["fy"].as<double>();
        lens_.cx = L["cx"].as<double>();
        lens_.cy = L["cy"].as<double>();
        lens_.k1 = L["k1"].as<double>();
        lens_.k2 = L["k2"].as<double>();
        lens_.p1 = L["p1"].as<double>();
        lens_.p2 = L["p2"].as<double>();
        lens_.k3 = L["k3"].as<double>();
        lens_.render_fx = R["fx"].as<double>();
        lens_.render_fy = R["fy"].as<double>();
        lens_.render_cx = R["cx"].as<double>();
        lens_.render_cy = R["cy"].as<double>();
        if (lens_.k3 != 0.0) throw std::runtime_error("live: PinholeRadtan carries k1 k2 p1 p2 only; rig.lens.k3 != 0");
        if (lens_on_) map_ = perception::BuildLensMap(lens_);   // off: no map, the per-frame resampling never runs
        // T_B_optical from the mission manifest: the same numbers the canonical /tf_static is built from and
        // checked against (D40), which a live run does not have.
        for (const char* side : {"left", "right"}) {
            core::SE3 T;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) T.R(i, j) = m["R_B_optical_rows"][i][j].as<double>();
                T.t(i) = m["t_B_camera_m"][side][i].as<double>();
            }
            (std::string(side) == "left" ? T_B_left_ : T_B_right_) = T;
        }
    }

    std::string RunName() const override { return scene_id_ + "-agent" + std::to_string(agent_); }
    std::string SceneName() const override { return "sim-live/" + scene_id_ + "/agent" + std::to_string(agent_); }
    std::string CameraConfig() const override { return rig_ + (lens_on_ ? "+lens(D41)" : "+lens(off)"); }
    std::string BagChecksum() const override { return kUnknownField; }   // the run records its consumed digest
    perception::RawStereoCalibration Calibration() const override {
        perception::PinholeRadtan c;
        c.width = lens_.width;
        c.height = lens_.height;
        if (lens_on_) {
            c.fu = lens_.fx;
            c.fv = lens_.fy;
            c.cu = lens_.cx;
            c.cv = lens_.cy;
            c.dist = {lens_.k1, lens_.k2, lens_.p1, lens_.p2};
        } else {   // the frames ARE the clean render: its pinhole, zero distortion (D227)
            c.fu = lens_.render_fx;
            c.fv = lens_.render_fy;
            c.cu = lens_.render_cx;
            c.cv = lens_.render_cy;
            c.dist = {0, 0, 0, 0};
        }
        perception::RawStereoCalibration s;
        s.left = s.right = c;
        s.T_right_left = T_B_right_.inverse() * T_B_left_;
        return s;
    }
    core::SE3 T_B_C0() const override { return T_B_left_; }
    std::vector<TimedPose> GroundTruth() const override { return {}; }   // late: StereoVoArgs::late_ground_truth
    size_t FrameCount() const override { return 0; }                     // unknown ahead of time
    uint32_t Agent() const override { return uint32_t(agent_); }

    std::vector<core::ImuMeasurement> Imu() override {
        if (imu_.empty() && !ReadOneImu()) throw std::runtime_error("live: IMU stream closed before its first sample");
        return imu_;
    }
    void RefreshImu(int64_t t_ns, std::vector<core::ImuMeasurement>& dst) override {
        if (&dst == &imu_) throw std::logic_error("live: caller must own the IMU vector");
        while (imu_.empty() || imu_.back().t_ns < t_ns) {
            if (!ReadOneImu()) {
                if (imu_.empty() || imu_.back().t_ns < t_ns)
                    throw std::runtime_error("live: IMU stream ended before frame timestamp " + std::to_string(t_ns) +
                                             "; last IMU=" + std::to_string(imu_.empty() ? -1 : imu_.back().t_ns));
                break;
            }
        }
        if (dst.size() > imu_.size()) throw std::logic_error("live: caller IMU vector is ahead of source");
        dst.insert(dst.end(), imu_.begin() + static_cast<std::ptrdiff_t>(dst.size()), imu_.end());
    }
    core::ImuNoise ImuNoiseModel() const override { return imu_noise_ ? imu_noise_->noise : core::ImuNoise{}; }
    std::string ImuNoiseConfig() const override { return imu_noise_ ? imu_noise_->name : "none"; }
    void SetImuNoise(const core::ImuNoiseOperatorParams& p, uint64_t run_seed) override {
        const uint64_t seed = core::DeriveSeed(core::DeriveSeed(run_seed, core::seed_purpose::kImuNoise),
                                               ("agent" + std::to_string(agent_)).c_str());
        imu_noise_ = p;
        imu_noise_stream_ = std::make_unique<core::ImuNoiseStream>(p, seed);
    }

    /// D167 timers, per emitted frame: how long this lane sat idle (queue wait), waited for the other agent's
    /// turn (the D152a lock), and tracked (Feed, the estimator inside). Sums since the previous emission, so a
    /// frame's cost includes the handling of its own unpaired half.
    std::function<void(int64_t t_ns, double wait_s, double lock_s, double track_s)> on_timing;

    size_t ForEachFrame(size_t max_frames, const std::function<void(const StereoFrame&)>& on_frame) override {
        SimFrameAssembler assembler(lens_on_ ? &map_ : nullptr, uint32_t(agent_), t_left_, t_right_, /*strict=*/false);
        using clk = std::chrono::steady_clock;
        auto sec = [](clk::time_point a, clk::time_point b) { return std::chrono::duration<double>(b - a).count(); };
        double wait_s = 0, lock_s = 0, track_s = 0;
        int64_t t_ns = -1;
        const std::function<void(const StereoFrame&)> wrapped = [&](const StereoFrame& f) {
            t_ns = f.t_ns;
            on_frame(f);
        };
        // Pair + track one ingested image under the turn lock; true = max_frames reached.
        auto apply = [&](SimFrameAssembler::Ingested&& in) {
            const auto t1 = clk::now();
            std::unique_lock<std::mutex> lk(turn_);
            const auto t2 = clk::now();
            const bool emitted = assembler.Apply(std::move(in), wrapped);
            const auto t3 = clk::now();
            lk.unlock();
            lock_s += sec(t1, t2), track_s += sec(t2, t3);
            if (emitted) {
                if (on_timing) on_timing(t_ns, wait_s, lock_s, track_s);
                wait_s = lock_s = track_s = 0;
                if (assembler.emitted() == max_frames) return true;
            }
            return false;
        };
        if (!pipelined_) {
            for (;;) {
                const auto t0 = clk::now();
                auto m = images_.Pop();
                if (!m) break;
                wait_s += sec(t0, clk::now());
                if (apply(assembler.Ingest(m->topic, m->data->data(), m->data->size()))) break;
            }
        } else {
            // D229: the ingest thread runs Ingest for message n+1 while the VO thread tracks frame n. The slot is
            // depth 1 and in order; Ingest's only cross-message state (D's arrival counters) stays on this one
            // thread in delivery order, so the emitted bytes equal the single-threaded shape exactly.
            std::mutex mu;
            std::condition_variable cv;
            std::optional<SimFrameAssembler::Ingested> slot;
            bool eof = false, stop = false;
            std::exception_ptr err;
            std::thread ingest([&] {
                try {
                    while (auto m = images_.Pop()) {
                        auto in = assembler.Ingest(m->topic, m->data->data(), m->data->size());
                        std::unique_lock<std::mutex> lk(mu);
                        cv.wait(lk, [&] { return !slot.has_value() || stop; });
                        if (stop) break;
                        slot.emplace(std::move(in));
                        cv.notify_all();
                    }
                } catch (...) {
                    std::lock_guard<std::mutex> lk(mu);
                    err = std::current_exception();
                }
                {
                    std::lock_guard<std::mutex> lk(mu);
                    eof = true;
                }
                cv.notify_all();
            });
            for (;;) {
                const auto t0 = clk::now();
                std::optional<SimFrameAssembler::Ingested> in;
                {
                    std::unique_lock<std::mutex> lk(mu);
                    cv.wait(lk, [&] { return slot.has_value() || eof; });
                    if (!slot.has_value()) break;   // eof with nothing pending
                    in.emplace(std::move(*slot));
                    slot.reset();
                }
                cv.notify_all();
                wait_s += sec(t0, clk::now());   // D167: the lane's idle time is now the slot wait
                if (apply(std::move(*in))) {
                    {
                        std::lock_guard<std::mutex> lk(mu);
                        stop = true;
                    }
                    cv.notify_all();
                    break;
                }
            }
            ingest.join();
            if (err) std::rethrow_exception(err);
        }
        unpaired_ = assembler.unpaired() + assembler.pending();
        return assembler.emitted();
    }

    const std::string& left_topic() const { return t_left_; }
    const std::string& right_topic() const { return t_right_; }
    const std::string& imu_topic() const { return t_imu_; }
    size_t unpaired() const { return unpaired_; }

private:
    int agent_;
    MessageQueue& images_;
    MessageQueue& imus_;
    const GtStore& gt_;
    std::mutex& turn_;
    std::string scene_id_, rig_, t_left_, t_right_, t_imu_;
    perception::LensModel lens_;
    perception::LensMap map_;   // empty when lens_on_ is false
    bool lens_on_ = true;
    bool pipelined_ = false;    // D229
    std::vector<core::ImuMeasurement> imu_;
    std::optional<core::ImuNoiseOperatorParams> imu_noise_;
    std::unique_ptr<core::ImuNoiseStream> imu_noise_stream_;
    core::SE3 T_B_left_, T_B_right_;
    size_t unpaired_ = 0;

    bool ReadOneImu() {
        auto msg = imus_.Pop();
        if (!msg) return false;
        const auto decoded = ros_cdr::DecodeImu(msg->data->data(), msg->data->size());
        core::ImuMeasurement s;
        s.t_ns = decoded.header.stamp.ns();
        s.gyro = core::Vec3(decoded.angular_velocity.x, decoded.angular_velocity.y, decoded.angular_velocity.z);
        s.accel = core::Vec3(decoded.linear_acceleration.x, decoded.linear_acceleration.y, decoded.linear_acceleration.z);
        if (imu_noise_stream_) s = imu_noise_stream_->Apply(s);
        if (!imu_.empty() && s.t_ns <= imu_.back().t_ns)
            throw std::runtime_error("live: IMU timestamps are not strictly increasing");
        imu_.push_back(std::move(s));
        return true;
    }
};

}  // namespace ariadne::runtime::live
