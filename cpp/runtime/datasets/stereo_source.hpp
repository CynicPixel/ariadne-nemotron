#pragma once
// ---------------------------------------------------------------------------
// One stereo input interface for the 1.4/1.5 pipeline, two implementations:
//
//   EurocSource  the EuRoC ASL folders (D35)
//   SimSource    a simulator recording (D39/D40) + its scene directory: frames
//                from sensors/, ground truth and static TF from canonical/,
//                every frame passed through the read-time LENS (D41) before
//                it is converted to gray. Calibration comes from rig.lens via
//                the mission manifest, never from the recorded camera_info
//                (whose D is zero and whose cx, cy are half a pixel off,
//                SMOKE §4n). ROS-free: RosbagReader + ros_cdr.
//
// A source hands out RAW (unrectified, still distorted) 8-bit gray stereo pairs
// in time order plus the raw calibration, exactly what EuRoC provides; the
// rectifier then exercises the undistort path on both.
//
// The sensor stages of Degrade() (2.2, D66): opt-in per run (SimSource::
// SetDegradation with a named config), applied to the RGB frame after the lens
// and before the gray conversion -- the order a real camera has (optics, then
// the sensor). Each (agent, camera) gets its own D seed, so the two cameras do
// not carry the same noise and PRNU fields. The dust stage gets GT depth for
// both cameras (D142: the right one warped from the left depth camera, then
// both through the lens like their images). Without SetDegradation the
// manifest records degradation_config = "none" and every pixel is as before.
// ---------------------------------------------------------------------------
#include "core/inertial/imu_noise_operator.hpp"
#include "core/random/seed.hpp"
#include "perception/degradation/degradation_config.hpp"
#include "perception/degradation/lens.hpp"
#include "perception/degradation/lens_parallel.hpp"
#include "perception/radiometric/flat_field.hpp"
#include "perception/rgbd/depth_stereo.hpp"
#include "perception/stereo/camera_model.hpp"
#include "runtime/datasets/sim_depth.hpp"
#include "runtime/datasets/euroc.hpp"
#include "runtime/manifest/run_manifest.hpp"
#include "runtime/metrics/trajectory_io.hpp"
#include "runtime/replay/ros_cdr.hpp"
#include "runtime/replay/rosbag_reader.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <cstring>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <map>
#include <set>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ariadne::runtime {

/// §4.1 `photo` (5.2 D146, 5.3 D147): the photometric state the frame was ACTUALLY taken with -- the report half of
/// the actuator contract (D147). Stage 4 (exposure x gain), Stage 3 (the lamp power), the known-gain LK ratio and
/// the exposure controller read only this, never a command. Set only when the pixels are LINEAR in radiance: in
/// sim, when D decodes the sRGB render (input_transfer: srgb, D130). Absent means "not known to be linear".
struct FramePhoto {
    double exposure_rel = 1.0;    // relative to the rig's nominal exposure (seconds = exposure_rel x t_ref_s)
    double gain = 1.0;            // linear analog gain
    double digital_gain = 1.0;    // D's digital gain (fixed by the config; not actuated)
    double lamp_rel = 1.0;        // lamp power relative to the lamp calibration's intensity_ref (replay: D lamp_power)
    double radiance_scale = 1.0;  // D's radiance scale (fixed by the config; scales the lamp's light in a lamp-only scene)
    /// The newest command this frame reflects, in whole or in part (0: none yet), and the stamp at which it reached
    /// the actuator (-1: none). Replay applies a command on the next frame, so applied_ns is that frame's stamp.
    uint64_t decision_id = 0;
    int64_t applied_ns = -1;
    bool clamped = false;         // the sink applied something other than the command
    std::string clamp_reason;

    double ExposureGain() const { return exposure_rel * gain * digital_gain; }
    double LampScale() const { return radiance_scale * lamp_rel; }
    /// What multiplies every pixel's signal: the known brightness ratio between two frames of one scene.
    double Brightness() const { return ExposureGain() * LampScale(); }
};

/// The command half of the actuator contract (5.3, D147): what the controller asks for. A sink applies commands in
/// decision_id order; a newer command supersedes a pending older one (the older is dropped, never applied late).
struct PhotometricCommand {
    uint64_t decision_id = 0;   // increasing, from 1
    int64_t decided_ns = 0;     // the stamp of the frame the decision was made on
    double exposure_rel = 1.0, gain = 1.0, lamp_rel = 1.0;   // the FramePhoto units
};

/// The carried lamp as the recording rendered it, and whether it was the only light (5.2, D146).
struct LampRig {
    std::string rig;               // the rig name, matched against the lamp calibration's
    double intensity = 0.0;        // the render's lamp intensity (gz <intensity>)
    core::Vec3 in_body_m = core::Vec3::Zero();
    bool lamp_only = false;        // scene ambient 0 and no world lights: every photon came from the lamp
};

/// §4.1 Frame, the subset Phase 1 produces. Not modeled yet, and left absent rather than defaulted:
///   imu_window    (§10; every Phase 1 run is imu_mode none)
///   illumination  (§6.5 lamp state beyond `photo`, Phase 5)
///   gt            ground truth never rides on the estimator's frame; sources expose it separately
///                 (GroundTruth()) for scoring only.
struct StereoFrame {
    uint32_t agent = 0;    // the source's agent (EuRoC: 0)
    uint64_t id = 0;       // 0-based index in this source's time-ordered stream
    int64_t t_ns = 0;
    cv::Mat left, right;   // raw 8-bit gray
    cv::Mat left_rgb;      // raw 8-bit RGB of the left camera (same pixels as `left`); empty if the source has none
    /// RGB-D sources only (2.6, D100): CV_32F metres registered to `left`'s pixels, 0 = no return. `right` is then
    /// empty. Empty for stereo sources.
    cv::Mat depth;
    std::optional<FramePhoto> photo;   // 5.2, D146: set only when the pixels are linear in radiance
    /// EVALUATION ONLY (5.5, D148): the GT depth on the raw left grid (CV_32F metres), set only when the source was
    /// asked for it (SimSource::SetGtDepth). Perception never reads it; the runner's depth_eval block does.
    cv::Mat gt_depth;
};

class StereoSource {
public:
    virtual ~StereoSource() = default;
    virtual std::string RunName() const = 0;      // run directory stem, e.g. MH_01_easy
    virtual std::string SceneName() const = 0;    // manifest scene, e.g. EuRoC/MH_01_easy
    virtual std::string CameraConfig() const = 0;
    virtual std::string BagChecksum() const = 0;  // "sha256:..." or UNRECORDED
    virtual perception::RawStereoCalibration Calibration() const = 0;
    virtual core::SE3 T_B_C0() const = 0;         // body <- raw left camera
    virtual std::vector<TimedPose> GroundTruth() const = 0;   // T_W_B, time-sorted
    virtual size_t FrameCount() const = 0;
    virtual uint32_t Agent() const = 0;           // stamped on every StereoFrame
    /// Calls on_frame for the first max_frames pairs (all if 0), in time order.
    virtual size_t ForEachFrame(size_t max_frames, const std::function<void(const StereoFrame&)>& on_frame) = 0;
    /// The body (IMU) stream, time-sorted, and its noise model (§10.2; plan 2.4). Empty when the source has no IMU.
    virtual std::vector<core::ImuMeasurement> Imu() { return {}; }
    /// Append source samples through `t_ns` to `dst`; live sources block until covered,
    /// recorded sources can keep the default no-op after returning their full vector.
    virtual void RefreshImu(int64_t /*t_ns*/, std::vector<core::ImuMeasurement>& /*dst*/) {}
    virtual core::ImuNoise ImuNoiseModel() const { return {}; }
    virtual void SetImuNoise(const core::ImuNoiseOperatorParams& /*p*/, uint64_t /*run_seed*/) {
        throw std::logic_error("this source cannot apply an IMU noise operator");
    }
    /// Ground-truth velocity and biases, time-sorted, for VALIDATING the inertial initialisation only (never fed to
    /// the estimator). Empty when the source has none.
    virtual std::vector<GroundTruthInertial> GroundTruthInertialStates() const { return {}; }
    /// The named degradation config applied to the frames (§5.6), "none" if none (2.2).
    virtual std::string DegradationConfig() const { return "none"; }
    /// Camera `cam`'s (0 left, 1 right) calibrated flat field for the radiometric stage; empty (K = 1) when the
    /// source has none -- EuRoC publishes none (2.2).
    virtual perception::radiometric::FlatField FlatFieldOf(int /*cam*/) const { return {}; }
    /// RGB-D sources (2.6, D100): the depth sensor's stated noise model and the virtual rig it is fed through.
    /// Present iff every frame carries `depth` and no `right`. Calibration() is then that virtual rig.
    virtual std::optional<perception::DepthSensorModel> DepthSensor() const { return std::nullopt; }
    /// The named IMU noise operator config applied to Imu() (L2, D101), "none" if none.
    virtual std::string ImuNoiseConfig() const { return "none"; }
    /// The carried lamp (5.2, D146); absent when the source has none or does not record it (EuRoC, Hilti).
    virtual std::optional<LampRig> Lamp() const { return std::nullopt; }
    /// The actuator seam (5.3, D147). A source that can act on exposure, gain and lamp says so; SetPhotometric then
    /// takes effect on a LATER frame, whose `photo` reports what was applied. A source that cannot actuate throws.
    virtual bool CanActuatePhotometric() const { return false; }
    virtual void SetPhotometric(const PhotometricCommand& /*cmd*/) {
        throw std::logic_error("this source has no photometric actuator");
    }
};

// --- EuRoC ---------------------------------------------------------------------

class EurocSource : public StereoSource {
public:
    explicit EurocSource(const std::string& sequence) : seq_(sequence), mav_(seq_ / "mav0") {
        cam0_ = LoadEurocCamera((mav_ / "cam0/sensor.yaml").string());
        cam1_ = LoadEurocCamera((mav_ / "cam1/sensor.yaml").string());
        pairs_ = PairByTimestamp(ParseEurocImageCsv(ReadTextFile((mav_ / "cam0/data.csv").string()), "cam0"),
                                 ParseEurocImageCsv(ReadTextFile((mav_ / "cam1/data.csv").string()), "cam1"));
        // Optional: an EuRoC-layout export of a dataset with only sparse GT (Hilti 2023, D141) has no
        // state_groundtruth_estimate0; the source then reports no GT and the runner scores nothing densely.
        if (std::filesystem::exists(GtCsv())) {
            for (const auto& g : ParseEurocGroundTruth(ReadTextFile(GtCsv()), "groundtruth")) {
                gt_.push_back({g.t_ns, g.T_W_B});
            }
        }
    }
    std::string RunName() const override { return seq_.filename().string(); }
    std::string SceneName() const override { return "EuRoC/" + seq_.filename().string(); }
    std::vector<core::ImuMeasurement> Imu() override {
        return ParseEurocImu(ReadTextFile((mav_ / "imu0/data.csv").string()), "imu0");
    }
    core::ImuNoise ImuNoiseModel() const override {
        return ParseEurocImuNoise(ReadTextFile((mav_ / "imu0/sensor.yaml").string()), "imu0/sensor.yaml");
    }
    std::vector<GroundTruthInertial> GroundTruthInertialStates() const override {
        if (!std::filesystem::exists(GtCsv())) return {};
        return ParseEurocGroundTruthInertial(ReadTextFile(GtCsv()), "groundtruth");
    }
    std::string CameraConfig() const override { return "euroc_cam0_cam1"; }
    std::string BagChecksum() const override {
        const auto prov = seq_.parent_path() / (seq_.filename().string() + ".provenance.json");
        return std::filesystem::exists(prov) ? "sha256:" + YAML::LoadFile(prov.string())["sha256"].as<std::string>()
                                             : std::string(kUnknownField);
    }
    perception::RawStereoCalibration Calibration() const override { return EurocStereo(cam0_, cam1_); }
    core::SE3 T_B_C0() const override { return cam0_.T_B_C; }
    std::vector<TimedPose> GroundTruth() const override { return gt_; }
    size_t FrameCount() const override { return pairs_.t_ns.size(); }
    uint32_t Agent() const override { return 0; }
    size_t ForEachFrame(size_t max_frames, const std::function<void(const StereoFrame&)>& on_frame) override {
        const size_t n = max_frames ? std::min(max_frames, FrameCount()) : FrameCount();
        for (size_t i = 0; i < n; ++i) {
            StereoFrame f;
            f.agent = 0;
            f.id = i;
            f.t_ns = pairs_.t_ns[i];
            f.left = cv::imread((mav_ / "cam0/data" / pairs_.left[i]).string(), cv::IMREAD_GRAYSCALE);
            f.right = cv::imread((mav_ / "cam1/data" / pairs_.right[i]).string(), cv::IMREAD_GRAYSCALE);
            if (f.left.empty() || f.right.empty()) throw std::runtime_error("failed to read frame " + std::to_string(i));
            on_frame(f);
        }
        return n;
    }

private:
    std::string GtCsv() const { return (mav_ / "state_groundtruth_estimate0/data.csv").string(); }
    std::filesystem::path seq_, mav_;
    EurocCamera cam0_, cam1_;
    StereoPairs pairs_;
    std::vector<TimedPose> gt_;
};

// --- Replay actuator (5.3, D147) ------------------------------------------------------

/// D's photometric state in the FramePhoto units (no command applied yet).
inline FramePhoto PhotoOf(const perception::DegradationParams& d) {
    FramePhoto p;
    p.exposure_rel = d.exposure;
    p.gain = d.analog_gain;
    p.digital_gain = d.digital_gain;
    p.lamp_rel = d.lamp_power;
    p.radiance_scale = d.radiance_scale;
    return p;
}

/// Apply a command to D from the frame stamped `t_ns` on, and report it. D itself has no limits (the controller
/// keeps its own), so replay never clamps; a live sink may.
inline void ApplyPhotometric(perception::DegradationParams& d, const PhotometricCommand& cmd, int64_t t_ns,
                             FramePhoto& photo) {
    d.exposure = cmd.exposure_rel;
    d.analog_gain = cmd.gain;
    d.lamp_power = cmd.lamp_rel;
    photo = PhotoOf(d);
    photo.decision_id = cmd.decision_id;
    photo.applied_ns = t_ns;
}

/// The scripted overexposure event of Failure E (5.3, D147): from `at_s` after the first frame on, the scene's
/// radiance is 2^stops brighter (a light comes on, a reflective surface fills the view). It acts on the SCENE, before
/// the sensor, and is NOT reported in FramePhoto: the system does not know the scene changed, so the controller has to
/// see it in the pixels and respond. (A step on D's exposure would be the controller's own actuator, overwritten by its
/// next command.)
struct RadianceEvent {
    double at_s = 0.0;
    double stops = 0.0;
};

/// The radiance factor the event applies to a frame `t_rel_ns` after the first one (1 before it, or with no event).
inline double RadianceEventFactor(const std::optional<RadianceEvent>& e, int64_t t_rel_ns) {
    if (!e || 1e-9 * double(t_rel_ns) < e->at_s) return 1.0;
    return std::exp2(e->stops);
}

// --- Simulator recording ---------------------------------------------------------

/// Raw stereo frame assembly from the sim's CDR images, shared by the bag replay (SimSource) and the live runtime
/// (live/live_source.hpp, design LIVE_RUNTIME_L1 D152): decode -> lens (D41; map null = lens off, the frame stays
/// the clean pinhole render - the live runtime's default since the D41 amendment, D227) -> D (2.2, when set) ->
/// gray, then pair
/// left/right by header stamp. Feed() one image message at a time, in delivery order.
///   strict (bags): a pair completing before an older pending stamp means a frame is missing -> throws.
///   lenient (live): the older, unpaired frames are dropped and counted (unpaired()); the run goes on.
/// The sensor-stage D of SimFrameAssembler (2.2, D66): params null = no D.
struct SimDegrade {
    const perception::DegradationParams* params = nullptr;
    uint64_t seed[2] = {0, 0};
    int threads = 1;
};

class SimFrameAssembler {
public:
    using Degrade = SimDegrade;

    SimFrameAssembler(const perception::LensMap* map, uint32_t agent, std::string t_left, std::string t_right,
                      bool strict, Degrade d = {})
        : map_(map), agent_(agent), t_left_(std::move(t_left)), t_right_(std::move(t_right)), strict_(strict), d_(d) {}

    /// One decoded, lens/degrade/gray-processed image: the per-message half of Feed (D229), safe to build on a
    /// dedicated ingest thread while Apply pairs and emits on the VO thread. The only cross-message state Ingest
    /// touches is D's per-side arrival counter, so Ingest calls must stay in delivery order (one ingest thread).
    struct Ingested {
        bool left = false;
        int64_t t_ns = 0;
        cv::Mat gray;
        cv::Mat rgb;   // left only: the post-lens/degrade RGB (console + splat tee)
    };

    Ingested Ingest(std::string_view topic, const uint8_t* data, size_t size) {
        const bool left = topic == t_left_;
        if (!left && topic != t_right_) throw std::logic_error("sim: assembler fed a foreign topic");
        const auto im = ros_cdr::DecodeImage(data, size);
        ros_cdr::ValidateImage(im);
        if (im.encoding != "rgb8") throw std::runtime_error("sim: expected rgb8, got " + im.encoding);
        core::ImageU8 clean(int(im.width), int(im.height), 3);
        for (uint32_t r = 0; r < im.height; ++r) {
            std::memcpy(&clean.data[size_t(r) * im.width * 3], &im.data[size_t(r) * im.step], im.width * 3);
        }
        core::ImageU8 seen = map_ ? perception::ApplyLensParallel(clean, *map_) : std::move(clean);
        if (d_.params) {
            const int c = left ? 0 : 1;
            seen = perception::DegradeParallel(seen, *d_.params, d_.seed[c], arrived_[c]++, d_.threads);
        }
        Ingested out;
        out.left = left;
        out.t_ns = im.header.stamp.ns();
        cv::cvtColor(cv::Mat(int(im.height), int(im.width), CV_8UC3, seen.data.data()), out.gray, cv::COLOR_RGB2GRAY);
        if (left) out.rgb = cv::Mat(int(im.height), int(im.width), CV_8UC3, seen.data.data()).clone();
        return out;
    }

    /// Pairing + emission (VO thread). Returns true if a frame was emitted through on_frame.
    bool Apply(Ingested&& in, const std::function<void(const StereoFrame&)>& on_frame) {
        const int64_t t = in.t_ns;
        if (t <= last_emitted_) {   // a side arriving after its stamp was emitted or dropped
            if (strict_) throw std::runtime_error("sim: image at " + std::to_string(t) + " after its stamp was emitted");
            ++unpaired_;
            return false;
        }
        StereoFrame& f = pending_[t];
        f.t_ns = t;
        (in.left ? f.left : f.right) = in.gray;
        if (in.left) f.left_rgb = in.rgb;
        if (f.left.empty() || f.right.empty()) return false;
        // Complete pair: it must be the oldest pending stamp, or a frame went missing.
        if (pending_.begin()->first != t) {
            if (strict_) {
                throw std::runtime_error("sim: stereo pair at " + std::to_string(t) +
                                         " completed before an older stamp: a frame is missing");
            }
            while (pending_.begin()->first != t) {
                pending_.erase(pending_.begin());
                ++unpaired_;
            }
        }
        last_emitted_ = t;
        StereoFrame done = std::move(f);
        pending_.erase(t);
        done.agent = agent_;
        done.id = n_++;
        on_frame(done);
        return true;
    }

    /// Returns true if a frame was emitted through on_frame (= Apply(Ingest(...)), the single-threaded shape).
    bool Feed(std::string_view topic, const uint8_t* data, size_t size,
              const std::function<void(const StereoFrame&)>& on_frame) {
        return Apply(Ingest(topic, data, size), on_frame);
    }

    size_t emitted() const { return n_; }
    size_t pending() const { return pending_.size(); }
    size_t unpaired() const { return unpaired_; }

private:
    const perception::LensMap* map_;
    uint32_t agent_;
    std::string t_left_, t_right_;
    bool strict_;
    Degrade d_;
    std::map<int64_t, StereoFrame> pending_;   // stamps with only one side so far
    int64_t last_emitted_ = -1;
    size_t n_ = 0, unpaired_ = 0;
    uint64_t arrived_[2] = {0, 0};             // D's frame id: per-camera arrival index (= the pair index)
};

class SimSource : public StereoSource {
public:
    /// `right_camera` picks the rig's right camera by name (5.5, D138): "right" (the main pair) or one of a sweep
    /// rig's right_b04/b08/b12/b20. Its topic and its extrinsics come from the mission and /tf_static like the main
    /// pair's (sim_record checked the TF against the manifest's t_B_camera_m exactly), so the baseline follows.
    SimSource(const std::string& rec, const std::string& scene, int agent, const std::string& mission = "default",
              const std::string& right_camera = "right")
        : rec_(rec), agent_(agent), right_camera_(right_camera) {
        const YAML::Node sc = YAML::LoadFile(scene + "/scene.json");
        const YAML::Node m = YAML::LoadFile(scene + "/mission_" + mission + ".json");
        scene_id_ = sc["scene_id"].as<std::string>();
        rig_ = m["rig"]["name"].as<std::string>();
        if (const YAML::Node lamp = m["rig"]["lamp"]) {   // 5.2 (D146): the lamp as rendered, and its scene's light
            LampRig l;
            l.rig = rig_;
            l.intensity = lamp["intensity"].as<double>();
            const auto b = lamp["in_body_m"].as<std::vector<double>>();
            l.in_body_m = core::Vec3(b.at(0), b.at(1), b.at(2));
            const YAML::Node lighting = sc["preset"]["values"]["lighting"];
            l.lamp_only = lighting && lighting["ambient"].as<double>() == 0.0 && sc["lights"] && sc["lights"].size() == 0;
            lamp_ = l;
        }
        YAML::Node ag;
        for (const auto& x : m["agents"]) {
            if (x["agent"].as<int>() == agent) ag.reset(x);
        }
        if (!ag) throw std::runtime_error("sim: agent " + std::to_string(agent) + " not in the mission");
        frame_ = ag["frame_id"].as<std::string>();
        t_left_ = ag["topics"]["left_image"].as<std::string>();
        if (!ag["topics"][right_camera_ + "_image"]) {
            throw std::runtime_error("sim: the mission has no right camera '" + right_camera_ + "' (rig " + rig_ + ")");
        }
        t_right_ = ag["topics"][right_camera_ + "_image"].as<std::string>();
        if (m["rig"]["depth"]["enabled"].as<int>()) t_depth_ = ag["topics"]["depth_image"].as<std::string>();
        const std::string t_pose = ag["topics"]["gt_pose"].as<std::string>();
        t_imu_ = ag["topics"]["imu"].as<std::string>();
        if (!m["lens"]) throw std::runtime_error("sim: mission has no rig.lens (pre-D41 recording)");

        // The lens: the camera the estimator sees (D41).
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
        if (lens_.k3 != 0.0) throw std::runtime_error("sim: PinholeRadtan carries k1 k2 p1 p2 only; rig.lens.k3 != 0");
        map_ = perception::BuildLensMap(lens_);

        // Canonical: GT body poses and the static camera extrinsics.
        RosbagReader bag;
        bag.Open(rec + "/canonical/canonical_0.mcap");
        bag.Replay([&](const RosMessage& msg) {
            if (msg.topic == t_pose) {
                const auto p = ros_cdr::DecodePoseStamped(msg.data, msg.size);
                TimedPose tp{p.header.stamp.ns(), {}};
                tp.T.R = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                             .normalized()
                             .toRotationMatrix();
                tp.T.t = core::Vec3(p.position.x, p.position.y, p.position.z);
                gt_.push_back(tp);
            } else {
                for (const auto& t : ros_cdr::DecodeTfMessage(msg.data, msg.size)) {
                    if (t.header.frame_id != frame_) continue;
                    core::SE3 T;
                    T.R = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
                              .normalized()
                              .toRotationMatrix();
                    T.t = core::Vec3(t.translation.x, t.translation.y, t.translation.z);
                    const std::string base = frame_.substr(0, frame_.size() - 9);   // "agent_k/"
                    if (t.child_frame_id == base + "left_optical") T_B_left_ = T, ++found_;
                    if (t.child_frame_id == base + right_camera_ + "_optical") T_B_right_ = T, ++found_;
                }
            }
        }, {t_pose, "/tf_static"});
        if (gt_.empty() || found_ != 2) throw std::runtime_error("sim: canonical bag lacks GT pose or static TF");

        const YAML::Node recj = YAML::LoadFile(rec + "/recording.json");
        for (const auto& f : recj["streams"]["sensors"]["files"]) {
            const std::string name = f.first.as<std::string>();
            if (name.size() > 5 && name.substr(name.size() - 5) == ".mcap") {
                sensors_mcap_ = rec + "/sensors/" + name;
                checksum_ = "sha256:" + f.second["sha256"].as<std::string>();
            }
        }
        if (sensors_mcap_.empty()) throw std::runtime_error("sim: recording.json lists no sensors .mcap");
        frames_ = 0;
        for (const auto& t : recj["result"]["topics"]) {
            if (t.first.as<std::string>() == t_left_) frames_ = t.second["count"].as<size_t>();
        }
    }

    std::string RunName() const override { return scene_id_ + "-agent" + std::to_string(agent_); }
    std::string SceneName() const override { return "sim/" + scene_id_ + "/agent" + std::to_string(agent_); }
    std::string CameraConfig() const override {
        return rig_ + "+lens(D41)" + (right_camera_ == "right" ? "" : "+" + right_camera_);
    }
    std::string BagChecksum() const override { return checksum_; }
    perception::RawStereoCalibration Calibration() const override {
        perception::PinholeRadtan c;
        c.width = lens_.width;
        c.height = lens_.height;
        c.fu = lens_.fx;
        c.fv = lens_.fy;
        c.cu = lens_.cx;
        c.cv = lens_.cy;
        c.model = perception::DistortionModel::kRadtan;   // the D41 lens is Brown-Conrady
        c.dist = {lens_.k1, lens_.k2, lens_.p1, lens_.p2};
        perception::RawStereoCalibration s;
        s.left = s.right = c;   // identical lenses (rig_stereo_v1)
        s.T_right_left = T_B_right_.inverse() * T_B_left_;
        return s;
    }
    core::SE3 T_B_C0() const override { return T_B_left_; }
    std::vector<TimedPose> GroundTruth() const override { return gt_; }
    size_t FrameCount() const override { return frames_; }
    uint32_t Agent() const override { return uint32_t(agent_); }
    std::string DegradationConfig() const override { return degradation_ ? degradation_name_ : "none"; }
    perception::radiometric::FlatField FlatFieldOf(int cam) const override { return flat_[cam == 0 ? 0 : 1]; }
    /// The IMU only once SetImuNoise was called (L2, D101): without it the source exposes no IMU, so default sim
    /// runs (imu_mode none, the Phase-1 gate) see exactly what they saw before.
    std::vector<core::ImuMeasurement> Imu() override { return imu_; }
    core::ImuNoise ImuNoiseModel() const override { return imu_noise_ ? imu_noise_->noise : core::ImuNoise{}; }
    std::string ImuNoiseConfig() const override { return imu_noise_ ? imu_noise_->name : "none"; }
    std::optional<LampRig> Lamp() const override { return lamp_; }
    /// Replay actuator (D147): D's exposure, analog gain and lamp_power, only on linear frames (D's sRGB decode).
    bool CanActuatePhotometric() const override {
        return degradation_ && degradation_->input_transfer == perception::InputTransfer::kSrgb;
    }
    /// Failure E's scripted event (D147); needs D (the event is a radiance scale applied before D's sensor).
    /// 5.5 (D148): attach the recording's GT depth to every frame (StereoFrame::gt_depth, evaluation only).
    void SetGtDepth(bool on) {
        if (on && t_depth_.empty()) throw std::runtime_error("sim: the recording has no depth camera (rig.depth off)");
        gt_depth_ = on;
    }

    void SetRadianceEvent(const RadianceEvent& e) {
        if (!degradation_) throw std::logic_error("sim: a radiance event needs --degradation");
        if (!(e.at_s >= 0.0) || !std::isfinite(e.stops)) throw std::invalid_argument("sim: radiance event at_s >= 0");
        radiance_event_ = e;
    }
    void SetPhotometric(const PhotometricCommand& cmd) override {
        if (!CanActuatePhotometric()) throw std::logic_error("sim: photometric commands need D with input_transfer: srgb");
        if (cmd.decision_id <= photo_.decision_id || (pending_cmd_ && cmd.decision_id <= pending_cmd_->decision_id)) {
            throw std::invalid_argument("sim: photometric decision_id must increase");
        }
        pending_cmd_ = cmd;   // supersedes any pending older one (dropped, never applied late)
    }

    /// Read the canonical (clean) IMU stream and apply the seeded noise operator `p` (L2, D101;
    /// core/inertial/imu_noise_operator.hpp). The seed derives from the run seed and the agent, as D's do. A config
    /// with all densities 0 exposes the clean stream. ImuNoiseModel() then reports exactly the injected model.
    void SetImuNoise(const core::ImuNoiseOperatorParams& p, uint64_t run_seed) override {
        std::vector<core::ImuMeasurement> clean;
        RosbagReader bag;
        bag.Open(rec_ + "/canonical/canonical_0.mcap");
        bag.Replay([&](const RosMessage& msg) {
            const auto m = ros_cdr::DecodeImu(msg.data, msg.size);
            core::ImuMeasurement s;
            s.t_ns = m.header.stamp.ns();
            s.gyro = core::Vec3(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
            s.accel = core::Vec3(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
            clean.push_back(s);
        }, {t_imu_});
        if (clean.empty()) throw std::runtime_error("sim: canonical bag has no IMU on " + t_imu_);
        std::stable_sort(clean.begin(), clean.end(),
                         [](const core::ImuMeasurement& a, const core::ImuMeasurement& b) { return a.t_ns < b.t_ns; });
        const uint64_t seed = core::DeriveSeed(core::DeriveSeed(run_seed, core::seed_purpose::kImuNoise),
                                               ("agent" + std::to_string(agent_)).c_str());
        imu_ = core::ApplyImuNoise(clean, p, seed);
        imu_noise_ = p;
    }

    /// Apply D (named config `d`) to every frame from now on (2.2, D66). Camera seeds derive from the run seed,
    /// the agent and the side. Also calibrates each camera's flat field from D frames of a flat target under
    /// the same sensor and seed (radiometric/flat_field.hpp), as a lab would before the mission.
    void SetDegradation(const perception::NamedDegradation& d, uint64_t run_seed, int threads) {
        if (d.params.enable_dust && t_depth_.empty()) {
            throw std::runtime_error("sim: degradation " + d.name + " enables dust, but the mission records no GT "
                                     "depth (rig.depth.enabled 0); refused rather than run without it");
        }
        degradation_ = d.params;
        degradation_name_ = d.name;
        photo_ = PhotoOf(d.params);
        threads_ = threads;
        const uint64_t root = core::DeriveSeed(run_seed, core::seed_purpose::kDegradation);
        const std::string agent = "agent" + std::to_string(agent_);
        seed_[0] = core::DeriveSeed(root, (agent + "/left").c_str());
        seed_[1] = core::DeriveSeed(root, (agent + "/right").c_str());
        for (int c = 0; c < 2; ++c) {
            flat_[c] = perception::radiometric::CalibrateFlatField(d.params, seed_[c], lens_.width, lens_.height, 3,
                                                                   16, 200, threads_);
        }
    }

    size_t ForEachFrame(size_t max_frames, const std::function<void(const StereoFrame&)>& on_frame) override {
        struct Stop {};
        // Stamps not yet complete: the clean images (and GT depth when D has dust, D142) wait here until every part
        // of the pair has arrived; the pair is then processed in one go. D's frame id is the pair index, exactly as
        // when each side was degraded on arrival (2.2), so dust-free runs are byte-identical to before.
        struct Raw {
            core::ImageU8 img[2];
            core::ImageF32 depth;
            bool have[3] = {false, false, false};
        };
        const bool dust = degradation_ && degradation_->enable_dust;
        std::map<int64_t, Raw> pending;
        int64_t last_emitted = -1;
        size_t n = 0;
        RosbagReader bag;
        bag.Open(sensors_mcap_);
        std::set<std::string> topics{t_left_, t_right_};
        if (dust || gt_depth_) topics.insert(t_depth_);
        try {
            bag.Replay([&](const RosMessage& msg) {
                const auto im = ros_cdr::DecodeImage(msg.data, msg.size);
                ros_cdr::ValidateImage(im);
                const int64_t t = im.header.stamp.ns();
                Raw& r = pending[t];
                if (msg.topic == t_depth_) {
                    if (im.encoding != "32FC1") throw std::runtime_error("sim: expected 32FC1 depth, got " + im.encoding);
                    r.depth = core::ImageF32(int(im.width), int(im.height));
                    for (uint32_t y = 0; y < im.height; ++y) {
                        std::memcpy(&r.depth.data[size_t(y) * im.width], &im.data[size_t(y) * im.step], im.width * 4);
                    }
                    r.have[2] = true;
                } else {
                    if (im.encoding != "rgb8") throw std::runtime_error("sim: expected rgb8, got " + im.encoding);
                    const int c = msg.topic == t_left_ ? 0 : 1;
                    r.img[c] = core::ImageU8(int(im.width), int(im.height), 3);
                    for (uint32_t y = 0; y < im.height; ++y) {
                        std::memcpy(&r.img[c].data[size_t(y) * im.width * 3], &im.data[size_t(y) * im.step],
                                    im.width * 3);
                    }
                    r.have[c] = true;
                }
                if (!r.have[0] || !r.have[1] || ((dust || gt_depth_) && !r.have[2])) return;
                // Complete pair: it must be the oldest pending stamp, or a frame went missing.
                if (pending.begin()->first != t || t <= last_emitted) {
                    throw std::runtime_error("sim: stereo pair at " + std::to_string(t) +
                                             " completed before an older stamp: a frame is missing");
                }
                last_emitted = t;
                StereoFrame done;
                done.t_ns = t;
                core::ImageF32 depth_seen[2];
                if (dust) {
                    // Each camera's depth through the same lens as its image; the right camera's by warping the
                    // left depth on the render grid first (runtime/datasets/sim_depth.hpp).
                    const double baseline = (T_B_left_.t - T_B_right_.t).norm();
                    depth_seen[0] = perception::ApplyLensDepth(r.depth, map_);
                    depth_seen[1] = perception::ApplyLensDepth(WarpDepthToRight(r.depth, lens_.render_fx, baseline), map_);
                }
                if (gt_depth_) {   // evaluation only (D148): the left camera's GT depth on the raw (lens) grid
                    const core::ImageF32 g = dust ? depth_seen[0] : perception::ApplyLensDepth(r.depth, map_);
                    done.gt_depth = cv::Mat(g.height, g.width, CV_32F, const_cast<float*>(g.data.data())).clone();
                }
                // A pending command (5.3, D147) takes effect on this pair, before D runs on it.
                if (pending_cmd_) {
                    ApplyPhotometric(*degradation_, *pending_cmd_, t, photo_);
                    pending_cmd_.reset();
                }
                if (first_t_ns_ < 0) first_t_ns_ = t;
                // Failure E (D147): the scene, not the sensor, gets brighter; FramePhoto does not report it.
                const double event = RadianceEventFactor(radiance_event_, t - first_t_ns_);
                std::optional<perception::DegradationParams> scene_params;
                if (degradation_ && event != 1.0) {
                    scene_params = *degradation_;
                    scene_params->radiance_scale *= event;
                }
                for (int c = 0; c < 2; ++c) {
                    core::ImageU8 seen = perception::ApplyLensParallel(r.img[c], map_);
                    if (degradation_) {
                        seen = perception::DegradeParallel(seen, depth_seen[c], scene_params ? *scene_params : *degradation_,
                                                           seed_[c], n, threads_);
                    }
                    const cv::Mat rgb(seen.height, seen.width, CV_8UC3, seen.data.data());
                    cv::Mat gray;
                    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
                    (c == 0 ? done.left : done.right) = gray;
                    if (c == 0) done.left_rgb = rgb.clone();
                }
                pending.erase(t);
                // Linear pixels only with D's sRGB decode (D130): then the photometric state is D's (D146).
                if (CanActuatePhotometric()) done.photo = photo_;
                done.agent = uint32_t(agent_);
                done.id = n;
                on_frame(done);
                if (++n == max_frames) throw Stop{};
            }, topics);
        } catch (const Stop&) {
        }
        if (max_frames == 0 && !pending.empty()) throw std::runtime_error("sim: unpaired frames at the end");
        return n;
    }

private:
    std::string rec_, scene_id_, rig_, frame_, t_left_, t_right_, t_depth_, t_imu_, sensors_mcap_,
        checksum_ = kUnknownField;
    int agent_ = 0;
    std::string right_camera_ = "right";               // 5.5 (D138): the right camera by name (sweep rigs)
    bool gt_depth_ = false;                            // 5.5 (D148): attach GT depth to frames (evaluation only)
    std::optional<LampRig> lamp_;
    FramePhoto photo_;                                 // D's state as applied to the latest frame (D147)
    std::optional<PhotometricCommand> pending_cmd_;    // applied to the next complete pair
    std::optional<RadianceEvent> radiance_event_;      // Failure E's scripted event (D147)
    int64_t first_t_ns_ = -1;
    size_t frames_ = 0;
    int found_ = 0;
    perception::LensModel lens_;
    perception::LensMap map_;
    core::SE3 T_B_left_, T_B_right_;
    std::vector<TimedPose> gt_;
    std::optional<perception::DegradationParams> degradation_;   // 2.2: D, when SetDegradation was called
    std::string degradation_name_;
    uint64_t seed_[2] = {0, 0};
    int threads_ = 1;
    perception::radiometric::FlatField flat_[2];
    std::vector<core::ImuMeasurement> imu_;                  // L2: the noisy stream, when SetImuNoise was called
    std::optional<core::ImuNoiseOperatorParams> imu_noise_;
};

}  // namespace ariadne::runtime
