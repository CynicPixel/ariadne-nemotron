#pragma once
// ---------------------------------------------------------------------------
// The 1.4 + 1.5 stereo VO run over a StereoSource (EuRoC or a sim recording),
// scored against the source's ground truth. Moved verbatim from
// ariadne_euroc_vo.cpp (2026-10-15) with the dataset access replaced by the
// source interface; the EuRoC MH_01 evidence reproduces bit-identically.
//
// Scoring: estimated poses are of the RECTIFIED left camera; they are mapped
// to the body frame with T_B_Crect = T_B_C0 * [R1^T | 0] (the frame trap),
// associated to ground truth by nearest timestamp within 0.01 s (evo's
// default), and scored with SE(3)-aligned ATE (translation + rotation), RPE
// per second, and the Sim(3) scale as a drift diagnostic. Trajectories are
// written in TUM format for evo (the 1.9 oracle).
// ---------------------------------------------------------------------------
#include "core/config/config.hpp"
#include "perception/features/engine_factory.hpp"
#include "perception/stereo/sgbm_stereo.hpp"
#include "perception/stereo/stereo_lk_matcher.hpp"
#include "perception/stereo/stereo_rectifier.hpp"
#include "perception/tracking/klt_tracker.hpp"
#include "runtime/agent/intra_loop_closure.hpp"
#include "runtime/tools/depth_eval.hpp"
#include "runtime/agent/keyframe_records.hpp"
#include "runtime/datasets/radiometric_source.hpp"
#include "runtime/datasets/stereo_source.hpp"
#include "runtime/manifest/build_identity.hpp"
#include "runtime/manifest/determinism.hpp"
#include "runtime/manifest/opencv_identity.hpp"
#include "runtime/manifest/run_manifest.hpp"
#include "runtime/metrics/trajectory_io.hpp"
#include "runtime/metrics/trajectory_metrics.hpp"
#include "runtime/tools/feature_engine_glue.hpp"
#include "runtime/tools/exposure_glue.hpp"
#include "runtime/tools/falloff_glue.hpp"
#include "runtime/tools/reassociation_glue.hpp"
#include "slam/inertial/attitude.hpp"
#include "slam/odometry/stereo_vo.hpp"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;


namespace ariadne::tools {

namespace fs = std::filesystem;
using namespace ariadne::perception;

namespace detail {
inline double Ms(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

inline double Mean(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x;
    return v.empty() ? -1.0 : s / v.size();
}

inline std::string UtcNow(const char* fmt) {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, fmt, &tm);
    return buf;
}

inline core::Mat3 ToEigen3(const cv::Mat& m) {
    core::Mat3 R;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R(r, c) = m.at<double>(r, c);
    return R;
}

}  // namespace detail
using namespace detail;

/// What a keyframe hook sees (1.6 collaboration, 1.7 spool): the canonical records, built here, once, by the
/// producer: the §4.2 Keyframe (pose of the rectified left camera in this agent's odometry frame, its absolute
/// covariance, SGBM depth, observed landmarks) and the §18.2 KeyframeBundle exported at the graph's current
/// version. Plus what only a feature extractor needs: the rectified gray image and the rectified rig.
struct KeyframeView {
    runtime::Keyframe keyframe;
    runtime::KeyframeBundle bundle;
    cv::Mat left_rect;       // 8-bit gray (the bundle carries the RGB)
    core::SE3 T_B_C;         // body <- rectified left camera
    double baseline_m = 0;
    /// The tracker's left-pixel observations at this keyframe (StereoVo::KeyframeObservationsLeft): what the
    /// console's feature overlay shows - the landmarks the estimator USED, never a redetection.
    std::vector<cv::Point2f> track_px;
    const slam::PoseGraph* graph = nullptr;   // the VO graph right after this keyframe (read-only; D171 pass A)
    std::optional<runtime::FramePhoto> photo;   // the applied photometric state (D147); unset without D
};

/// Every frame (optional hook): the estimated rectified-left camera pose and a
/// lazy rectified-RGB getter (only frames that ask pay for the remap).
struct FrameView {
    uint64_t frame_id = 0;
    int64_t t_ns = 0;
    bool is_keyframe = false;
    core::SE3 T_O_C;
    std::function<cv::Mat()> left_rect_rgb;   // empty Mat if the source has no colour
    std::optional<runtime::FramePhoto> photo; // the applied photometric state (D147); unset without D
};

struct StereoVoArgs {
    std::string perception_config, slam_config, imu_noise_config, out;
    std::string lamp_calib;   // 5.2 (D146): the rig's lamp calibration, for a perception config with a falloff: section
    std::string scenario;     // 5.3 (D147): a scripted event the tool set on the source, for the manifest ("" = none)
    bool depth_eval = false;  // 5.5 (D148): SGBM depth vs the frames' GT depth at keyframes (evaluation only)
    int frames = 0;
    uint64_t seed = 1;
    std::string tool = "ariadne_euroc_vo";   // results.yaml header
    std::function<void(const KeyframeView&)> on_keyframe;   // optional; never changes the run
    std::function<void(int64_t, const core::SE3&)> on_body_pose;   // every frame's T_O_B; optional
    std::function<void(const FrameView&)> on_frame;                 // every frame; optional; never changes the run
    /// Fake outages (INERTIAL_2.4 §4.4, deciding imu.full.lost_motion): [start, start + length) seconds from the first
    /// frame, the VO receives no tracks (as in a lamp failure); after each one the tracks return under FRESH ids, as
    /// after a real loss with no re-association. The tracker itself runs on, so later frames are unaffected otherwise.
    std::vector<std::pair<double, double>> outages;
    /// Live sources (design LIVE_RUNTIME_L1 D152) receive their ground truth during the run: when set, scoring uses
    /// what this returns after the last frame instead of GroundTruth() at the start. GT never reaches the estimator.
    std::function<std::vector<runtime::TimedPose>()> late_ground_truth;
};

/// The 1.4 + 1.5 pipeline over any StereoSource: rectify -> LK tracker ->
/// StereoVo, keyframes with LK stereo matches + SGBM, then scoring against the
/// source's ground truth. Shared by ariadne_euroc_vo and ariadne_sim_vo.
inline int RunStereoVo(runtime::StereoSource& inner_source, const StereoVoArgs& args, int argc, char** argv) {
    const PerceptionConfig pcfg = LoadPerception(args.perception_config);
    // 2.2: the radiometric front end between the source and the rectifier (a call-through when mode is off).
    runtime::RadiometricSource source(inner_source, pcfg.radiometric);
    if (!args.imu_noise_config.empty() && source.ImuNoiseConfig() == "none")
        source.SetImuNoise(core::LoadImuNoiseOperator(args.imu_noise_config), args.seed);
    // 2.6 (D100): an RGB-D source feeds sensor depth through a virtual rig (perception/rgbd/depth_stereo.hpp) in
    // place of the right image, SGBM and the LK stereo matcher. Stereo sources take the unchanged path.
    const std::optional<DepthSensorModel> depth_model = source.DepthSensor();
    const bool rgbd = depth_model.has_value();
    if (rgbd && pcfg.radiometric.Enabled()) throw std::runtime_error("the radiometric stage needs a stereo pair");
    slam::SlamConfig scfg_loaded = slam::LoadSlam(args.slam_config);

    StereoRectifier rectifier(source.Calibration(), 0.0);   // not const: the right map takes the row refinement
    const RectifiedStereoRig& rr = rectifier.Rig();
    const core::RectifiedStereo rig{{rr.fx, rr.fy, rr.cx, rr.cy}, rr.baseline_m};
    const core::SE3 T_B_Crect = runtime::BodyFromRectifiedLeft(source.T_B_C0(), ToEigen3(rectifier.R1()));
    // RGB-D: the sensor's sigma_Z = k Z^2 enters the triangulation gate as the virtual rig's disparity noise.
    if (rgbd) scfg_loaded.triangulation.disparity_sigma_px = depth_model->DisparitySigmaPx(rr.fx);
    const slam::SlamConfig& scfg = scfg_loaded;
    if (!rgbd && scfg.triangulation.min_depth_m < MinObservableRange(pcfg.stereo, rr)) {
        std::fprintf(stderr, "warning: slam min_depth_m %.3f is below the stereo Z_min %.3f\n",
                     scfg.triangulation.min_depth_m, MinObservableRange(pcfg.stereo, rr));
    }

    const std::unique_ptr<LocalFeatureEngine> features = MakeFeatureEngine(pcfg.features);   // 2.1: orb | xfeat
    // D73 part 2b: intra-agent loop closure, only with a `loop_closure:` config section. Read-only towards the VO.
    std::unique_ptr<runtime::IntraLoopClosure> loops;
    if (scfg.loop_closure.enabled) loops = std::make_unique<runtime::IntraLoopClosure>(scfg.loop_closure, rig, args.seed);
    KltTracker tracker(pcfg.tracker, *features);
    const SgbmStereo sgbm(pcfg.stereo, rr);
    // imu_mode full (D59): the IMU's mount (rectified left camera in the body) and its datasheet noise; StereoVo
    // applies imu.full's inflation.
    const slam::VoImuRig imu_rig{T_B_Crect, source.ImuNoiseModel()};
    slam::StereoVo vo(scfg, rig, args.seed, source.Agent(), scfg.imu.mode == "full" ? &imu_rig : nullptr);
    LostReassociator reassoc(pcfg.reassociation, *features, rr.fx, rr.fy, rr.cx, rr.cy);   // 2.1; off unless configured
    // --outages hands the VO track ids shifted per outage; re-association renames ids in the TRACKER, so the two
    // would rename in different id spaces. Refused rather than silently wrong.
    if (pcfg.reassociation.enabled && !args.outages.empty()) {
        throw std::runtime_error("--outages cannot be combined with perception reassociation (track id spaces differ)");
    }
    std::vector<double> t_conf;   // 2.1 §10: right-matcher + WLS per stereo pass (stereo.confidence = wls only)

    std::vector<runtime::TimedPose> gt_poses = source.GroundTruth();
    const int n_planned = args.frames > 0 ? std::min<int>(args.frames, static_cast<int>(source.FrameCount()))
                                          : static_cast<int>(source.FrameCount());
    std::printf("%s: %d frames; rig fx=%.2f B=%.4f; gt samples %zu\n", source.SceneName().c_str(), n_planned, rr.fx,
                rr.baseline_m, gt_poses.size());

    // 2.4 step 2: the gyro rotation between consecutive frames, in the rectified left camera, logged next to the
    // bridge (vo_trace.csv) to validate frame, sign and time alignment against ground truth BEFORE the estimator uses
    // the IMU. Zero bias here; the bias comes from initialisation later.
    std::vector<core::ImuMeasurement> imu = source.Imu();
    int64_t prev_frame_t_ns = -1;
    // imu_mode attitude_only (2.4, slam/inertial/attitude.hpp): gyro rotation while lost, gyro-seeded LK, gravity.
    std::unique_ptr<slam::AttitudeTracker> attitude;
    if (scfg.imu.mode != "none") {
        if (imu.empty()) throw std::runtime_error("slam config imu.mode " + scfg.imu.mode + " but the source has no IMU");
        slam::AttitudeParams ap;
        ap.bias_window = scfg.imu.bias_window;
        ap.stationary_s = scfg.imu.stationary_s;
        ap.kp = scfg.imu.kp;
        ap.ki = scfg.imu.ki;
        attitude = std::make_unique<slam::AttitudeTracker>(ap, imu, source.ImuNoiseModel(), T_B_Crect.R);
    }
    const cv::Matx33d K_rect(rr.fx, 0, rr.cx, 0, rr.fy, rr.cy, 0, 0, 1);
    // 5.2 Stage 3 (D146): the albedo proxy of every keyframe; refuses here, before any frame, if its inputs are wrong.
    KeyframeFalloff falloff(pcfg.falloff, args.lamp_calib, source.Lamp(), source.T_B_C0(), K_rect,
                            cv::Matx33d(rectifier.R1()));
    // 5.3 (D147): the exposure + lamp co-controller at keyframes, commanding the source; refuses a source that
    // cannot actuate. prev_photo / lk_rescales: known-gain LK (below), a no-op while the applied state is constant.
    KeyframeExposure exposure(pcfg.exposure_control, source.CanActuatePhotometric());
    KeyframeDepthEval depth_eval;   // 5.5 (D148): used only with args.depth_eval
    std::optional<runtime::FramePhoto> prev_photo;
    long lk_rescales = 0;
    slam::VoTrackResult prev_tr;
    long gyro_seeded = 0;
    std::vector<double> gravity_err_deg;   // at keyframes: Mahony gravity vs ground truth (EuRoC world z is up)
    // imu_mode full (D59): solve and initialisation counters, the initialisation against ground truth (INERTIAL_2.4
    // §4.3), and a per-keyframe trace of the solved velocity and biases next to ground truth (imu_trace.csv).
    const bool full_imu = scfg.imu.mode == "full";
    const std::vector<runtime::GroundTruthInertial> gt_inertial = full_imu ? source.GroundTruthInertialStates()
                                                                            : std::vector<runtime::GroundTruthInertial>{};
    long imu_solves = 0, imu_v0_held = 0, imu_factors = 0, imu_repropagated = 0, imu_prior_marginal = 0,
         imu_prior_carried = 0, imu_init_refused = 0, lost_inertial = 0;
    double imu_chi2 = 0.0, imu_tilt_sum = 0.0, imu_tilt_max = 0.0;
    int64_t imu_init_t_ns = -1, first_t_ns = -1;
    std::string imu_init_vs_gt;   // filled at initialisation
    std::ostringstream imu_trace;
    imu_trace << "keyframe,t_ns,source,ba_inertial,v0_held,vx_B,vy_B,vz_B,bgx,bgy,bgz,bax,bay,baz,"
                 "gt_vx_B,gt_vy_B,gt_vz_B,gt_bgx,gt_bgy,gt_bgz,gt_bax,gt_bay,gt_baz,gravity_err_deg,tilt_deg,g_prior_sigma_deg,"
                 "ba_landmarks\n";
    std::vector<std::pair<double, double>> gravity_err_t;   // (t_s, deg): the estimated gravity vs GT, per keyframe
    // GRAVITY_PRIOR (§4.10): per candidate, NEES and angle error vs GT at each emission; edges emitted.
    struct GpStats { std::vector<double> nees, ang; long failed = 0; };
    std::map<std::string, GpStats> gp_stats;
    long gp_edges = 0;
    std::ostringstream gp_trace;
    gp_trace << "keyframe,t_ns,candidate,nees,angle_err_deg,sigma_deg,emitted\n";
    // Nearest ground-truth pose / inertial state within 10 ms (EuRoC GT is 200 Hz).
    const auto gt_pose_at = [&](int64_t t, core::SE3& T) {
        const auto it = std::lower_bound(gt_poses.begin(), gt_poses.end(), t,
                                         [](const runtime::TimedPose& p, int64_t x) { return p.t_ns < x; });
        if (it == gt_poses.end() || std::llabs(it->t_ns - t) >= 10000000LL) return false;
        T = it->T;
        return true;
    };
    const auto gt_inertial_at = [&](int64_t t, runtime::GroundTruthInertial& g) {
        const auto it = std::lower_bound(gt_inertial.begin(), gt_inertial.end(), t,
                                         [](const runtime::GroundTruthInertial& p, int64_t x) { return p.t_ns < x; });
        if (it == gt_inertial.end() || std::llabs(it->t_ns - t) >= 10000000LL) return false;
        g = *it;
        return true;
    };
    std::vector<runtime::TimedPose> est_body;
    // ORB-SLAM's trajectory protocol (System::SaveTrajectoryEuRoC): each frame is stored relative to its reference
    // keyframe and re-composed at the end from that keyframe's FINAL (window-refined) pose. Window backend only, next
    // to the causal per-frame score, which stays the primary number (LOCAL_SLAM_2.3 §10 protocol caveat).
    struct FrameRef { uint64_t kf; core::SE3 T_kf_c; };
    std::vector<FrameRef> frame_ref;
    std::vector<double> t_track, t_vo, t_kf;
    int lost = 0, keyframes = 0, ba_runs = 0;
    int lost_run = 0;            // consecutive lost frames (diagnostic)
    // Lost-frame handling (LOST_FRAMES.md, D56): written to results.yaml only when the config has a `lost:` section.
    long lost_bridged = 0, lost_coasted = 0, lost_rot_h = 0, lost_rot_e = 0, lost_rot_none = 0, bridge_retries = 0;
    long reanchor_probes = 0, reanchor_refused = 0, keyframes_lost = 0;
    uint32_t vo_segments = 0;   // loss onsets so far (StereoVo's segment counter)
    // Per-frame trace (vo_trace.csv): what the frontend saw on every frame, so a lost stretch can be explained.
    std::ostringstream trace;
    trace << "frame,t_ns,status,tracks,correspondences,pnp_inliers,keyframe,stereo_valid,created,source,segment,"
             "probe,bridge_model,bridge_tracks,bridge_inliers_h,bridge_inliers_e,bridge_cheiral_e,bridge_disagreement_deg,"
             "bridge_rx,bridge_ry,bridge_rz,anchor_model,anchor_tracks,anchor_age,anchor_drift_deg,gyro_rx,gyro_ry,gyro_rz\n";
    double last_finite_t = 0.0;
    slam::TriangulationStats tri;
    double ba_cost_ratio_sum = 0.0;
    // 2.3 window backend only (results.yaml keeps the v1 bytes): solve outcomes and the post-solve hygiene.
    long win_not_found = 0, win_obs = 0, win_invalid_entry = 0, win_chi2_failed = 0, win_rejected = 0,
         win_culled = 0, win_exit_edges = 0, win_unobservable = 0, win_ill = 0, win_ill_after = 0,
         win_unobservable_after = 0, win_dof = 0, win_degenerate = 0;
    double win_sq = 0.0;
    long stereo_valid = 0, stereo_total = 0;
    // Keyframe stereo outcomes by reason x image third (top/middle/bottom), and the valid matches' disparity sums:
    // diagnostic output only (D144: where and why stereo fails on a real rig).
    long stereo_reason[3][5] = {};
    double stereo_disp_sum[3] = {}, stereo_disp_max[3] = {};
    long usac_retries = 0, usac_skipped = 0;
    // Online row refinement (D54): estimated at keyframes until it succeeds, then frozen and disclosed.
    perception::RowCorrection row_fix, row_check;   // row_check: re-estimated AFTER the refinement (acceptance: ~0)
    perception::RowSamples row_samples;              // pooled over keyframes until the fit succeeds
    int row_attempts = 0, row_keyframe = -1;
    std::string row_last_refusal = "never attempted";   // OpenCV 5.0.0 USAC defect (SMOKE §4q); written only when non-zero

    int i = 0;
    const int n = static_cast<int>(source.ForEachFrame(static_cast<size_t>(std::max(args.frames, 0)),
                                                       [&](const runtime::StereoFrame& frame) {
        if (frame.agent != source.Agent() || frame.id != static_cast<uint64_t>(i)) {
            throw std::logic_error("stereo source: frame identity out of sequence at " + std::to_string(i));
        }
        source.RefreshImu(frame.t_ns, imu);
        cv::Mat l, r, dz;   // dz: RGB-D depth on the rectified left grid (nearest: no invented depths)
        if (rgbd) {
            rectifier.RectifyLeft(frame.left, l);
            rectifier.RectifyLeftNearest(frame.depth, dz);
        } else {
            rectifier.Rectify(frame.left, frame.right, l, r);
        }

        // 5.3 (D147): known-gain LK. When the applied brightness changed since the last frame, the tracker's previous
        // pyramid is brought to this frame's level first (LK has no gain term).
        if (frame.photo && prev_photo && frame.photo->Brightness() != prev_photo->Brightness()) {
            tracker.RescalePrevious(frame.photo->Brightness() / prev_photo->Brightness());
            ++lk_rescales;
        }
        prev_photo = frame.photo;
        auto t0 = std::chrono::steady_clock::now();
        bool have_rot = false;
        core::Mat3 R_pc = core::Mat3::Identity();
        cv::Matx33d H_pred;
        if (attitude && prev_frame_t_ns >= 0 && attitude->Covers(prev_frame_t_ns, frame.t_ns)) {
            R_pc = attitude->PredictCameraRotation(prev_frame_t_ns, frame.t_ns);
            have_rot = true;
            cv::Matx33d Rt;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) Rt(r, c) = R_pc(c, r);   // R_C(cur)_C(prev): prev-camera rays into the current one
            H_pred = K_rect * Rt * K_rect.inv();
        }
        // The angular rate for the exposure controller's blur ceiling (§10.5); 0 without an IMU (ceiling = t_max_s).
        const double omega_rad_s =
            have_rot ? core::SO3Log(R_pc).norm() / (1e-9 * double(frame.t_ns - prev_frame_t_ns)) : 0.0;
        const bool seed_lk = have_rot && scfg.imu.seed_lk;
        gyro_seeded += seed_lk;
        const TrackedFrame tf = tracker.Process(l, static_cast<uint64_t>(i), args.seed, seed_lk ? &H_pred : nullptr);
        t_track.push_back(Ms(t0));
        usac_retries += tf.stats.usac_defect_retries;
        usac_skipped += tf.stats.usac_defect_skipped;

        slam::VoFrame vf;
        vf.frame_id = static_cast<uint64_t>(i);
        vf.t_ns = frame.t_ns;
        if (first_t_ns < 0) first_t_ns = frame.t_ns;
        // --outages: the VO's view of the tracks (empty inside an outage, fresh ids after each).
        const double t_rel = 1e-9 * double(frame.t_ns - first_t_ns);
        bool in_outage = false;
        uint64_t id_offset = 0;
        for (const auto& [o0, olen] : args.outages) {
            in_outage = in_outage || (t_rel >= o0 && t_rel < o0 + olen);
            if (t_rel >= o0 + olen) id_offset += uint64_t(1) << 40;
        }
        TrackedFrame tfv = tf;
        if (in_outage) {
            tfv.ids.clear();
            tfv.points.clear();
        } else {
            for (auto& id : tfv.ids) id += id_offset;
        }
        vf.track_ids = tfv.ids;
        vf.points = tfv.points;
        reassoc.BeforeTrack(l, vo, tracker, vf, args.seed);   // 2.1: may rename orphan tracks to old landmarks
        t0 = std::chrono::steady_clock::now();
        vf.has_rotation_prior = have_rot;
        vf.R_prev_cur_prior = R_pc;
        if (full_imu) {
            // [the last sample at or before the previous frame (this one, for the first), the first at or after this one]
            const int64_t from = prev_frame_t_ns >= 0 ? prev_frame_t_ns : frame.t_ns;
            auto lo = std::upper_bound(imu.begin(), imu.end(), from,
                                       [](int64_t t, const core::ImuMeasurement& m) { return t < m.t_ns; });
            if (lo != imu.begin()) --lo;
            auto hi = std::lower_bound(imu.begin(), imu.end(), frame.t_ns,
                                       [](const core::ImuMeasurement& m, int64_t t) { return m.t_ns < t; });
            if (hi != imu.end()) ++hi;
            if (lo < hi) vf.imu.assign(lo, hi);
        }
        const slam::VoTrackResult tr = vo.Track(vf);
        if (attitude) {
            // Consecutive MEASURED frames refine the gyro bias (both PnP poses, before any keyframe refinement).
            if (tr.status == slam::VoStatus::kTracking && prev_tr.status == slam::VoStatus::kTracking && prev_frame_t_ns >= 0)
                attitude->ObserveTracked(prev_frame_t_ns, frame.t_ns, prev_tr.T_W_C.R.transpose() * tr.T_W_C.R);
            attitude->AdvanceTo(frame.t_ns);
        }
        prev_tr = tr;
        t_vo.push_back(Ms(t0));
        lost += tr.status == slam::VoStatus::kLost;
        lost_run = tr.status == slam::VoStatus::kLost ? lost_run + 1 : 0;
        core::SE3 T_W_C = tr.T_W_C;
        if (!T_W_C.R.allFinite() || !T_W_C.t.allFinite()) {
            // A non-finite pose poisons every later prediction and turns the score into NaN: stop here, loudly.
            throw std::runtime_error("stereo VO: non-finite pose at frame " + std::to_string(i) + " (status " +
                                     std::to_string(static_cast<int>(tr.status)) + ", correspondences " +
                                     std::to_string(tr.correspondences) + ", lost for " + std::to_string(lost_run) +
                                     " frames, last finite |t| " + std::to_string(last_finite_t) + " m, tracks " +
                                     std::to_string(tf.ids.size()) + ", keyframes " + std::to_string(keyframes) + ")");
        }
        last_finite_t = T_W_C.t.norm();

        long trace_valid = -1, trace_created = -1;   // keyframes (and re-anchor probes) only
        bool made_keyframe = false;
        if (tr.status == slam::VoStatus::kLost) {
            lost_bridged += tr.source == slam::PoseSource::kBridged;
            lost_coasted += tr.source == slam::PoseSource::kCoasted;
            lost_inertial += tr.source == slam::PoseSource::kInertial;
            if (tr.source == slam::PoseSource::kBridged) {
                lost_rot_h += tr.anchor_bridge.ok && tr.anchor_bridge.model == 'H';
                lost_rot_e += tr.anchor_bridge.ok && tr.anchor_bridge.model == 'E';
                lost_rot_none += !tr.anchor_bridge.ok;
            }
        }
        bridge_retries += tr.bridge.usac_retries + tr.anchor_bridge.usac_retries;
        vo_segments = tr.segment;
        // A lost frame under lost.reanchor = gated is only PROBED: its stereo is computed, and it becomes a keyframe
        // only if StereoVo::ShouldReanchor says the next frame's PnP would then have enough landmarks.
        if (tr.need_keyframe || tr.reanchor_probe) {
            t0 = std::chrono::steady_clock::now();
            slam::VoStereo st;
            std::vector<uchar> stereo_reason_q;   // MatchLeftToRight's per-point reasons (stereo path only; D144 diag)
            StereoDepth depth;   // the keyframe records' dense depth (SGBM, or the sensor's on RGB-D)
            cv::Mat supported;
            if (rgbd) {
                depth = DenseDepthFromSensor(dz, rr.fx, *depth_model);
                supported = depth.valid;
                const VirtualStereo vs = VirtualStereoFromDepth(dz, tfv.points, rr.fx, *depth_model);
                st.right = vs.right;
                st.valid = vs.valid;
                st.sgbm_z = vs.z;          // the §9.6 slot: sensor depth where supported
                st.sgbm_sigma = vs.sigma_z;
            } else {
                // The stereo pair as matched: right exposure matched to the left (EuRoC per-camera auto-exposure),
                // then, until it succeeds, the row-refinement estimate on THIS pair -- applied before the first real
                // match, so a misaligned rig does not start out lost (AUDIT_2.3.md, D54).
                const auto condition = [&](const cv::Mat& right) {
                    return pcfg.stereo_match.exposure_match ? MatchExposure(l, right) : right;
                };
                cv::Mat rs = condition(r);
                if (tr.need_keyframe && pcfg.stereo_match.row_refinement && !row_fix.ok) {
                    ++row_attempts;
                    CollectRowSamples(l, rs, tfv.points, pcfg.stereo_match, row_samples);
                    const RowCorrection est = FitRowCorrection(row_samples, pcfg.stereo_match);
                    if (est.ok) {
                        row_fix = est;
                        row_keyframe = keyframes;
                        rectifier.RefineRightRows(est.a, est.b, est.c);
                        cv::Mat l_again;
                        rectifier.Rectify(frame.left, frame.right, l_again, r);
                        rs = condition(r);
                        std::printf("  stereo row refinement at keyframe %d: dy = %+.3f %+.5f x %+.5f y px (%d matches, "
                                    "MAD %.3f px%s)\n", keyframes, est.a, est.b, est.c, est.matches, est.mad_px,
                                    est.slopes_fitted ? "" : ", offset only");
                    } else {
                        row_last_refusal = est.why;
                    }
                } else if (tr.need_keyframe && pcfg.stereo_match.row_refinement && !row_check.ok && keyframes > row_keyframe) {
                    // Acceptance, measured once on a later pair: the residual left after the refinement (not applied).
                    row_check = EstimateRowCorrection(l, rs, tfv.points, pcfg.stereo_match);
                }
                const StereoMatches sm = MatchLeftToRight(l, rs, tfv.points, pcfg.stereo_match);
                depth = sgbm.Compute(l, rs);
                if (!depth.confidence_is_placeholder) t_conf.push_back(depth.confidence_ms);
                supported = DenseSupportMask(depth, pcfg.stereo);
                cv::Mat z32, s32;
                depth.Z.convertTo(z32, CV_32F);
                depth.sigma_Z.convertTo(s32, CV_32F);
                st.right = sm.right;
                st.valid = sm.valid;
                stereo_reason_q = sm.reason;
                for (const cv::Point2f& p : tfv.points) {
                    const int x = cvRound(p.x), y = cvRound(p.y);
                    const bool ok = x >= 0 && y >= 0 && x < rr.width && y < rr.height && supported.at<uchar>(y, x);
                    st.sgbm_z.push_back(ok ? z32.at<float>(y, x) : std::numeric_limits<float>::quiet_NaN());
                    st.sgbm_sigma.push_back(ok ? s32.at<float>(y, x) : std::numeric_limits<float>::quiet_NaN());
                }
            }
            trace_valid = 0;
            for (uchar v : st.valid) trace_valid += v;
            reanchor_probes += !tr.need_keyframe;
            made_keyframe = tr.need_keyframe || vo.ShouldReanchor(st);
            reanchor_refused += !made_keyframe;
            if (made_keyframe) {
                stereo_valid += trace_valid;
                stereo_total += static_cast<long>(st.valid.size());
                for (size_t q = 0; q < stereo_reason_q.size(); ++q) {
                    const int band = std::clamp(static_cast<int>(3.0f * tfv.points[q].y / rr.height), 0, 2);
                    ++stereo_reason[band][stereo_reason_q[q]];
                    if (stereo_reason_q[q] == 0) {
                        const double d = tfv.points[q].x - st.right[q].x;
                        stereo_disp_sum[band] += d;
                        stereo_disp_max[band] = std::max(stereo_disp_max[band], d);
                    }
                }
                // 5.2 (D146): with features: raw it never feeds tracking or the VO (the trajectory is the same as with
                // the stage off); features: proxy uses it only for detection at Replenish below.
                perception::radiometric::AlbedoProxy albedo;
                if (falloff.enabled()) albedo = falloff.Normalize(l, depth.Z, supported, frame.photo);
                if (args.depth_eval) {   // 5.5 (D148), evaluation only: the GT never reaches tracking or the VO
                    if (frame.gt_depth.empty()) throw std::runtime_error("depth_eval: the source attached no GT depth");
                    cv::Mat g;
                    rectifier.RectifyLeftNearest(frame.gt_depth, g);
                    depth_eval.Add(g, depth.Z, depth.valid, K_rect, cv::Matx33d(rectifier.R1()));
                }
                // 5.3 (D147): one controller decision per keyframe, from the state this frame reports as applied.
                if (exposure.enabled()) {
                    const int64_t interval = prev_frame_t_ns >= 0 ? frame.t_ns - prev_frame_t_ns : 0;
                    source.SetPhotometric(exposure.Decide(l, *frame.photo, frame.t_ns, omega_rad_s, rr.fx, interval));
                }
                if (attitude) vo.SetKeyframeGravity(attitude->GravityInCamera());   // GRAVITY_PRIOR (§4.10)
                const slam::VoKeyframeResult kr = vo.AddKeyframe(st);
                // GRAVITY_PRIOR candidates against ground truth (INERTIAL_2.4 §4.10): the 2-dof NEES of each candidate's
                // down direction vs the true one in this keyframe's camera (EuRoC GT world: z up).
                if (!kr.gravity_candidates.empty() && !gt_poses.empty()) {
                    const auto it = std::lower_bound(gt_poses.begin(), gt_poses.end(), frame.t_ns,
                                                     [](const runtime::TimedPose& p, int64_t t) { return p.t_ns < t; });
                    if (it != gt_poses.end() && std::llabs(it->t_ns - frame.t_ns) < 10000000LL) {
                        const core::Vec3 g_true = ((it->T * T_B_Crect).R.transpose() * core::Vec3(0, 0, -1)).normalized();
                        for (const auto& c : kr.gravity_candidates) {
                            auto& acc = gp_stats[c.name];
                            if (!c.ok) {
                                ++acc.failed;
                                continue;
                            }
                            const Eigen::Vector2d e = slam::GravityTangentBasis(c.g_C.normalized()).transpose() * g_true;
                            const double nees = e.dot(c.cov.ldlt().solve(e));
                            const double ang = std::acos(std::clamp(c.g_C.normalized().dot(g_true), -1.0, 1.0)) * 180.0 / M_PI;
                            const double sig = std::sqrt(0.5 * c.cov.trace()) * 180.0 / M_PI;
                            acc.nees.push_back(nees);
                            acc.ang.push_back(ang);
                            gp_trace << kr.keyframe_id << ',' << frame.t_ns << ',' << c.name << ',' << nees << ',' << ang
                                     << ',' << sig << ',' << int(kr.gravity_prior_emitted) << '\n';
                        }
                    }
                }
                gp_edges += kr.gravity_prior_emitted;
                t_kf.push_back(Ms(t0));
                reassoc.AfterKeyframe(l, vo, vf);   // 2.1: landmark descriptors at this keyframe
                // 2.1 §4.2: new tracks at keyframes under replenish: keyframe (no-op otherwise); 5.2 (D146) row
                // features: proxy detects them on the albedo proxy, inside its feature mask.
                if (falloff.detects_on_proxy()) tracker.Replenish(l, falloff.DetectionImage(albedo), albedo.feature_mask);
                else tracker.Replenish(l);
                ++keyframes;
                keyframes_lost += tr.status == slam::VoStatus::kLost;
                if (attitude && !gt_poses.empty()) {
                    const auto it = std::lower_bound(gt_poses.begin(), gt_poses.end(), frame.t_ns,
                                                     [](const runtime::TimedPose& p, int64_t t) { return p.t_ns < t; });
                    if (it != gt_poses.end() && std::llabs(it->t_ns - frame.t_ns) < 10000000LL) {
                        const core::Vec3 g_true = it->T.R.transpose() * core::Vec3(0, 0, -1);
                        const core::Vec3 g_est = T_B_Crect.R * attitude->GravityInCamera();
                        gravity_err_deg.push_back(std::acos(std::clamp(g_est.dot(g_true), -1.0, 1.0)) * 180.0 / M_PI);
                    }
                }
                tri.candidates += kr.triangulation.candidates;
                tri.created += kr.triangulation.created;
                trace_created = kr.triangulation.created;
                tri.no_stereo += kr.triangulation.no_stereo;
                tri.nonpositive_disparity += kr.triangulation.nonpositive_disparity;
                tri.depth_range += kr.triangulation.depth_range;
                tri.reprojection += kr.triangulation.reprojection;
                tri.sgbm_inconsistent += kr.triangulation.sgbm_inconsistent;
                tri.uncertain += kr.triangulation.uncertain;
                if (kr.ba_ran) {
                    ++ba_runs;
                    ba_cost_ratio_sum += kr.ba_final_cost / std::max(kr.ba_initial_cost, 1e-12);
                    win_not_found += !kr.ba_found;
                    win_obs += kr.ba_observations;
                    win_invalid_entry += kr.ba_invalid_at_entry;
                    win_chi2_failed += kr.ba_chi2_failed;
                    win_sq += kr.ba_inlier_sq_px;
                    win_dof += kr.ba_inlier_dof;
                    win_ill_after += kr.ba_ill_conditioned_after;
                    win_unobservable_after += kr.ba_unobservable_after;
                    win_degenerate += kr.ba_marginal_degenerate;
                }
                if (full_imu) {
                    imu_solves += kr.ba_inertial;
                    imu_v0_held += kr.ba_v0_held;
                    imu_factors += kr.ba_inertial_factors;
                    imu_chi2 += kr.ba_inertial_chi2;
                    imu_repropagated += kr.imu_repropagated;
                    imu_prior_marginal += kr.imu_prior == 1;
                    imu_prior_carried += kr.imu_prior == 2;
                    imu_init_refused += kr.imu_init_refused;
                    imu_tilt_sum += kr.imu_tilt_deg;
                    imu_tilt_max = std::max(imu_tilt_max, kr.imu_tilt_deg);
                    core::SE3 T_gt;
                    runtime::GroundTruthInertial gi;
                    if (kr.imu_initialized) {
                        // §4.3: the initialisation against ground truth, in frames that VO drift cannot bias -- each
                        // keyframe's own body frame (its estimated orientation vs the ground-truth one).
                        imu_init_t_ns = frame.t_ns;
                        const slam::InertialInitResult& in = vo.inertial_init();
                        const std::vector<int64_t>& ts = vo.init_times();
                        const uint64_t kf0 = kr.keyframe_id + 1 - ts.size();
                        double v_sq = 0.0, g_deg = -1.0;
                        int nv = 0;
                        for (size_t k = 0; k < ts.size(); ++k) {
                            if (!gt_pose_at(ts[k], T_gt) || !gt_inertial_at(ts[k], gi)) continue;
                            const core::Mat3 R_est = (vo.graph().nodes().at(kf0 + k).T_W_C * T_B_Crect.inverse()).R;
                            v_sq += (R_est.transpose() * in.v[k] - T_gt.R.transpose() * gi.v_W).squaredNorm();
                            ++nv;
                            if (k + 1 == ts.size()) {
                                const core::Vec3 a = (R_est.transpose() * in.g_W).normalized();
                                const core::Vec3 b = T_gt.R.transpose() * core::Vec3(0, 0, -1);
                                g_deg = std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 180.0 / M_PI;
                            }
                        }
                        std::ostringstream o;
                        o << "{gravity_angle_deg: " << g_deg << ", v_rms_m_s: " << (nv ? std::sqrt(v_sq / nv) : -1.0);
                        if (gt_inertial_at(ts.back(), gi))
                            o << ", bg_err: " << (in.bg - gi.b_g).norm() << ", ba_err: " << (in.ba - gi.b_a).norm()
                              << ", bg_gt_norm: " << gi.b_g.norm() << ", ba_gt_norm: " << gi.b_a.norm();
                        o << "}";
                        imu_init_vs_gt = o.str();
                    }
                    if (vo.inertial_ready()) {
                        core::Vec3 v, bg, ba;
                        vo.NewestInertialState(v, bg, ba);
                        const core::Mat3 R_est = (vo.graph().nodes().at(kr.keyframe_id).T_W_C * T_B_Crect.inverse()).R;
                        const core::Vec3 vb = R_est.transpose() * v;
                        imu_trace << kr.keyframe_id << ',' << frame.t_ns << ',' << slam::ToString(kr.source) << ','
                                  << int(kr.ba_inertial) << ',' << int(kr.ba_v0_held) << ',' << vb.x() << ',' << vb.y()
                                  << ',' << vb.z() << ',' << bg.x() << ',' << bg.y() << ',' << bg.z() << ',' << ba.x()
                                  << ',' << ba.y() << ',' << ba.z();
                        if (gt_pose_at(frame.t_ns, T_gt) && gt_inertial_at(frame.t_ns, gi)) {
                            const core::Vec3 gv = T_gt.R.transpose() * gi.v_W;
                            // Gravity in this keyframe's own body frame, estimated vs ground truth (immune to VO drift
                            // in the world frame; includes this keyframe's own roll/pitch error).
                            const core::Vec3 ge = (R_est.transpose() * vo.gravity()).normalized();
                            const core::Vec3 gg = T_gt.R.transpose() * core::Vec3(0, 0, -1);
                            const double gdeg = std::acos(std::clamp(ge.dot(gg), -1.0, 1.0)) * 180.0 / M_PI;
                            gravity_err_t.emplace_back(1e-9 * double(frame.t_ns - first_t_ns), gdeg);
                            imu_trace << ',' << gv.x() << ',' << gv.y() << ',' << gv.z() << ',' << gi.b_g.x() << ','
                                      << gi.b_g.y() << ',' << gi.b_g.z() << ',' << gi.b_a.x() << ',' << gi.b_a.y() << ','
                                      << gi.b_a.z() << ',' << gdeg;
                        } else {
                            imu_trace << ",nan,nan,nan,nan,nan,nan,nan,nan,nan,nan";
                        }
                        imu_trace << ',' << kr.imu_tilt_deg << ',' << kr.imu_g_prior_sigma_deg << ',' << kr.ba_landmarks
                                  << '\n';
                    }
                }
                win_rejected += kr.ba_rejected;
                win_culled += kr.ba_culled;
                win_exit_edges += kr.exit_edge;
                win_unobservable += kr.ba_unobservable;
                win_ill += kr.ba_ill_conditioned;
                T_W_C = vo.graph().nodes().rbegin()->second.T_W_C;
                if (args.on_keyframe) {
                    const slam::PoseNode& node = vo.graph().nodes().at(kr.keyframe_id);
                    if (node.agent != frame.agent || node.T_W_C.t != T_W_C.t) {
                        throw std::logic_error("keyframe hook: graph node does not match the keyframe just added");
                    }
                    KeyframeView kv;
                    runtime::Keyframe& kf = kv.keyframe;
                    kf.id = kr.keyframe_id;
                    kf.frame_id = frame.id;
                    kf.agent = frame.agent;
                    kf.t_ns = frame.t_ns;
                    kf.T_world_cam = node.T_W_C;
                    kf.pose_cov = node.covariance;
                    kf.depth = depth;
                    kf.landmark_ids = vo.KeyframeLandmarks(kr.keyframe_id);
                    kv.track_px = vo.KeyframeObservationsLeft(kr.keyframe_id);
                kf.pose_source = kr.source;
                kf.segment = kr.segment;
                if (attitude) {
                    kf.has_gravity = true;
                    kf.gravity_dir_C = attitude->GravityInCamera();
                }
                    cv::Mat rgb = l;
                    if (!frame.left_rgb.empty()) rectifier.RectifyLeft(frame.left_rgb, rgb);
                    kv.bundle = runtime::BuildKeyframeBundle(vo.graph(), kr.keyframe_id, rig.cam, rgb, depth, supported);
                    kv.bundle.albedo = albedo.albedo;   // empty unless Stage 3 ran (D146)
                    kv.bundle.albedo_valid = albedo.valid;
                    kv.bundle.albedo_feature_mask = albedo.feature_mask;
                    if (exposure.enabled()) {   // §18.2 states, owned by 5.3 (D147): what this keyframe was taken with
                        kv.bundle.exposure_state = runtime::ExposureState{exposure.ExposureSeconds(*frame.photo),
                                                                          frame.photo->gain};
                        kv.bundle.lamp_state = runtime::LampState{frame.photo->lamp_rel > 0.0, frame.photo->lamp_rel};
                    }
                    kv.left_rect = l;
                    kv.T_B_C = T_B_Crect;
                    kv.baseline_m = rr.baseline_m;
                    kv.graph = &vo.graph();
                    kv.photo = frame.photo;
                    args.on_keyframe(kv);
                }
                if (loops) {
                    const runtime::KeyframeBundle lb =
                        runtime::BuildKeyframeBundle(vo.graph(), kr.keyframe_id, rig.cam, l, depth, supported);
                    loops->OnKeyframe(kr.keyframe_id, frame.t_ns, l, runtime::MaskedDepth32(lb.depth, lb.mask), vo.graph());
                }
            }
        }
        const core::Vec3 brot = core::SO3Log(tr.bridge.R_prev_cur);
        trace << i << ',' << vf.t_ns << ',' << static_cast<int>(tr.status) << ',' << tf.ids.size() << ','
              << tr.correspondences << ',' << tr.pnp_inliers << ',' << int(made_keyframe) << ',' << trace_valid << ','
              << trace_created << ',' << slam::ToString(tr.source) << ',' << tr.segment << ',' << int(tr.reanchor_probe)
              << ',' << tr.bridge.model << ',' << tr.bridge.tracks << ',' << tr.bridge.inliers_h << ','
              << tr.bridge.inliers_e << ',' << tr.bridge.cheiral_e << ',' << tr.bridge.disagreement_deg << ',' << brot.x() << ',' << brot.y() << ',' << brot.z() << ','
              << tr.anchor_bridge.model << ',' << tr.anchor_bridge.tracks << ',' << tr.anchor_age << ','
              << tr.anchor_drift_deg << ',';
        if (!imu.empty() && prev_frame_t_ns >= 0 && imu.front().t_ns <= prev_frame_t_ns && imu.back().t_ns >= vf.t_ns) {
            const core::Mat3 R_BC = T_B_Crect.R;
            const core::Vec3 g =
                core::SO3Log(R_BC.transpose() * core::IntegrateGyro(imu, prev_frame_t_ns, vf.t_ns, core::Vec3::Zero()) * R_BC);
            trace << g.x() << ',' << g.y() << ',' << g.z() << '\n';
        } else {
            trace << "nan,nan,nan\n";
        }
        prev_frame_t_ns = vf.t_ns;
        est_body.push_back({vf.t_ns, T_W_C * T_B_Crect.inverse()});
        if (!vo.graph().nodes().empty()) {
            const slam::PoseNode& ref = vo.graph().nodes().rbegin()->second;
            frame_ref.push_back({ref.keyframe, ref.T_W_C.inverse() * T_W_C});
        } else {
            frame_ref.push_back({~uint64_t(0), T_W_C});
        }
        if (args.on_body_pose) args.on_body_pose(vf.t_ns, est_body.back().T);
        if (args.on_frame) {
            FrameView fv;
            fv.frame_id = vf.frame_id;
            fv.t_ns = vf.t_ns;
            fv.is_keyframe = made_keyframe;
            fv.T_O_C = T_W_C;
            fv.photo = frame.photo;
            fv.left_rect_rgb = [&]() {
                cv::Mat o;
                if (!frame.left_rgb.empty()) rectifier.RectifyLeft(frame.left_rgb, o);
                return o;
            };
            args.on_frame(fv);
        }
        if (i % 500 == 0) std::printf("  frame %d: tracks %zu, status %d, keyframes %d\n", i, tf.ids.size(),
                                      static_cast<int>(tr.status), keyframes);
        ++i;
    }));
    if (args.late_ground_truth) gt_poses = args.late_ground_truth();

    // --- Scoring ---------------------------------------------------------------
    const std::vector<int> assoc = runtime::AssociateNearest(est_body, gt_poses);
    std::vector<core::TrajectorySample> e, g;
    for (size_t i = 0; i < est_body.size(); ++i) {
        if (assoc[i] < 0) continue;
        e.push_back({core::Timestamp{est_body[i].t_ns}, 0, est_body[i].T});
        g.push_back({core::Timestamp{gt_poses[static_cast<size_t>(assoc[i])].t_ns}, 0,
                     gt_poses[static_cast<size_t>(assoc[i])].T});
    }
    const auto ate = runtime::ComputeATE(e, g);
    if (!std::isfinite(ate.rmse)) {
        throw std::runtime_error("stereo VO: non-finite ATE over " + std::to_string(e.size()) + " associated poses");
    }
    const auto ate_rot = runtime::ComputeATERotation(e, g);
    const auto sim3 = runtime::AlignTrajectories(e, g, true);
    // 1 s by timestamp, not 20 samples: a stream with dropped frames (EuRoC V2_03) would otherwise stretch the window.
    const auto rpe = runtime::ComputeRPE(e, g, 1.0, 0.05);
    double gt_length = 0.0;
    for (size_t i = 1; i < g.size(); ++i) gt_length += (g[i].pose.t - g[i - 1].pose.t).norm();
    runtime::ErrorStats ate_kf, ate_kf_rot;
    runtime::RelativeError rpe_kf;
    if (scfg.ba.backend == "window") {
        std::vector<core::TrajectorySample> ek;
        for (size_t i = 0; i < est_body.size(); ++i) {
            if (assoc[i] < 0) continue;
            const FrameRef& fr = frame_ref[i];
            const core::SE3 T = fr.kf == ~uint64_t(0) ? fr.T_kf_c : vo.graph().nodes().at(fr.kf).T_W_C * fr.T_kf_c;
            ek.push_back({core::Timestamp{est_body[i].t_ns}, 0, T * T_B_Crect.inverse()});
        }
        ate_kf = runtime::ComputeATE(ek, g);
        ate_kf_rot = runtime::ComputeATERotation(ek, g);
        rpe_kf = runtime::ComputeRPE(ek, g, 1.0, 0.05);
    }
    // The same keyframe-refined protocol on the loop-corrected keyframe poses (D73 part 2b, design §8).
    runtime::ErrorStats ate_loop, ate_loop_rot;
    if (loops) {
        loops->Finish(vo.graph(), est_body.empty() ? 0 : est_body.back().t_ns);
        std::vector<core::TrajectorySample> el;
        for (size_t i = 0; i < est_body.size(); ++i) {
            if (assoc[i] < 0) continue;
            const FrameRef& fr = frame_ref[i];
            const core::SE3 T = fr.kf == ~uint64_t(0) ? fr.T_kf_c : loops->Corrected(fr.kf, vo.graph()) * fr.T_kf_c;
            el.push_back({core::Timestamp{est_body[i].t_ns}, 0, T * T_B_Crect.inverse()});
        }
        ate_loop = runtime::ComputeATE(el, g);
        ate_loop_rot = runtime::ComputeATERotation(el, g);
    }

    const fs::path run_dir = fs::path(args.out) / (source.RunName() + "-" + UtcNow("%Y%m%dT%H%M%SZ"));
    fs::create_directories(run_dir);
    runtime::WriteTum((run_dir / "estimate_body.tum").string(), est_body);
    std::ofstream(run_dir / "vo_trace.csv") << trace.str();
    if (reassoc.enabled()) std::ofstream(run_dir / "reassoc_trace.csv") << reassoc.Trace();
    if (exposure.enabled()) std::ofstream(run_dir / "exposure_trace.csv") << exposure.Trace();   // 5.3, D147
    // Ground truth goes under raw/, which .gitignore excludes: for EuRoC it is
    // the dataset's own data (InC-NC, D35), so evidence commits carry our
    // estimate and derived numbers but never a copy of the dataset.
    fs::create_directories(run_dir / "raw");
    runtime::WriteTum((run_dir / "raw" / "groundtruth_body.tum").string(), gt_poses);

    runtime::RunManifest m;
    m.run_id = run_dir.filename().string();
    m.created_utc = UtcNow("%Y-%m-%dT%H:%M:%SZ");
    m.git_commit = std::getenv("ARIADNE_GIT_COMMIT") ? std::getenv("ARIADNE_GIT_COMMIT") : runtime::kUnknownField;
    m.git_dirty = std::getenv("ARIADNE_GIT_DIRTY") && std::string(std::getenv("ARIADNE_GIT_DIRTY")) == "true";
    m.image_digest = std::getenv("ARIADNE_IMAGE_DIGEST") ? std::getenv("ARIADNE_IMAGE_DIGEST") : runtime::kUnknownField;
    m.scene = source.SceneName();
    m.camera_config = source.CameraConfig();
    m.seed = args.seed;
    m.seed_recorded = true;
    m.agent_count = 1;
    m.execution_mode = "replay";
    m.perception_config = pcfg.name;
    {
        std::ostringstream rf;
        if (!pcfg.stereo_match.row_refinement) rf << "off";
        else if (!row_fix.ok) rf << "not_estimated: " << row_last_refusal;
        else rf << "a=" << row_fix.a << " b=" << row_fix.b << " c=" << row_fix.c << " matches=" << row_fix.matches
                << " keyframe=" << row_keyframe;
        m.stereo_row_refinement = rf.str();
    }
    m.slam_config = scfg.name;
    m.lost_handling = scfg.lost.motion + "/" + scfg.lost.reanchor;
    m.degradation_config = source.DegradationConfig();
    m.radiometric = source.Summary();
    if (falloff.enabled()) m.extra["falloff"] = falloff.Summary();   // 5.2; default manifests unchanged
    if (exposure.enabled()) m.extra["exposure_control"] = exposure.Summary();   // 5.3
    if (!args.scenario.empty()) m.extra["scenario"] = args.scenario;            // 5.3: e.g. Failure E's radiance event
    RecordFeatureEngine(m, *features, pcfg);   // 2.1: engine, caps, cadence
    m.loop_backend = "none";
    m.imu_mode = scfg.imu.mode;
    m.bag_checksum = source.BagChecksum();
    runtime::RecordBuildIdentity(m);
    runtime::RecordOpenCvIdentity(m);
    runtime::RecordDeterminismDigests(m);
    m.extra["opencv_threads"] = std::to_string(cv::getNumThreads());
    m.extra["frames"] = std::to_string(n);
    std::string cmd;
    for (int i = 0; i < argc; ++i) cmd += (i ? " " : "") + std::string(argv[i]);
    m.extra["command"] = cmd;
    if (reassoc.enabled()) m.extra["reassociation"] = reassoc.ManifestValue();
    std::ofstream(run_dir / "manifest.yaml") << runtime::ToYaml(m);

    std::ofstream res(run_dir / "results.yaml");
    // A sequence without dense GT (Hilti 2023: sparse control points only, D141) has nothing to align: the metric
    // structs then hold their zero defaults, which must not read as a 0 m error. Those fields print null, and the
    // sequence is scored by its own protocol from estimate_body.tum (analysis.hilti_score).
    const bool dense_gt = !gt_poses.empty();
    const auto gt_num = [dense_gt](double v) {
        std::ostringstream o;
        if (dense_gt) o << v;
        else o << "null";
        return o.str();
    };
    res << "# " << args.tool << " results. Generated; do not hand-edit.\n"
        << "sequence: " << source.RunName() << "\nframes: " << n << "\n"
        << "dense_gt: " << (dense_gt ? "true" : "false  # ATE/RPE fields are null: score with the dataset's protocol")
        << "\n"
        << "associated_poses: " << e.size() << "  # nearest GT within 0.01 s (evo default)\n"
        << "gt_path_length_m: " << gt_num(gt_length) << "\n"
        << "ate_se3: {rmse_m: " << gt_num(ate.rmse) << ", mean_m: " << gt_num(ate.mean) << ", max_m: "
        << gt_num(ate.max) << ", rotation_rmse_rad: " << gt_num(ate_rot.rmse) << "}\n"
        << "ate_rmse_percent_of_path: " << (gt_length > 0 ? 100.0 * ate.rmse / gt_length : -1.0) << "\n"
        << "sim3_scale: " << gt_num(sim3.scale) << "  # 1.0 = no scale drift; stereo is metric\n"
        << "rpe_1s: {translation_rmse_m: " << gt_num(rpe.translation.rmse) << ", rotation_rmse_rad: "
        << gt_num(rpe.rotation.rmse) << "}\n"
        << "vo: {lost_frames: " << lost << ", keyframes: " << keyframes << ", ba_runs: " << ba_runs
        << ", ba_mean_cost_ratio: " << (ba_runs ? ba_cost_ratio_sum / ba_runs : -1.0)
        << ", landmarks_alive_at_end: " << vo.landmark_count()
        << ", graph_edges: " << vo.graph().edges().size() << ", graph_version: " << vo.graph().version() << "}\n"
        << "stereo_match_valid_fraction: " << (stereo_total ? double(stereo_valid) / stereo_total : -1.0) << "\n"
        << "stereo_match_by_third: {";
    for (int b = 0; b < 3; ++b) {
        const long* r = stereo_reason[b];
        res << (b ? ", " : "") << (b == 0 ? "top" : b == 1 ? "middle" : "bottom") << ": {valid: " << r[0]
            << ", lk_lost: " << r[1] << ", fb: " << r[2] << ", off_row: " << r[3] << ", nonpositive: " << r[4]
            << ", disparity_mean_px: " << (r[0] ? stereo_disp_sum[b] / r[0] : -1.0)
            << ", disparity_max_px: " << stereo_disp_max[b] << "}";
    }
    res << "}  # keyframe left->right LK outcomes per image third (diagnostic)\n"
        << "triangulation: {candidates: " << tri.candidates << ", created: " << tri.created
        << ", no_stereo: " << tri.no_stereo << ", nonpositive_disparity: " << tri.nonpositive_disparity
        << ", depth_range: " << tri.depth_range << ", reprojection: " << tri.reprojection
        << ", sgbm_inconsistent: " << tri.sgbm_inconsistent << ", uncertain: " << tri.uncertain << "}\n"
        << "timing_ms: {track_mean: " << Mean(t_track) << ", vo_track_mean: " << Mean(t_vo)
        << ", keyframe_mean: " << Mean(t_kf) << ", opencv_threads: " << cv::getNumThreads() << "}\n";
    res << "stereo_row_refinement: {enabled: " << (pcfg.stereo_match.row_refinement ? "true" : "false")
        << ", applied: " << (row_fix.ok ? "true" : "false") << ", a_px: " << row_fix.a << ", b: " << row_fix.b
        << ", c: " << row_fix.c << ", matches: " << row_fix.matches << ", mad_px: " << row_fix.mad_px
        << ", slopes_fitted: " << (row_fix.slopes_fitted ? "true" : "false") << ", keyframe: " << row_keyframe
        << ", attempts: " << row_attempts << ", residual_after: {measured: " << (row_check.ok ? "true" : "false")
        << ", a_px: " << row_check.a << ", b: " << row_check.b << ", c: " << row_check.c
        << ", max_abs_dy_px: "
        << std::max({std::abs(row_check.a), std::abs(row_check.a + row_check.b * (rr.width - 1)),
                     std::abs(row_check.a + row_check.c * (rr.height - 1)),
                     std::abs(row_check.a + row_check.b * (rr.width - 1) + row_check.c * (rr.height - 1))})
        << ", matches: " << row_check.matches << "}}\n"
        << "stereo_exposure_match: " << (pcfg.stereo_match.exposure_match ? "true" : "false") << "\n";
    if (rgbd) {   // 2.6 (D100): only for RGB-D sources, so stereo results stay byte-identical
        res << "rgbd_depth: {sigma_z_model: \"k*Z^2 (Khoshelham & Elberink 2012 eq. 7)\", k_per_m: "
            << depth_model->sigma_k_per_m << ", z_min_m: " << depth_model->z_min_m << ", z_max_m: " << depth_model->z_max_m
            << ", max_neighbour_ratio: " << depth_model->max_neighbour_ratio << ", virtual_baseline_m: "
            << depth_model->virtual_baseline_m << ", disparity_sigma_px: " << scfg.triangulation.disparity_sigma_px
            << ", valid_fraction_at_keyframes: " << (stereo_total ? double(stereo_valid) / stereo_total : -1.0) << "}\n";
    }
    if (usac_retries > 0 || usac_skipped > 0) {
        res << "tracker_opencv_usac_defect: {redraws: " << usac_retries << ", epipolar_check_skipped_frames: "
            << usac_skipped << "}\n";
    }
    if (scfg.ba.backend == "window") {
        if (loops) {
            res << "loop_closed_keyframe_refined: {ate_rmse_m: " << gt_num(ate_loop.rmse)
                << ", ate_rotation_rmse_rad: " << gt_num(ate_loop_rot.rmse) << "}\n";
            loops->WriteOutputs(run_dir.string(),
                                // Interpolated between the bracketing GT samples (<= 20 ms apart), not the nearest one:
                                // nearest-sample timing error would otherwise count as loop error (design §8).
                                [&](int64_t t) -> std::optional<core::SE3> {
                                    const auto it = std::lower_bound(
                                        gt_poses.begin(), gt_poses.end(), t,
                                        [](const runtime::TimedPose& p, int64_t x) { return p.t_ns < x; });
                                    if (it == gt_poses.begin() || it == gt_poses.end()) return std::nullopt;
                                    const runtime::TimedPose& b = *it;
                                    const runtime::TimedPose& a = *(it - 1);
                                    if (b.t_ns - a.t_ns > 20000000LL) return std::nullopt;
                                    const double s = double(t - a.t_ns) / double(b.t_ns - a.t_ns);
                                    const core::SE3 Ti = a.T * core::SE3Exp(s * core::SE3Log(a.T.inverse() * b.T));
                                    return Ti * T_B_Crect;
                                },
                                res);
        }
        res << "keyframe_refined: {ate_rmse_m: " << gt_num(ate_kf.rmse) << ", ate_rotation_rmse_rad: "
            << gt_num(ate_kf_rot.rmse) << ", rpe_1s_translation_rmse_m: " << gt_num(rpe_kf.translation.rmse)
            << "}  # ORB-SLAM protocol: frames re-composed on final keyframe poses\n";
        res << "ba_window: {not_found: " << win_not_found << ", observations: " << win_obs
            << ", invalid_at_entry: " << win_invalid_entry << ", chi2_failed: " << win_chi2_failed
            << ", chi2_failed_fraction: " << (win_obs ? double(win_chi2_failed) / win_obs : -1.0)
            << ", keyframe_observations_rejected: " << win_rejected
            << ", culled: " << win_culled << ", exit_edges: " << win_exit_edges
            << ", keyframes_held: " << win_unobservable << ", keyframes_held_at_solution: " << win_unobservable_after
            << ", landmarks_ill_conditioned: " << win_ill << ", landmarks_ill_conditioned_at_solution: " << win_ill_after
            << ", marginal_degenerate: " << win_degenerate
            << ", inlier_sigma_px: " << (win_dof > 0 ? std::sqrt(win_sq / double(win_dof)) : -1.0) << "}\n";
    }
    if (attitude) {
        const core::Vec3 b = attitude->Bias();
        if (source.ImuNoiseConfig() != "none") res << "imu_noise_config: " << source.ImuNoiseConfig() << "\n";   // L2
        res << "imu: {mode: " << scfg.imu.mode << ", seed_lk: " << (scfg.imu.seed_lk ? "true" : "false")
            << ", stationary_start: " << (attitude->StationaryStart() ? "true" : "false") << ", gyro_bias_final: [" << b.x()
            << ", " << b.y() << ", " << b.z() << "], bias_samples: " << attitude->BiasSamples()
            << ", lk_seeded_frames: " << gyro_seeded;
        if (!gravity_err_deg.empty()) {
            std::vector<double> g = gravity_err_deg;
            std::sort(g.begin(), g.end());
            res << ", gravity_err_deg_vs_gt: {median: " << g[g.size() / 2] << ", p90: " << g[g.size() * 9 / 10]
                << ", max: " << g.back() << "}";
        }
        res << "}\n";
    }
    if (full_imu) {
        const slam::InertialInitResult& in = vo.inertial_init();
        res << "imu_full: {initialized: " << (vo.inertial_ready() ? "true" : "false")
            << ", init_attempts: " << vo.init_attempts() << ", init_refused: " << imu_init_refused
            << ", init_t_s: " << (imu_init_t_ns >= 0 ? 1e-9 * double(imu_init_t_ns - first_t_ns) : -1.0)
            << ", init: {gravity_norm_unconstrained: " << in.gravity_norm_unconstrained
            << ", chi2_per_dof: " << in.chi2_per_dof << ", condition: " << in.condition
            << ", rotation_rms_deg: " << in.rotation_rms_deg << ", last_refusal: \"" << in.why << "\"}"
            << ", init_vs_gt: " << (imu_init_vs_gt.empty() ? std::string("{}") : imu_init_vs_gt)
            << ", inertial_solves: " << imu_solves << ", v0_held: " << imu_v0_held
            << ", inertial_chi2_per_dof: " << (imu_factors ? imu_chi2 / (9.0 * double(imu_factors)) : -1.0)
            << ", repropagated: " << imu_repropagated << ", bias_prior: {marginal: " << imu_prior_marginal
            << ", carried: " << imu_prior_carried << "}, lost_inertial_frames: " << lost_inertial
            << ", tilt_correction_deg: {sum: " << imu_tilt_sum << ", max: " << imu_tilt_max << "}";
        // The estimated gravity vs ground truth over the run, per keyframe body frame: median and p90 over all
        // keyframes since initialisation, and the final value.
        if (!gravity_err_t.empty()) {
            std::vector<double> g;
            for (const auto& e : gravity_err_t) g.push_back(e.second);
            std::sort(g.begin(), g.end());
            res << ", gravity_err_deg_vs_gt: {median: " << g[g.size() / 2] << ", p90: " << g[g.size() * 9 / 10]
                << ", final: " << gravity_err_t.back().second << "}";
        }
        res << ", refinements: [";
        for (size_t r = 0; r < vo.refinements().size(); ++r) {
            const slam::InertialRefinement& f = vo.refinements()[r];
            res << (r ? ", " : "") << "{t_s: " << 1e-9 * double(f.t_ns - first_t_ns) << ", keyframes: " << f.keyframes
                << ", ok: " << (f.ok ? "true" : "false") << ", gravity_change_deg: " << f.gravity_change_deg
                << ", chi2_per_dof: " << f.chi2_per_dof << ", gravity_norm_unconstrained: " << f.gravity_norm_unconstrained
                << ", why: \"" << f.why << "\"}";
        }
        res << "]}\n";
        std::ofstream(run_dir / "imu_trace.csv") << imu_trace.str();
    }
    if (scfg.imu.gravity_prior.enabled) {
        // Mean NEES ~2 for a consistent 2-dof measurement; the p90 shows the tail (chi2_2 p90 = 4.6).
        res << "gravity_prior: {source: " << scfg.imu.gravity_prior.source << ", stride_keyframes: "
            << scfg.imu.gravity_prior.stride_keyframes << ", edges: " << gp_edges << ", candidates: {";
        bool firstc = true;
        for (auto& [name, a] : gp_stats) {
            double mean = 0.0;
            for (double v : a.nees) mean += v;
            std::sort(a.nees.begin(), a.nees.end());
            std::sort(a.ang.begin(), a.ang.end());
            const size_t n = a.nees.size();
            res << (firstc ? "" : ", ") << name << ": {n: " << n << ", failed: " << a.failed
                << ", nees_mean: " << (n ? mean / double(n) : -1.0) << ", nees_median: " << (n ? a.nees[n / 2] : -1.0)
                << ", nees_p90: " << (n ? a.nees[n * 9 / 10] : -1.0) << ", angle_median_deg: " << (n ? a.ang[n / 2] : -1.0)
                << ", angle_p90_deg: " << (n ? a.ang[n * 9 / 10] : -1.0) << "}";
            firstc = false;
        }
        res << "}}\n";
        std::ofstream(run_dir / "gravity_prior_trace.csv") << gp_trace.str();
    }
    if (!args.outages.empty()) {
        // Drift across each fake outage, against ground truth (translation m, rotation deg), from the last frame before
        // it to (1) the last frame inside it -- the prediction at the moment tracking resumes -- and (2) the frame
        // 2.5 s after it ends, once re-anchoring and the window solves have acted: what persists (INERTIAL_2.4 §4.4).
        // Frame poses are causal (as tracked), not re-refined afterwards.
        res << "outages: [";
        const auto drift = [&](int a, int b, double& dt_m, double& dr_deg) {
            core::SE3 ga, gb;
            dt_m = dr_deg = -1.0;
            if (a < 0 || b <= a || !gt_pose_at(est_body[size_t(a)].t_ns, ga) || !gt_pose_at(est_body[size_t(b)].t_ns, gb))
                return;
            const core::SE3 e = est_body[size_t(a)].T.inverse() * est_body[size_t(b)].T, g = ga.inverse() * gb;
            dt_m = (e.t - g.t).norm();
            dr_deg = core::SO3Log(e.R.transpose() * g.R).norm() * 180.0 / M_PI;
        };
        for (size_t o = 0; o < args.outages.size(); ++o) {
            const double o0 = args.outages[o].first, o1 = o0 + args.outages[o].second;
            int a = -1, b = -1, c = -1;
            for (size_t k = 0; k < est_body.size(); ++k) {
                const double t = 1e-9 * double(est_body[k].t_ns - first_t_ns);
                if (t < o0) a = int(k);
                if (t < o1) b = int(k);
                if (t < o1 + 2.5) c = int(k);
            }
            double dt_m, dr_deg, at_m, ar_deg;
            drift(a, b, dt_m, dr_deg);
            drift(a, c, at_m, ar_deg);
            res << (o ? ", " : "") << "{start_s: " << o0 << ", length_s: " << args.outages[o].second
                << ", drift_m: " << dt_m << ", drift_deg: " << dr_deg << ", after_2_5s_m: " << at_m
                << ", after_2_5s_deg: " << ar_deg << ", frames: " << (b - a) << "}";
        }
        res << "]\n";
    }
    if (scfg.lost.motion != "coast" || scfg.lost.reanchor != "every_frame") {
        res << "lost: {motion: " << scfg.lost.motion << ", reanchor: " << scfg.lost.reanchor
            << ", segments: " << vo_segments << ", bridged_frames: " << lost_bridged << ", coasted_frames: " << lost_coasted
            << ", bridge_rotation: {H: " << lost_rot_h << ", E: " << lost_rot_e << ", none: " << lost_rot_none
            << "}, keyframes_while_lost: " << keyframes_lost << ", reanchor_probes: " << reanchor_probes
            << ", reanchor_refused: " << reanchor_refused << ", bridge_usac_redraws: " << bridge_retries << "}\n";
    }
    source.WriteResults(res);   // 2.2; nothing when the radiometric stage is off
    falloff.WriteResults(res);  // 5.2; nothing when Stage 3 is off
    depth_eval.WriteResults(res);   // 5.5 (D148); nothing unless --depth-eval
    exposure.WriteResults(res); // 5.3; nothing when the controller is off
    if (lk_rescales > 0) res << "known_gain_lk_rescales: " << lk_rescales << "\n";   // 5.3: only when the state varied
    if (reassoc.enabled()) res << reassoc.ResultsLine();
    res << features->ResultsYaml();   // 2.1: empty for orb
    if (!t_conf.empty()) res << StereoConfidenceResults(t_conf);   // 2.1 §10
    res.close();

    const auto missing = runtime::MissingRequiredFields(m);
    std::printf("run written to %s\nmanifest missing required fields: %zu", run_dir.c_str(), missing.size());
    for (const auto& f : missing) std::printf(" [%s]", f.c_str());
    std::printf("\n");
    std::fputs(runtime::ReadTextFile((run_dir / "results.yaml").string()).c_str(), stdout);
    return 0;
}

}  // namespace ariadne::tools
