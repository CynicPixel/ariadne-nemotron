#pragma once
// ---------------------------------------------------------------------------
// The radiometric front end (2.2, perception/radiometric/radiometric_stage.hpp)
// as a decorator over any StereoSource: the raw gray pair each frame goes
// through the stage BEFORE the runner rectifies it, exactly where arch §6.1
// puts radiometric normalization. Everything else forwards to the wrapped
// source unchanged.
//
// Mode off (the default: no `radiometric:` section in the perception config):
// ForEachFrame is a straight call-through; no frame is copied or touched.
//
// Otherwise the first gate_frames pairs are held back, the stage fits each
// camera's noise on them and decides the VST gate once (§6.4), and then those
// and all later pairs are emitted corrected. Holding frames back is fine in
// replay; a live run would spend those frames as warm-up.
//
// Only the gray geometry path is corrected. left_rgb (the reconstruction spool's
// colour) passes through: its photometric treatment is the albedo proxy of §19.1,
// which needs the Phase-5 falloff stage.
// ---------------------------------------------------------------------------
#include "perception/radiometric/radiometric_stage.hpp"
#include "runtime/datasets/stereo_source.hpp"

#include <ostream>
#include <sstream>
#include <vector>

namespace ariadne::runtime {

class RadiometricSource : public StereoSource {
public:
    RadiometricSource(StereoSource& inner, const perception::radiometric::RadiometricParams& params)
        : inner_(inner), stage_(params, inner.FlatFieldOf(0), inner.FlatFieldOf(1)) {}

    std::string RunName() const override { return inner_.RunName(); }
    std::string SceneName() const override { return inner_.SceneName(); }
    std::string CameraConfig() const override { return inner_.CameraConfig(); }
    std::string BagChecksum() const override { return inner_.BagChecksum(); }
    perception::RawStereoCalibration Calibration() const override { return inner_.Calibration(); }
    core::SE3 T_B_C0() const override { return inner_.T_B_C0(); }
    std::vector<TimedPose> GroundTruth() const override { return inner_.GroundTruth(); }
    size_t FrameCount() const override { return inner_.FrameCount(); }
    uint32_t Agent() const override { return inner_.Agent(); }
    std::vector<core::ImuMeasurement> Imu() override { return inner_.Imu(); }
    void RefreshImu(int64_t t_ns, std::vector<core::ImuMeasurement>& dst) override { inner_.RefreshImu(t_ns, dst); }
    core::ImuNoise ImuNoiseModel() const override { return inner_.ImuNoiseModel(); }
    void SetImuNoise(const core::ImuNoiseOperatorParams& p, uint64_t run_seed) override {
        inner_.SetImuNoise(p, run_seed);
    }
    std::vector<GroundTruthInertial> GroundTruthInertialStates() const override {
        return inner_.GroundTruthInertialStates();
    }
    std::string DegradationConfig() const override { return inner_.DegradationConfig(); }
    perception::radiometric::FlatField FlatFieldOf(int cam) const override { return inner_.FlatFieldOf(cam); }
    std::optional<perception::DepthSensorModel> DepthSensor() const override { return inner_.DepthSensor(); }
    std::string ImuNoiseConfig() const override { return inner_.ImuNoiseConfig(); }
    std::optional<LampRig> Lamp() const override { return inner_.Lamp(); }
    bool CanActuatePhotometric() const override { return inner_.CanActuatePhotometric(); }
    void SetPhotometric(const PhotometricCommand& cmd) override { inner_.SetPhotometric(cmd); }

    size_t ForEachFrame(size_t max_frames, const std::function<void(const StereoFrame&)>& on_frame) override {
        if (!stage_.Enabled()) return inner_.ForEachFrame(max_frames, on_frame);
        const size_t hold = static_cast<size_t>(stage_.GateFrames());
        std::vector<StereoFrame> held;
        bool decided = hold == 0;
        const auto emit = [&](StereoFrame f) {
            f.left = stage_.Apply(f.left, 0);
            f.right = stage_.Apply(f.right, 1);
            on_frame(f);
        };
        const auto decide = [&] {
            std::vector<cv::Mat> l, r;
            for (const StereoFrame& f : held) l.push_back(f.left), r.push_back(f.right);
            stage_.Decide(l, r);
            decided = true;
            for (StereoFrame& f : held) emit(std::move(f));
            held.clear();
        };
        const size_t n = inner_.ForEachFrame(max_frames, [&](const StereoFrame& f) {
            if (decided) return emit(f);
            held.push_back(f);   // cv::Mat headers: the source hands out a fresh buffer per frame
            if (held.size() == hold) decide();
        });
        if (!decided) decide();   // a run shorter than the gate window decides on what it has
        return n;
    }

    const perception::radiometric::RadiometricReport& Report() const { return stage_.Report(); }

    /// One line for the manifest (RunManifest::radiometric).
    std::string Summary() const {
        const auto& r = stage_.Report();
        if (r.mode == "off") return "off";
        std::ostringstream s;
        s << r.mode << ": " << r.decision;
        if (stage_.VstActive()) s << " (" << Params().denoiser << ", " << Params().inverse << " inverse)";
        s << "; flat field L=" << r.flat_field[0] << " R=" << r.flat_field[1];
        return s.str();
    }

    /// The results.yaml block; written only when the stage is enabled, so default runs keep their bytes.
    void WriteResults(std::ostream& out) const {
        const auto& r = stage_.Report();
        if (r.mode == "off") return;
        out << "radiometric: {mode: " << r.mode << ", decision: " << r.decision << ", reason: \"" << r.reason
            << "\", denoiser: " << Params().denoiser << ", inverse: " << Params().inverse
            << ", ms_per_image: " << r.MsPerImage() << ", images: " << r.frames << ",\n"
            << "  exposure_norm: \"" << r.exposure_norm << "\",\n";
        for (int c = 0; c < 2; ++c) {
            const auto& n = r.noise[c];
            out << "  " << (c == 0 ? "left" : "right") << ": {flat_field: \"" << r.flat_field[c]
                << "\", k_min: " << r.k_min[c] << ", fit_valid: " << (n.fit_valid ? "true" : "false")
                << ", why: \"" << n.why << "\", a: " << n.a << ", b: " << n.b << ", bins: " << n.bins
                << ", sigma_z: " << n.sigma_z << ", median_level: " << n.median_level
                << ", snr_at_median: " << n.snr_at_median << ", passes_gate: " << (n.passes_gate ? "true" : "false")
                << "}" << (c == 0 ? ",\n" : "}\n");
        }
    }

private:
    const perception::radiometric::RadiometricParams& Params() const { return stage_.Params(); }

    StereoSource& inner_;
    perception::radiometric::RadiometricStage stage_;
};

}  // namespace ariadne::runtime
