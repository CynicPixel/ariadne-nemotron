#pragma once
// ---------------------------------------------------------------------------
// L2 (D101): the seeded sim IMU noise operator -- D's counterpart for the IMU.
//
// The sim records a CLEAN IMU stream (rig imu.period_ns; D37: noise is a
// consumption-time operator). This applies the standard continuous-time model
// (Kalibr / EuRoC sensor.yaml convention, Forster et al. T-RO 2017 §IV),
// discretised at the nominal rate 1/dt:
//
//   gyro_k  = gyro_true_k  + b_g,k + (sigma_g  / sqrt(dt)) n_k
//   accel_k = accel_true_k + b_a,k + (sigma_a  / sqrt(dt)) n'_k
//   b_{k+1} = b_k + (sigma_bg * sqrt(dt)) m_k     (per sensor; b_0 = the config's initial bias)
//
// with n, n', m standard normal. sigma_* are noise densities (unit / sqrt(Hz)),
// sigma_b* random-walk densities (unit / s / sqrt(Hz)).
//
// Determinism (as D, §5.6): every draw for sample k comes from
// PhiloxStream(seed, k, 0) -- a keyed hash of (seed, sample index), never a
// sequential generator. White noise at k is a function of (seed, k) alone;
// the bias at k is the running sum of the steps 0..k-1, so the first n
// outputs are bit-identical whatever the stream's length or how it is
// replayed. `seed` is per (run, agent): SimSource derives it from the run seed.
// ---------------------------------------------------------------------------
#include "core/config/config.hpp"
#include "core/inertial/imu.hpp"
#include "core/random/distributions.hpp"
#include "core/random/philox.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace ariadne::core {

struct ImuNoiseOperatorParams {
    std::string name = "none";
    ImuNoise noise;               // densities + rate_hz (the dt of the model)
    Vec3 gyro_bias0 = Vec3::Zero();    // rad/s, b_g,0
    Vec3 accel_bias0 = Vec3::Zero();   // m/s^2, b_a,0
};

/// Incremental form of ApplyImuNoise for a live stream. It preserves the same
/// Philox key, draw order, arithmetic order, and running bias state as the
/// batch operator, so live chunks are bit-identical to a recorded replay.
class ImuNoiseStream {
public:
    ImuNoiseStream(const ImuNoiseOperatorParams& p, uint64_t seed)
        : p_(p), seed_(seed), bg_(p.gyro_bias0), ba_(p.accel_bias0) {
        if (!(p_.noise.rate_hz > 0.0)) throw std::invalid_argument("IMU noise stream rate_hz must be > 0");
        const double dt = 1.0 / p_.noise.rate_hz;
        sw_g_ = p_.noise.gyro_noise_density / std::sqrt(dt);
        sw_a_ = p_.noise.accel_noise_density / std::sqrt(dt);
        sb_g_ = p_.noise.gyro_random_walk * std::sqrt(dt);
        sb_a_ = p_.noise.accel_random_walk * std::sqrt(dt);
    }

    ImuMeasurement Apply(const ImuMeasurement& clean) {
        PhiloxStream s(seed_, index_++, 0);
        Vec3 ng, na, mg, ma;
        for (int i = 0; i < 3; ++i) ng[i] = SampleStandardNormal(s);
        for (int i = 0; i < 3; ++i) na[i] = SampleStandardNormal(s);
        for (int i = 0; i < 3; ++i) mg[i] = SampleStandardNormal(s);
        for (int i = 0; i < 3; ++i) ma[i] = SampleStandardNormal(s);
        ImuMeasurement out;
        out.t_ns = clean.t_ns;
        out.gyro = clean.gyro + bg_ + sw_g_ * ng;
        out.accel = clean.accel + ba_ + sw_a_ * na;
        bg_ += sb_g_ * mg;
        ba_ += sb_a_ * ma;
        return out;
    }

private:
    ImuNoiseOperatorParams p_;
    uint64_t seed_ = 0, index_ = 0;
    Vec3 bg_, ba_;
    double sw_g_ = 0, sw_a_ = 0, sb_g_ = 0, sb_a_ = 0;
};

/// A rig-scoped IMU noise config (configs/sim/imu_noise_*.yaml); strict keys.
inline ImuNoiseOperatorParams ParseImuNoiseOperator(StrictConfig& c) {
    ImuNoiseOperatorParams p;
    p.name = c.Require<std::string>("name");
    c.Require<std::string>("rig");   // documentation of which rig it models; not used numerically
    p.noise.rate_hz = c.Require<double>("rate_hz");
    p.noise.gyro_noise_density = c.Require<double>("gyroscope_noise_density");
    p.noise.gyro_random_walk = c.Require<double>("gyroscope_random_walk");
    p.noise.accel_noise_density = c.Require<double>("accelerometer_noise_density");
    p.noise.accel_random_walk = c.Require<double>("accelerometer_random_walk");
    const auto vec = [&](const char* key) {
        const std::vector<double> v = c.Optional<std::vector<double>>(key, {0.0, 0.0, 0.0});
        if (v.size() != 3) throw ConfigError(c.Origin() + ": " + key + " must have 3 entries");
        return Vec3(v[0], v[1], v[2]);
    };
    p.gyro_bias0 = vec("gyroscope_bias_initial");
    p.accel_bias0 = vec("accelerometer_bias_initial");
    c.RequireAllConsumed();
    const ImuNoise& n = p.noise;
    if (!(n.rate_hz > 0.0) || n.gyro_noise_density < 0.0 || n.gyro_random_walk < 0.0 ||
        n.accel_noise_density < 0.0 || n.accel_random_walk < 0.0) {
        throw ConfigError(c.Origin() + ": rate_hz must be > 0 and every density >= 0");
    }
    return p;
}

inline ImuNoiseOperatorParams LoadImuNoiseOperator(const std::string& path) {
    StrictConfig c = StrictConfig::FromFile(path);
    return ParseImuNoiseOperator(c);
}

/// The noisy stream, parallel to `clean` (sample k = clean[k], in stream order).
inline std::vector<ImuMeasurement> ApplyImuNoise(const std::vector<ImuMeasurement>& clean,
                                                 const ImuNoiseOperatorParams& p, uint64_t seed) {
    ImuNoiseStream stream(p, seed);
    std::vector<ImuMeasurement> out(clean.size());
    for (size_t k = 0; k < clean.size(); ++k) out[k] = stream.Apply(clean[k]);
    return out;
}

}  // namespace ariadne::core
