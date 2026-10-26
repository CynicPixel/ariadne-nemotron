// L2 (D101): the seeded sim IMU noise operator -- determinism and the discretised noise statistics.
#include "core/inertial/imu_noise_operator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using namespace ariadne::core;

namespace {

std::vector<ImuMeasurement> Clean(size_t n) {
    std::vector<ImuMeasurement> v(n);
    for (size_t k = 0; k < n; ++k) {
        v[k].t_ns = static_cast<int64_t>(k) * 5000000;
        v[k].gyro = Vec3(0.1, -0.2, 0.3) * std::sin(0.01 * double(k));
        v[k].accel = Vec3(0.0, 0.0, 9.81);
    }
    return v;
}

ImuNoiseOperatorParams Params(double nd_g, double rw_g, double nd_a, double rw_a) {
    ImuNoiseOperatorParams p;
    p.name = "test";
    p.noise = {nd_g, rw_g, nd_a, rw_a, 200.0};
    return p;
}

bool BitEqual(const ImuMeasurement& a, const ImuMeasurement& b) {
    return a.t_ns == b.t_ns && std::memcmp(a.gyro.data(), b.gyro.data(), sizeof(double) * 3) == 0 &&
           std::memcmp(a.accel.data(), b.accel.data(), sizeof(double) * 3) == 0;
}

}  // namespace

// Bit-identical per (seed, sample): a replay of any prefix (fewer frames) reproduces the full run's samples exactly,
// and a second call is identical. The bias random walk is a running sum, so this is the property that matters.
TEST(ImuNoise, BitIdenticalPerSeedAndSampleWhateverTheReplayLength) {
    const auto p = Params(1.6968e-4, 1.9393e-5, 2.0e-3, 3.0e-3);
    const auto clean = Clean(4000);
    const auto full = ApplyImuNoise(clean, p, 42);
    const auto again = ApplyImuNoise(clean, p, 42);
    for (size_t n : {size_t(1), size_t(17), size_t(1000), size_t(3999)}) {
        const auto prefix = ApplyImuNoise(std::vector<ImuMeasurement>(clean.begin(), clean.begin() + n), p, 42);
        for (size_t k = 0; k < n; ++k) ASSERT_TRUE(BitEqual(prefix[k], full[k])) << "n " << n << " k " << k;
    }
    for (size_t k = 0; k < full.size(); ++k) ASSERT_TRUE(BitEqual(again[k], full[k]));
    const auto other = ApplyImuNoise(clean, p, 43);
    EXPECT_FALSE(BitEqual(other[10], full[10]));
}

TEST(ImuNoise, IncrementalStreamMatchesBatchAcrossArbitraryChunks) {
    const auto p = Params(1.6968e-4, 1.9393e-5, 2.0e-3, 3.0e-3);
    const auto clean = Clean(4073);
    const auto batch = ApplyImuNoise(clean, p, 9182);
    ImuNoiseStream stream(p, 9182);
    std::vector<ImuMeasurement> streamed;
    for (size_t begin = 0; begin < clean.size();) {
        const size_t count = std::min<size_t>((begin % 137) + 1, clean.size() - begin);
        for (size_t k = begin; k < begin + count; ++k) streamed.push_back(stream.Apply(clean[k]));
        begin += count;
    }
    ASSERT_EQ(streamed.size(), batch.size());
    for (size_t k = 0; k < batch.size(); ++k) EXPECT_TRUE(BitEqual(streamed[k], batch[k])) << "sample " << k;
}

TEST(ImuNoise, ZeroDensitiesReturnTheCleanStreamPlusTheInitialBias) {
    auto p = Params(0, 0, 0, 0);
    p.gyro_bias0 = Vec3(0.01, 0, 0);
    const auto clean = Clean(50);
    const auto out = ApplyImuNoise(clean, p, 7);
    for (size_t k = 0; k < clean.size(); ++k) {
        EXPECT_EQ(out[k].gyro, clean[k].gyro + p.gyro_bias0);
        EXPECT_EQ(out[k].accel, clean[k].accel);
    }
}

// White noise: per-sample std = density / sqrt(dt). 3 axes x 60k samples: the std estimate is good to ~0.4 %.
TEST(ImuNoise, WhiteNoiseStdIsDensityOverSqrtDt) {
    const double nd_g = 1.6968e-4, nd_a = 2.0e-3, dt = 1.0 / 200.0;
    const auto clean = Clean(60000);
    const auto out = ApplyImuNoise(clean, Params(nd_g, 0, nd_a, 0), 11);
    double sg = 0, sa = 0;
    for (size_t k = 0; k < out.size(); ++k) {
        sg += (out[k].gyro - clean[k].gyro).squaredNorm();
        sa += (out[k].accel - clean[k].accel).squaredNorm();
    }
    EXPECT_NEAR(std::sqrt(sg / (3.0 * out.size())) / (nd_g / std::sqrt(dt)), 1.0, 0.02);
    EXPECT_NEAR(std::sqrt(sa / (3.0 * out.size())) / (nd_a / std::sqrt(dt)), 1.0, 0.02);
}

// Bias random walk: Var(b(T)) = rw^2 T. 300 seeds x 3 axes = 900 independent finals after T = 10 s (2000 steps):
// the variance estimate's relative sd is sqrt(2/900) = 4.7 %, so 20 % is a ~4 sigma bound.
TEST(ImuNoise, BiasRandomWalkVarianceGrowsAsDensitySquaredTimesT) {
    const double rw_g = 1.0e-3, rw_a = 3.0e-3;
    const size_t steps = 2001;   // the output at index 2000 carries 2000 steps of the walk
    const double T = 2000.0 / 200.0;
    const auto clean = Clean(steps);
    double vg = 0, va = 0;
    const int seeds = 300;
    for (int s = 0; s < seeds; ++s) {
        const auto out = ApplyImuNoise(clean, Params(0, rw_g, 0, rw_a), 1000 + s);
        vg += (out.back().gyro - clean.back().gyro).squaredNorm();
        va += (out.back().accel - clean.back().accel).squaredNorm();
    }
    EXPECT_NEAR(vg / (3.0 * seeds) / (rw_g * rw_g * T), 1.0, 0.2);
    EXPECT_NEAR(va / (3.0 * seeds) / (rw_a * rw_a * T), 1.0, 0.2);
}

TEST(ImuNoise, ShippedConfigLoadsAndMatchesTheRigRate) {
    const auto p = LoadImuNoiseOperator(std::string(ARIADNE_SOURCE_DIR) + "/configs/sim/imu_noise_v1.yaml");
    EXPECT_EQ(p.name, "imu_noise_v1");
    EXPECT_DOUBLE_EQ(p.noise.rate_hz, 200.0);   // rig_stereo_v1 imu.period_ns = 5 ms
    EXPECT_DOUBLE_EQ(p.noise.gyro_noise_density, 1.6968e-4);
    EXPECT_EQ(p.gyro_bias0, Vec3::Zero());
}
