// 2.1 hard-pair path (PERCEPTION_2.1.md §12.3): HardPairMatcher's conventions. The model-backed tests use the
// ALIKED + LightGlue export in $ARIADNE_ALIKED_LG_REF (SKIPPED without it, FAILED under ARIADNE_REQUIRE_MODELS=1).
#include "perception/association/hard_pair_matcher.hpp"
#include "perception/features/xfeat/atb.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/features.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>

namespace fs = std::filesystem;
using namespace ariadne::perception;
using ariadne::perception::xfeat::LoadAtb;
using ariadne::perception::xfeat::Tensor;

namespace {

std::string RefDir(const char* var = "ARIADNE_ALIKED_LG_REF") {
    const char* d = std::getenv(var);
    return d ? d : "";
}
bool Required() {
    const char* r = std::getenv("ARIADNE_REQUIRE_MODELS");
    return r && std::string(r) == "1";
}
#define REQUIRE_EXPORT(dir)                                                                         \
    if ((dir).empty() || !fs::exists(fs::path(dir) / "manifest.yaml")) {                           \
        if (Required()) FAIL() << "ARIADNE_REQUIRE_MODELS=1 but no ALIKED/LightGlue export";        \
        GTEST_SKIP() << "no ALIKED/LightGlue export (set ARIADNE_ALIKED_LG_REF)";                  \
    }

HardPairParams Params(const std::string& dir, const std::string& norm, cv::Size size = {752, 480}) {
    HardPairParams p;
    p.aliked_onnx = (fs::path(dir) / ("aliked_n16_k1024_" + std::to_string(size.width) + "x" +
                                      std::to_string(size.height) + ".onnx")).string();
    p.lightglue_onnx = (fs::path(dir) / "lightglue_aliked.onnx").string();
    p.normalization = norm;
    p.width = size.width;
    p.height = size.height;
    return p;
}

/// HardPairFeatures from a reference pair file's upstream keypoints (stored normalised) and descriptors.
HardPairFeatures FromRef(const Tensor& kn, const Tensor& d, cv::Size size = {752, 480}) {
    HardPairFeatures f;
    f.size = size;
    const int n = int(kn.shape[0]);
    const float s = 0.5f * float(std::max(size.width, size.height)), cx = 0.5f * size.width, cy = 0.5f * size.height;
    for (int i = 0; i < n; ++i) {   // invert upstream's normalisation: px = kn * max(W,H)/2 + (W/2, H/2)
        f.keypoints.emplace_back(kn.f32[2 * size_t(i)] * s + cx, kn.f32[2 * size_t(i) + 1] * s + cy, 1.0f);
    }
    f.descriptors = cv::Mat(n, 128, CV_32F, const_cast<float*>(d.f32.data())).clone();
    return f;
}

}  // namespace

// Upstream normalize_keypoints with size (752, 480): shift (376, 240), ONE scale 376 for both axes.
TEST(HardPair, UpstreamNormalisationKeepsTheAspect) {
    const std::vector<cv::KeyPoint> kp = {{0, 0, 1}, {752, 480, 1}, {376, 240, 1}};
    const cv::Mat n = HardPairMatcher::NormalizeUpstream(kp, cv::Size(752, 480));
    EXPECT_FLOAT_EQ(n.at<float>(0, 0), -1.0f);
    EXPECT_FLOAT_EQ(n.at<float>(0, 1), -240.0f / 376.0f);   // per-axis (OpenCV's rule) would give -1
    EXPECT_FLOAT_EQ(n.at<float>(1, 0), 1.0f);
    EXPECT_FLOAT_EQ(n.at<float>(1, 1), 240.0f / 376.0f);
    EXPECT_FLOAT_EQ(n.at<float>(2, 0), 0.0f);
    EXPECT_FLOAT_EQ(n.at<float>(2, 1), 0.0f);
}

TEST(HardPair, RejectsBadParamsAndTheWrongImageSize) {
    HardPairParams bad;
    bad.normalization = "per-axis";
    EXPECT_THROW(HardPairMatcher{bad}, std::invalid_argument);
    const std::string dir = RefDir();
    REQUIRE_EXPORT(dir);
    const HardPairMatcher m(Params(dir, "upstream"));
    EXPECT_THROW(m.Extract(cv::Mat(720, 1280, CV_8UC1, cv::Scalar(0))), std::invalid_argument);
    EXPECT_THROW(m.Extract(cv::Mat(480, 752, CV_8UC3, cv::Scalar(0))), std::invalid_argument);
    EXPECT_EQ(m.Name(), "aliked_n16_k1024_752x480+lightglue_aliked/cv_dnn_new/norm_upstream");
}

// Through the wrapper, upstream mode reproduces upstream's match SET from upstream's features; OpenCV's per-axis
// mode runs on the same features and is reported (the §12.4 row-D question is measured on EuRoC, not here).
TEST(HardPair, UpstreamModeReproducesUpstreamMatches) {
    const std::string dir = RefDir();
    REQUIRE_EXPORT(dir);
    const HardPairMatcher up(Params(dir, "upstream")), cvm(Params(dir, "opencv"));
    for (const char* pair : {"mh01_f200_mh01_f205", "v101_a_v101_b"}) {
        const auto ref = LoadAtb((fs::path(dir) / ("ref_pair_" + std::string(pair) + "_k1024.atb")).string());
        const HardPairFeatures a = FromRef(ref.at("kpts0n"), ref.at("desc0")), b = FromRef(ref.at("kpts1n"), ref.at("desc1"));
        std::set<std::pair<int, int>> want;
        const auto& rm = ref.at("matches").i64;
        for (size_t i = 0; i + 1 < rm.size(); i += 2) want.insert({int(rm[i]), int(rm[i + 1])});
        std::set<std::pair<int, int>> got, got_cv;
        for (const auto& m : up.Match(a, b)) got.insert({m.queryIdx, m.trainIdx});
        for (const auto& m : cvm.Match(a, b)) got_cv.insert({m.queryIdx, m.trainIdx});
        int shared_cv = 0;
        for (const auto& m : got_cv) shared_cv += int(want.count(m));
        std::printf("[%s] upstream mode %zu (upstream %zu); opencv per-axis mode %zu, %d shared\n", pair, got.size(),
                    want.size(), got_cv.size(), shared_cv);
        EXPECT_EQ(got, want);
        EXPECT_GT(got_cv.size(), 0u);
    }
}

// The sim rig's RGB path (L3). ExtractRgb on the frame in RGB order is cv::ALIKED on the same frame in BGR (the
// reference test's input, checked against upstream there) to the byte; the frame in BGR order through ExtractRgb is a
// different input. Then upstream's sim match set through the wrapper at 1280x720.
TEST(HardPair, SimRgbPathIsTheReferenceInputAndSeesAChannelSwap) {
    const std::string dir = RefDir("ARIADNE_ALIKED_LG_SIM_REF");
    REQUIRE_EXPORT(dir);
    const cv::Size size(1280, 720);
    const HardPairMatcher m(Params(dir, "upstream", size));
    EXPECT_THROW(m.ExtractRgb(cv::Mat(720, 1280, CV_8UC1, cv::Scalar(0))), std::invalid_argument);
    EXPECT_THROW(m.ExtractRgb(cv::Mat(480, 752, CV_8UC3, cv::Scalar(0))), std::invalid_argument);
    cv::ALIKED::Params ap;
    ap.inputSize = size;
    ap.engine = cv::dnn::ENGINE_NEW;
    const auto al = cv::ALIKED::create(Params(dir, "upstream", size).aliked_onnx, ap);
    for (const char* name : {"sim_a0_f200", "sim_a0_f700"}) {
        const cv::Mat bgr = cv::imread((fs::path(dir) / ("input_" + std::string(name) + ".png")).string(), cv::IMREAD_COLOR);
        ASSERT_EQ(bgr.size(), size);
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
        al->detectAndCompute(bgr, cv::noArray(), kp, desc);
        const HardPairFeatures f = m.ExtractRgb(rgb), sw = m.ExtractRgb(bgr);
        ASSERT_EQ(f.keypoints.size(), kp.size());
        bool same_kp = true;
        for (size_t i = 0; i < kp.size(); ++i) same_kp &= kp[i].pt == f.keypoints[i].pt && kp[i].response == f.keypoints[i].response;
        EXPECT_TRUE(same_kp) << name;
        EXPECT_EQ(cv::norm(desc, f.descriptors, cv::NORM_INF), 0.0) << name;
        std::set<std::pair<float, float>> a, b;
        for (const auto& k : f.keypoints) a.insert({k.pt.x, k.pt.y});
        for (const auto& k : sw.keypoints) b.insert({k.pt.x, k.pt.y});
        int shared = 0;
        for (const auto& p : b) shared += int(a.count(p));
        std::printf("[%s] RGB path = reference input; BGR-swapped input keeps %d of %zu keypoints exactly\n", name, shared,
                    a.size());
        EXPECT_LT(shared, int(a.size()) * 9 / 10);
    }
    for (const char* pair : {"sim_a0_f200_sim_a0_f205", "sim_a0_f700_sim_a0_f710"}) {
        const auto ref = LoadAtb((fs::path(dir) / ("ref_pair_" + std::string(pair) + "_k1024.atb")).string());
        const HardPairFeatures a = FromRef(ref.at("kpts0n"), ref.at("desc0"), size),
                               b = FromRef(ref.at("kpts1n"), ref.at("desc1"), size);
        std::set<std::pair<int, int>> want, got;
        const auto& rm = ref.at("matches").i64;
        for (size_t i = 0; i + 1 < rm.size(); i += 2) want.insert({int(rm[i]), int(rm[i + 1])});
        for (const auto& x : m.Match(a, b)) got.insert({x.queryIdx, x.trainIdx});
        std::printf("[%s] wrapper %zu matches, upstream %zu\n", pair, got.size(), want.size());
        EXPECT_EQ(got, want);
    }
}

#ifdef ARIADNE_HAVE_ORT
// The ONNX Runtime path (the live GPU route) must reproduce cv::dnn's on the same export: same keypoints (to float
// round-off), same descriptors, and the same LightGlue matches. Run here on ORT's CPU provider, so it needs no GPU;
// the CUDA provider runs the same graph and is checked by the GPU smoke test.
TEST(HardPair, OrtPathReproducesCvDnnOnTheSimExport) {
    const std::string dir = RefDir("ARIADNE_ALIKED_LG_SIM_REF");
    REQUIRE_EXPORT(dir);
    const cv::Size size(1280, 720);
    HardPairParams pc = Params(dir, "upstream", size), po = pc;
    po.backend = "ort_cpu";
    const HardPairMatcher cvm(pc), ort(po);
    EXPECT_EQ(ort.Records().front().engine, "onnxruntime_cpu");
    for (const char* pair : {"sim_a0_f200_sim_a0_f205", "sim_a0_f700_sim_a0_f710"}) {
        const std::string p(pair);
        HardPairFeatures fc[2], fo[2];
        for (int k = 0; k < 2; ++k) {
            const std::string name = p.substr(k ? p.size() / 2 + 1 : 0, p.size() / 2);
            const cv::Mat bgr = cv::imread((fs::path(dir) / ("input_" + name + ".png")).string(), cv::IMREAD_COLOR);
            ASSERT_EQ(bgr.size(), size) << name;
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            fc[k] = cvm.ExtractRgb(rgb), fo[k] = ort.ExtractRgb(rgb);
            ASSERT_EQ(fc[k].keypoints.size(), fo[k].keypoints.size()) << name;
            // The two runtimes may list the same top-K in a different order (ties in the score sort), so pair each
            // cv::dnn keypoint with the ORT keypoint at the same position, then compare score and descriptor.
            const size_t n = fc[k].keypoints.size();
            size_t paired = 0;
            double dsc = 0, dd = 0;
            for (size_t i = 0; i < n; ++i) {
                size_t best = n;
                double bd = 1e-2;   // px
                for (size_t j = 0; j < n; ++j) {
                    const double d = cv::norm(fc[k].keypoints[i].pt - fo[k].keypoints[j].pt);
                    if (d < bd) bd = d, best = j;
                }
                if (best == n) continue;
                ++paired;
                dsc = std::max(dsc, double(std::abs(fc[k].keypoints[i].response - fo[k].keypoints[best].response)));
                dd = std::max(dd, cv::norm(fc[k].descriptors.row(int(i)), fo[k].descriptors.row(int(best)), cv::NORM_INF));
            }
            std::printf("[%s] ort vs cv::dnn: %zu keypoints, %zu at the same position, max |dscore| %.2e, max |ddesc| %.2e\n",
                        name.c_str(), n, paired, dsc, dd);
            EXPECT_GE(paired, n * 99 / 100) << name;
            EXPECT_LT(dsc, 1e-3) << name;
            EXPECT_LT(dd, 1e-3) << name;
        }
        // LightGlue on cv::dnn features through both backends: the same matches.
        const auto mc = cvm.Match(fc[0], fc[1]), mo = ort.Match(fc[0], fc[1]);
        std::set<std::pair<int, int>> a, b;
        for (const auto& m : mc) a.insert({m.queryIdx, m.trainIdx});
        for (const auto& m : mo) b.insert({m.queryIdx, m.trainIdx});
        int both = 0;
        for (const auto& x : a) both += int(b.count(x));
        const double jac = double(both) / double(std::max<size_t>(1, a.size() + b.size() - size_t(both)));
        std::printf("[%s] LightGlue cv::dnn %zu matches, ort %zu, Jaccard %.4f\n", pair, a.size(), b.size(), jac);
        EXPECT_GT(a.size(), 100u) << pair;
        EXPECT_GE(jac, 0.98) << pair;
    }
}

// ort_cuda either runs on CUDA or falls back for a CUDA reason (no GPU or libraries here), never because ONNX
// Runtime's logger did not exist yet when the provider was appended (g4dn run 7b34cf, ort_session.cpp).
TEST(HardPair, OrtCudaFallbackIsNeverTheLoggerOrder) {
    const std::string dir = RefDir("ARIADNE_ALIKED_LG_SIM_REF");
    REQUIRE_EXPORT(dir);
    HardPairParams p = Params(dir, "upstream", cv::Size(1280, 720));
    p.backend = "ort_cuda";
    const HardPairMatcher m(p);
    const std::string engine = m.Records().front().engine;
    std::printf("ort_cuda engine: %s\n", engine.c_str());
    EXPECT_EQ(engine.find("DefaultLogger"), std::string::npos) << engine;
}
#endif
