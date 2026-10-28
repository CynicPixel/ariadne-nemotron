#pragma once
// ---------------------------------------------------------------------------
// Hard-pair matcher (arch §7.2 "ALIKED + LightGlue on hard pairs only", C8;
// perception.md §12): the expensive verifier for rare, high-value pairs,
// such as loop or inter-agent candidates that survived retrieval. Never at
// frame rate (≈ 0.5 s per extraction, ≈ 0.6 s per match at K = 1024 on this CPU).
//
//   ALIKED-n16 (Zhao et al., T-IM 2023): a CNN keypoint detector and 128-D
//     descriptor whose head samples features at learned, deformable positions
//     around each keypoint. Top-K keypoints, sub-pixel.
//   LightGlue (Lindenberger et al., ICCV 2023): a transformer that matches two
//     keypoint sets jointly. Each keypoint attends to its own image
//     (self-attention, with a rotary encoding of its normalised position) and
//     to the other image (cross-attention); a dual-softmax assignment plus a
//     per-point "matchability" gives mutual matches with a confidence.
//
// Both run through OpenCV's own classes (cv::ALIKED, cv::LightGlueMatcher;
// cv::dnn ENGINE_NEW) on our export from upstream
// (python/model_tools/aliked_lightglue_reference.py; exact against upstream,
// SMOKE §4s-2). Conventions this wrapper owns, all from SMOKE §4s:
//   - the image size must equal the export's (the ONNX input is fixed; a
//     1280x720 rig needs its own export), so Extract throws otherwise;
//   - keypoints reach LightGlue normalised as upstream does (shift W/2, H/2;
//     scale max(W,H)/2) through setPairInfo(..., Size(), Size()).
//     normalization "opencv" instead passes pixels and sizes, so OpenCV
//     rescales each axis to [-1, 1] on its own (the ablation, §12.4 row D);
//   - a cv::dnn::Net is not thread-safe and LightGlueMatcher::clone() shares
//     its Net (§7.4), so the ALIKED + LightGlue pair lives in a
//     dnn::PerThread (D91): one HardPairMatcher may be shared by threads, each
//     calling thread loads and runs its own pair.
// ---------------------------------------------------------------------------
#include "perception/dnn/per_thread.hpp"
#ifdef ARIADNE_HAVE_ORT
#include "perception/dnn/ort_session.hpp"
#endif

#include <opencv2/core.hpp>
#include <opencv2/features.hpp>

#include <string>
#include <vector>

namespace ariadne::perception {

struct HardPairParams {
    std::string aliked_onnx;          // aliked_n16_k<K>_<W>x<H>.onnx
    std::string lightglue_onnx;       // lightglue_aliked.onnx
    int width = 752, height = 480;    // the ALIKED export's fixed input size
    std::string normalization = "upstream";   // upstream | opencv
    float min_score = 0.0f;           // on top of the graph's own filter (LightGlue filter_threshold 0.1)
    // cv_dnn (default, the evidence path) | ort_cuda | ort_cpu: the same ONNX files through ONNX Runtime
    // (ARIADNE_WITH_ORT builds; ort_session.hpp says why cv::dnn cannot reach the GPU for these graphs). The ORT
    // path reproduces cv::ALIKED / cv::LightGlueMatcher's pre- and post-processing exactly (tested against them).
    std::string backend = "cv_dnn";
};

struct HardPairFeatures {
    std::vector<cv::KeyPoint> keypoints;   // pixels; response = ALIKED score
    cv::Mat descriptors;                   // K x 128 CV_32F, unit rows
    cv::Size size;
};

class HardPairMatcher {
public:
    explicit HardPairMatcher(const HardPairParams& p);

    /// ALIKED on an 8-bit gray image of exactly the export's size.
    HardPairFeatures Extract(const cv::Mat& gray) const;
    /// ALIKED on an 8-bit image in RGB ORDER (CV_8UC3, StereoFrame::left_rgb) of exactly the export's size: the
    /// sim rig's path. cv::ALIKED itself expects BGR (blobFromImage swapRB), so this converts RGB -> BGR first.
    HardPairFeatures ExtractRgb(const cv::Mat& rgb) const;

    /// LightGlue matches a -> b (queryIdx in a, trainIdx in b, distance = 1 - confidence).
    std::vector<cv::DMatch> Match(const HardPairFeatures& a, const HardPairFeatures& b) const;

    /// Upstream's normalize_keypoints with size = (W, H): N x 2 CV_32F.
    static cv::Mat NormalizeUpstream(const std::vector<cv::KeyPoint>& kp, cv::Size size);

    /// Manifest id, e.g. "aliked_n16_k1024_752x480+lightglue_aliked/cv_dnn_new/norm_upstream".
    std::string Name() const;
    /// The manifest records (§7.4): ALIKED and LightGlue, both cv::dnn ENGINE_NEW.
    std::vector<dnn::ModelRecord> Records() const;

private:
    struct Nets {
        cv::Ptr<cv::ALIKED> aliked;
        cv::Ptr<cv::LightGlueMatcher> lightglue;
#ifdef ARIADNE_HAVE_ORT
        std::unique_ptr<dnn::OrtNet> ort_aliked, ort_lightglue;
#endif
    };
    void CheckSize(const cv::Mat& im) const;
    void Detect(const cv::Mat& bgr, HardPairFeatures* f) const;
    HardPairParams p_;
    dnn::PerThread<Nets> nets_;
};

}  // namespace ariadne::perception
