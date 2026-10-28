#pragma once
// ---------------------------------------------------------------------------
// One ONNX Runtime session (ARIADNE_WITH_ORT builds only): the live GPU path for the hard-pair matcher's ALIKED +
// LightGlue and the place recognizer's EigenPlaces, on the SAME ONNX exports cv::dnn reads. Why not cv::dnn on
// CUDA: in OpenCV 5.0.0, ENGINE_NEW supports no non-CPU backend (dnn.hpp) and ENGINE_CLASSIC cannot import the
// ALIKED/LightGlue graphs (SMOKE §4s). cv::dnn stays the default and the evidence path.
//
// Provider "cuda" asks for ONNX Runtime's CUDA execution provider and falls back to its CPU provider when the CUDA
// one cannot be created (no GPU, no CUDA/cuDNN libraries); Provider() says which one actually runs, for the
// manifest ("log which ran"). The CPU provider is bounded to `cpu_threads` intra-op threads.
//
// Inputs and outputs are cv::Mat: float inputs (any dims, continuous); float outputs come back CV_32F, int64
// outputs CV_32S (indices), each with the graph's own output shape.
// ---------------------------------------------------------------------------
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ariadne::perception::dnn {

class OrtNet {
public:
    OrtNet(const std::string& onnx, const std::string& provider, int cpu_threads = 2, int device = 0);
    ~OrtNet();
    OrtNet(const OrtNet&) = delete;
    OrtNet& operator=(const OrtNet&) = delete;

    std::vector<cv::Mat> Run(const std::vector<std::pair<std::string, cv::Mat>>& inputs,
                             const std::vector<std::string>& outputs);
    /// "cuda" or "cpu" (the one in use), plus " (cuda unavailable: <why>)" after a fallback.
    const std::string& Provider() const { return provider_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string provider_;
};

}  // namespace ariadne::perception::dnn
