#include "perception/dnn/ort_session.hpp"

#include <onnxruntime_cxx_api.h>

#include <cstring>
#include <stdexcept>

namespace ariadne::perception::dnn {

namespace {
Ort::Env& Env() {   // one per process, as ONNX Runtime requires
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ariadne");
    return env;
}
}  // namespace

struct OrtNet::Impl {
    Ort::Session session{nullptr};
};

OrtNet::OrtNet(const std::string& onnx, const std::string& provider, int cpu_threads, int device)
    : impl_(std::make_unique<Impl>()) {
    if (provider != "cuda" && provider != "cpu") throw std::invalid_argument("OrtNet: provider cuda|cpu");
    auto Options = [&](bool cuda) {
        Ort::SessionOptions so;
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        so.SetIntraOpNumThreads(cpu_threads);
        so.SetInterOpNumThreads(1);
        if (cuda) {
            OrtCUDAProviderOptions co{};
            co.device_id = device;
            so.AppendExecutionProvider_CUDA(co);
        }
        return so;
    };
    if (provider == "cuda") {
        try {
            impl_->session = Ort::Session(Env(), onnx.c_str(), Options(true));
            provider_ = "cuda";
            return;
        } catch (const Ort::Exception& e) {
            provider_ = std::string("cpu (cuda unavailable: ") + e.what() + ")";
        }
    } else {
        provider_ = "cpu";
    }
    impl_->session = Ort::Session(Env(), onnx.c_str(), Options(false));
}

OrtNet::~OrtNet() = default;

std::vector<cv::Mat> OrtNet::Run(const std::vector<std::pair<std::string, cv::Mat>>& inputs,
                                 const std::vector<std::string>& outputs) {
    const Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> in_values;
    std::vector<const char*> in_names, out_names;
    std::vector<cv::Mat> keep;   // continuous float copies alive through Run
    for (const auto& [name, m] : inputs) {
        if (m.depth() != CV_32F) throw std::invalid_argument("OrtNet: float inputs only (" + name + ")");
        keep.push_back(m.isContinuous() ? m : m.clone());
        const cv::Mat& c = keep.back();
        std::vector<int64_t> shape;
        for (int d = 0; d < c.dims; ++d) shape.push_back(c.size[d]);
        if (c.channels() > 1) shape.push_back(c.channels());
        in_values.push_back(Ort::Value::CreateTensor<float>(mem, const_cast<float*>(c.ptr<float>()), c.total() * size_t(c.channels()),
                                                            shape.data(), shape.size()));
        in_names.push_back(name.c_str());
    }
    for (const auto& n : outputs) out_names.push_back(n.c_str());
    auto out = impl_->session.Run(Ort::RunOptions{nullptr}, in_names.data(), in_values.data(), in_values.size(),
                                  out_names.data(), out_names.size());
    std::vector<cv::Mat> res;
    for (auto& v : out) {
        const auto info = v.GetTensorTypeAndShapeInfo();
        const std::vector<int64_t> shape = info.GetShape();
        std::vector<int> sz;
        for (const int64_t d : shape) sz.push_back(int(d));
        if (sz.empty()) sz.push_back(1);
        if (sz.size() == 1) sz.push_back(1);   // cv::Mat needs >= 2 dims: [n] -> n x 1
        const size_t n = info.GetElementCount();
        switch (info.GetElementType()) {
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: {
                cv::Mat m(int(sz.size()), sz.data(), CV_32F);
                if (n) std::memcpy(m.ptr<float>(), v.GetTensorData<float>(), n * sizeof(float));
                res.push_back(m);
                break;
            }
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: {
                cv::Mat m(int(sz.size()), sz.data(), CV_32S);
                const int64_t* p = v.GetTensorData<int64_t>();
                int* q = m.ptr<int>();
                for (size_t i = 0; i < n; ++i) q[i] = int(p[i]);
                res.push_back(m);
                break;
            }
            default:
                throw std::runtime_error("OrtNet: unsupported output element type");
        }
    }
    return res;
}

}  // namespace ariadne::perception::dnn
