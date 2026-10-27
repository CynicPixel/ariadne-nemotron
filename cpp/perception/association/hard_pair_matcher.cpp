#include "perception/association/hard_pair_matcher.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace ariadne::perception {

namespace {

cv::Mat Pixels(const std::vector<cv::KeyPoint>& kp) {
    cv::Mat m(static_cast<int>(kp.size()), 2, CV_32F);
    for (size_t i = 0; i < kp.size(); ++i) {
        m.at<float>(static_cast<int>(i), 0) = kp[i].pt.x;
        m.at<float>(static_cast<int>(i), 1) = kp[i].pt.y;
    }
    return m;
}

HardPairParams Checked(const HardPairParams& p) {
    if (p.normalization != "upstream" && p.normalization != "opencv") {
        throw std::invalid_argument("HardPairMatcher: normalization upstream|opencv");
    }
    if (p.width <= 0 || p.height <= 0) throw std::invalid_argument("HardPairMatcher: size");
    if (p.backend != "cv_dnn" && p.backend != "ort_cuda" && p.backend != "ort_cpu") {
        throw std::invalid_argument("HardPairMatcher: backend cv_dnn|ort_cuda|ort_cpu");
    }
#ifndef ARIADNE_HAVE_ORT
    if (p.backend != "cv_dnn") throw std::invalid_argument("HardPairMatcher: " + p.backend + " needs an ARIADNE_WITH_ORT build");
#endif
    return p;
}

}  // namespace

HardPairMatcher::HardPairMatcher(const HardPairParams& p)
    : p_(Checked(p)), nets_([q = p_] {
          cv::ALIKED::Params ap;
          ap.inputSize = cv::Size(q.width, q.height);   // = the image size, so blobFromImage does not resize
          ap.engine = cv::dnn::ENGINE_NEW;              // ENGINE_CLASSIC cannot import either graph (SMOKE §4s)
          auto n = std::make_unique<Nets>();
#ifdef ARIADNE_HAVE_ORT
          if (q.backend != "cv_dnn") {
              const std::string prov = q.backend == "ort_cuda" ? "cuda" : "cpu";
              n->ort_aliked = std::make_unique<dnn::OrtNet>(q.aliked_onnx, prov);
              n->ort_lightglue = std::make_unique<dnn::OrtNet>(q.lightglue_onnx, prov);
              return n;
          }
#endif
          n->aliked = cv::ALIKED::create(q.aliked_onnx, ap);
          n->lightglue = cv::LightGlueMatcher::create(q.lightglue_onnx, q.min_score);
          if (n->aliked.empty() || n->aliked->empty() || n->lightglue.empty()) {
              throw std::runtime_error("HardPairMatcher: cannot load " + q.aliked_onnx + " / " + q.lightglue_onnx);
          }
          return n;
      }) {
    nets_.Local();   // load on the constructing thread: a bad file fails here
}

void HardPairMatcher::CheckSize(const cv::Mat& im) const {
    if (im.cols != p_.width || im.rows != p_.height) {
        throw std::invalid_argument("HardPairMatcher: image " + std::to_string(im.cols) + "x" +
                                    std::to_string(im.rows) + " but the export is " + std::to_string(p_.width) +
                                    "x" + std::to_string(p_.height));
    }
}

HardPairFeatures HardPairMatcher::ExtractRgb(const cv::Mat& rgb) const {
    if (rgb.empty() || rgb.type() != CV_8UC3) throw std::invalid_argument("HardPairMatcher: expected CV_8UC3 (RGB)");
    CheckSize(rgb);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    HardPairFeatures f;
    f.size = rgb.size();
    Detect(bgr, &f);
    return f;
}

HardPairFeatures HardPairMatcher::Extract(const cv::Mat& gray) const {
    if (gray.empty() || gray.type() != CV_8UC1) throw std::invalid_argument("HardPairMatcher: expected CV_8UC1");
    CheckSize(gray);
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);   // gray replicated: the model is RGB-trained
    HardPairFeatures f;
    f.size = gray.size();
    Detect(bgr, &f);
    return f;
}

void HardPairMatcher::Detect(const cv::Mat& bgr, HardPairFeatures* f) const {
#ifdef ARIADNE_HAVE_ORT
    if (dnn::OrtNet* net = nets_.Local().ort_aliked.get()) {   // cv::ALIKED's runNetwork, step for step
        const cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(p_.width, p_.height), cv::Scalar(),
                                                    /*swapRB=*/true, /*crop=*/false);
        const std::vector<cv::Mat> out = net->Run({{"image", blob}}, {"keypoints", "descriptors", "scores"});
        const int n = int(out[0].total() / 2);
        const cv::Mat kp = out[0].reshape(1, n), sc = out[2].reshape(1, n);
        cv::Mat desc = out[1].reshape(1, n).clone();
        f->keypoints.resize(size_t(n));
        for (int i = 0; i < n; ++i) {
            const float px = (kp.at<float>(i, 0) + 1.0f) * 0.5f * float(f->size.width);
            const float py = (kp.at<float>(i, 1) + 1.0f) * 0.5f * float(f->size.height);
            f->keypoints[size_t(i)] = cv::KeyPoint(px, py, 1.0f, -1.0f, sc.at<float>(i, 0), 0, -1);
            cv::Mat row = desc.row(i);
            cv::normalize(row, row);   // ALIKED::Params::normalizeDescriptors = true (the default we use)
        }
        f->descriptors = desc;
        return;
    }
#endif
    nets_.Local().aliked->detectAndCompute(bgr, cv::noArray(), f->keypoints, f->descriptors);
}

cv::Mat HardPairMatcher::NormalizeUpstream(const std::vector<cv::KeyPoint>& kp, cv::Size size) {
    const float sx = 0.5f * static_cast<float>(size.width), sy = 0.5f * static_cast<float>(size.height);
    const float scale = 0.5f * static_cast<float>(std::max(size.width, size.height));
    cv::Mat m = Pixels(kp);
    for (int i = 0; i < m.rows; ++i) {
        m.at<float>(i, 0) = (m.at<float>(i, 0) - sx) / scale;
        m.at<float>(i, 1) = (m.at<float>(i, 1) - sy) / scale;
    }
    return m;
}

std::vector<cv::DMatch> HardPairMatcher::Match(const HardPairFeatures& a, const HardPairFeatures& b) const {
    std::vector<cv::DMatch> out;
    if (a.keypoints.empty() || b.keypoints.empty()) return out;
#ifdef ARIADNE_HAVE_ORT
    if (dnn::OrtNet* net = nets_.Local().ort_lightglue.get()) {   // cv::LightGlueMatcher::lightglueMatch + match()
        auto Norm = [&](const HardPairFeatures& f) {
            if (p_.normalization == "upstream") return NormalizeUpstream(f.keypoints, f.size);
            cv::Mat m = Pixels(f.keypoints);   // opencv: each axis to [-1, 1] on its own
            for (int i = 0; i < m.rows; ++i) {
                m.at<float>(i, 0) = m.at<float>(i, 0) / float(f.size.width) * 2.0f - 1.0f;
                m.at<float>(i, 1) = m.at<float>(i, 1) / float(f.size.height) * 2.0f - 1.0f;
            }
            return m;
        };
        const int N = a.descriptors.rows, M = b.descriptors.rows, D = a.descriptors.cols;
        const int k0[] = {1, N, 2}, k1[] = {1, M, 2}, d0[] = {1, N, D}, d1[] = {1, M, D};
        const cv::Mat ka = Norm(a), kb = Norm(b), da = a.descriptors.clone(), db = b.descriptors.clone();
        const std::vector<cv::Mat> res = net->Run({{"kpts0", ka.reshape(1, 3, k0)}, {"kpts1", kb.reshape(1, 3, k1)},
                                                   {"desc0", da.reshape(1, 3, d0)}, {"desc1", db.reshape(1, 3, d1)}},
                                                  {"matches0", "mscores0"});
        const int K = int(res[0].total() / 2);
        const cv::Mat mi = res[0].reshape(1, K), ms = res[1].reshape(1, std::max(K, 1));
        std::vector<std::vector<cv::DMatch>> per(static_cast<size_t>(N));   // knnMatch(k = 1), then match() keeps each first
        for (int i = 0; i < K; ++i) {
            const int q = mi.at<int>(i, 0), t = mi.at<int>(i, 1);
            if (q >= 0 && t >= 0 && q < N && t < M && ms.at<float>(i, 0) >= p_.min_score)
                per[size_t(q)].push_back(cv::DMatch(q, t, 1.0f - ms.at<float>(i, 0)));
        }
        for (const auto& v : per)
            if (!v.empty()) out.push_back(v.front());
        return out;
    }
#endif
    cv::LightGlueMatcher& lightglue = *nets_.Local().lightglue;
    if (p_.normalization == "upstream") {
        // Empty sizes: LightGlueMatcher passes the keypoints through unchanged.
        lightglue.setPairInfo(NormalizeUpstream(a.keypoints, a.size), NormalizeUpstream(b.keypoints, b.size),
                                cv::Size(), cv::Size());
    } else {
        lightglue.setPairInfo(Pixels(a.keypoints), Pixels(b.keypoints), a.size, b.size);
    }
    lightglue.match(a.descriptors, b.descriptors, out);   // setPairInfo is consumed by this call
    return out;
}

std::vector<dnn::ModelRecord> HardPairMatcher::Records() const {
    std::string engine = "cv_dnn_new";
#ifdef ARIADNE_HAVE_ORT
    if (const dnn::OrtNet* net = nets_.Local().ort_aliked.get()) engine = "onnxruntime_" + net->Provider();   // what ran
#endif
    return {{std::filesystem::path(p_.aliked_onnx).stem().string(), p_.aliked_onnx, engine},
            {std::filesystem::path(p_.lightglue_onnx).stem().string(), p_.lightglue_onnx, engine}};
}

std::string HardPairMatcher::Name() const {
    return std::filesystem::path(p_.aliked_onnx).stem().string() + "+" +
           std::filesystem::path(p_.lightglue_onnx).stem().string() + "/" +
           (p_.backend == "cv_dnn" ? std::string("cv_dnn_new") : p_.backend) + "/norm_" + p_.normalization;
}

}  // namespace ariadne::perception
