// ---------------------------------------------------------------------------
// ariadne_collab_worker: one agent's DNN work for the live collaboration (design LIVE_COLLAB §1-§2, D174). Spawned by
// ariadne_live per agent with `collab: true`; never part of the estimator worker, whose OpenCV pool must stay
// unperturbed (D152a/D168).
//
//   ariadne_collab_worker --socket PATH --agent K --place-onnx ONNX --place-size WxH --aliked-onnx ONNX
//       --lightglue-onnx ONNX --threads N [--keep-s 120] [--n-features 1024]
//
// Listens on PATH, accepts two peers, each identified by its first frame (#hello role): the agent's estimator worker
// (#ckin, #crel, #cprog) and the parent (#freq). One processing thread handles every frame in arrival order
// (runtime/live/collab_worker_core.hpp), so EigenPlaces and ALIKED never overlap in this process's pool. `--threads`
// pins OpenCV's pool (cv::ALIKED's output depends on the thread count, LIVE_COLLAB §7): recorded on stdout's last line
// with the counters. Exits when both peers have closed.
// ---------------------------------------------------------------------------
#include "runtime/live/collab_worker_core.hpp"
#include "runtime/live/link_frame.hpp"

#include <opencv2/core.hpp>

#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

using namespace ariadne;
namespace live = ariadne::runtime::live;

int main(int argc, char** argv) {
    std::string socket_path, place_onnx, place_size = "640x360", aliked_onnx, lightglue_onnx, dnn_backend = "cv_dnn";
    int agent = 0, threads = 0;
    double keep_s = 120.0;
    uint32_t n_features = 1024;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", k.c_str()); return 2; }
        const std::string v = argv[++i];
        if (k == "--socket") socket_path = v;
        else if (k == "--agent") agent = std::stoi(v);
        else if (k == "--place-onnx") place_onnx = v;
        else if (k == "--place-size") place_size = v;
        else if (k == "--aliked-onnx") aliked_onnx = v;
        else if (k == "--lightglue-onnx") lightglue_onnx = v;
        else if (k == "--threads") threads = std::stoi(v);
        else if (k == "--keep-s") keep_s = std::stod(v);
        else if (k == "--n-features") n_features = uint32_t(std::stoul(v));
        else if (k == "--dnn-backend") dnn_backend = v;   // ALIKED: cv_dnn | ort_cuda | ort_cpu
        else { std::fprintf(stderr, "unknown argument %s\n", k.c_str()); return 2; }
    }
    if (socket_path.empty() || place_onnx.empty() || aliked_onnx.empty() || lightglue_onnx.empty() || threads <= 0) {
        std::fprintf(stderr, "usage: see the header of ariadne_collab_worker.cpp\n");
        return 2;
    }
    try {
        cv::setNumThreads(threads);
        perception::PlaceRecognizerParams pp;
        pp.onnx = place_onnx;
        if (std::sscanf(place_size.c_str(), "%dx%d", &pp.width, &pp.height) != 2)
            throw std::invalid_argument("--place-size WxH");
        const perception::PlaceRecognizer place(pp);
        perception::HardPairParams hp;
        hp.aliked_onnx = aliked_onnx;
        hp.lightglue_onnx = lightglue_onnx;
        hp.width = 1280, hp.height = 720;   // the D90 sim export
        hp.backend = dnn_backend;
        const perception::HardPairMatcher aliked(hp);
        for (const auto& r : aliked.Records())   // the backend that actually runs (log which ran)
            std::fprintf(stderr, "ariadne_collab_worker %d: %s via %s\n", agent, r.model.c_str(), r.engine.c_str());
        runtime::CollabFeatureExtractor x(place, aliked, keep_s, /*evict_slack_s=*/60.0);   // D179: memory only, never the answer

        // Accept both peers on one listening socket.
        const int s = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (s < 0) throw std::runtime_error("socket");
        ::unlink(socket_path.c_str());
        const sockaddr_un a = live::UnixAddress(socket_path);
        if (::bind(s, reinterpret_cast<const sockaddr*>(&a), sizeof a) != 0 || ::listen(s, 2) != 0)
            throw std::runtime_error("bind/listen " + socket_path);
        int parent_fd = -1;
        std::mutex mu;
        std::condition_variable cv;
        std::deque<live::LinkFrame> inbox;
        int open_peers = 2;
        std::mutex send_mu;
        live::CollabWorkerCore core(x, [&](const std::string& topic, const std::vector<uint8_t>& b) {
            std::lock_guard<std::mutex> lk(send_mu);
            if (parent_fd >= 0) live::WriteFrame(parent_fd, topic, b.data(), b.size());
        }, n_features);
        // The peers' reader threads, joined on EVERY exit: an exception (an accept/hello failure, or a DNN error in
        // Dispatch) unwinding past a joinable std::thread is std::terminate "without an active exception", which
        // hides the real error (collab60c's replay, 2026-10-24). Shutting the fds down unblocks ReadFrame.
        struct Readers {
            std::vector<std::thread> threads;
            std::vector<int> fds;
            ~Readers() {
                for (int fd : fds) ::shutdown(fd, SHUT_RDWR);
                for (auto& t : threads)
                    if (t.joinable()) t.join();
                for (int fd : fds) ::close(fd);
            }
        } readers;
        for (int i = 0; i < 2; ++i) {
            const int c = ::accept(s, nullptr, nullptr);
            if (c < 0) throw std::runtime_error("accept");
            readers.fds.push_back(c);
            live::LinkFrame hello;
            if (!live::ReadFrame(c, hello) || hello.topic != "#hello") throw std::runtime_error("peer sent no #hello");
            const live::CollabHello h = live::DecodeHello(hello.payload);
            if (h.agent != uint32_t(agent)) throw std::runtime_error("peer is for another agent");
            if (h.role == "parent") {
                std::lock_guard<std::mutex> lk(send_mu);
                parent_fd = c;
            } else if (h.role != "estimator") {
                throw std::runtime_error("unknown peer role " + h.role);
            }
            readers.threads.emplace_back([&, c] {
                live::LinkFrame f;
                try {
                    while (live::ReadFrame(c, f)) {
                        std::lock_guard<std::mutex> lk(mu);
                        inbox.push_back(std::move(f));
                        cv.notify_one();
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "ariadne_collab_worker %d: %s\n", agent, e.what());
                }
                std::lock_guard<std::mutex> lk(mu);
                --open_peers;
                cv.notify_one();
            });
        }
        ::close(s);
        uint64_t unknown = 0;
        for (;;) {
            live::LinkFrame f;
            {
                std::unique_lock<std::mutex> lk(mu);
                cv.wait(lk, [&] { return !inbox.empty() || open_peers == 0; });
                if (inbox.empty()) break;
                f = std::move(inbox.front());
                inbox.pop_front();
            }
            if (!core.Dispatch(f.topic, f.payload)) ++unknown;
        }
        for (auto& t : readers.threads) t.join();
        const auto& st = core.stats();
        std::printf("collab_worker agent %d: threads %d, keyframes %llu, releases %llu, requests %llu, gone %llu, "
                    "unknown %llu, evicted_early %llu, ms_place %.0f, ms_aliked %.0f\n",
                    agent, cv::getNumThreads(), (unsigned long long)st.keyframes_in, (unsigned long long)st.releases,
                    (unsigned long long)st.requests, (unsigned long long)st.replies_gone, (unsigned long long)unknown, (unsigned long long)x.stats().evicted_early,
                    x.stats().ms_place, x.stats().ms_aliked);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ariadne_collab_worker: %s\n", e.what());
        return 1;
    }
}
