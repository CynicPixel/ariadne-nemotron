// ---------------------------------------------------------------------------
// ariadne_live_canonical (P8 D214): the canonical bag of a LIVE recording, so SimSource (and with it pass A of the
// 3.2 collab tools and ariadne_sim_vo --spool) reads a live run exactly like a sim_record recording.
//   In:  --run DIR (the live run's inner dir, holds inputs.mcap)  --scene DIR  --out RECDIR
//   Out: RECDIR/canonical/canonical_0.mcap - every agent's GT poses copied verbatim from inputs.mcap, plus one
//        /tf_static with each agent's left/right optical extrinsics from the scene rig (what the sim publishes).
// The sensors bag + recording.json are written by infra/scripts/live_to_rec.sh around this tool.
// ---------------------------------------------------------------------------
#include "runtime/live/live_source.hpp"
#include "runtime/replay/ros_cdr_writer.hpp"
#include "runtime/replay/rosbag_reader.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

namespace fs = std::filesystem;
using namespace ariadne;
namespace live = ariadne::runtime::live;

int main(int argc, char** argv) {
    try {
        std::string run, scene, out;
        for (int i = 1; i + 1 < argc; i += 2) {
            const std::string k = argv[i], v = argv[i + 1];
            if (k == "--run") run = v;
            else if (k == "--scene") scene = v;
            else if (k == "--out") out = v;
            else throw std::invalid_argument("unknown argument " + k);
        }
        if (run.empty() || scene.empty() || out.empty())
            throw std::invalid_argument("usage: ariadne_live_canonical --run DIR --scene DIR --out RECDIR");
        const YAML::Node m = YAML::LoadFile(scene + "/mission_live.json");
        fs::create_directories(fs::path(out) / "canonical");

        live::ConsumedRecorder rec;
        rec.Open((fs::path(out) / "canonical" / "canonical_0.mcap").string());

        // One /tf_static with every agent's optical extrinsics, stamped 0 (SimSource reads values only).
        std::vector<runtime::ros_cdr::TransformStamped> tfs;
        std::set<std::string> gt_topics;
        std::map<std::string, long> counts;
        for (const auto& ag : m["agents"]) {
            const int k = ag["agent"].as<int>();
            gt_topics.insert(ag["topics"]["gt_pose"].as<std::string>());
            live::MessageQueue q{1};
            live::MessageQueue imu_q{1};   // this tool reads calibration only; the IMU queue is intentionally unused
            live::GtStore g;
            std::mutex mu;
            live::LiveSimSource src(scene, k, q, imu_q, g, mu, "live");
            const auto calib = src.Calibration();
            const core::SE3 T_B_left = src.T_B_C0();
            const core::SE3 T_B_right = T_B_left * calib.T_right_left.inverse();
            const std::string base = ag["frame_id"].as<std::string>();   // "agent_k/base_link"
            const std::string stem = base.substr(0, base.size() - 9);    // "agent_k/"
            for (const auto& [child, T] : {std::pair<std::string, core::SE3>{stem + "left_optical", T_B_left},
                                           {stem + "right_optical", T_B_right}}) {
                runtime::ros_cdr::TransformStamped t;
                t.header.frame_id = base;
                t.child_frame_id = child;
                const Eigen::Quaterniond qr(T.R);
                t.rotation = {qr.x(), qr.y(), qr.z(), qr.w()};
                t.translation = {T.t.x(), T.t.y(), T.t.z()};
                tfs.push_back(t);
            }
        }
        {
            live::StreamMessage s;
            s.topic = "/tf_static";
            s.type = "tf2_msgs/msg/TFMessage";
            s.stamp_ns = 0;
            s.data = std::make_shared<const std::vector<uint8_t>>(runtime::ros_cdr::EncodeTfMessage(tfs));
            s.seq = 1;
            rec.Write(s);
        }
        uint64_t seq = 1, gt = 0;
        runtime::RosbagReader bag;
        bag.Open(run + "/inputs.mcap");
        bag.Replay([&](const runtime::RosMessage& msg) {
            const std::string topic(msg.topic);
            ++counts[topic];
            if (!gt_topics.count(topic)) return;
            live::StreamMessage s;
            s.topic = topic;
            s.type = "geometry_msgs/msg/PoseStamped";
            s.stamp_ns = runtime::ros_cdr::DecodePoseStamped(msg.data, msg.size).header.stamp.ns();
            s.data = std::make_shared<const std::vector<uint8_t>>(msg.data, msg.data + msg.size);
            s.seq = ++seq;
            rec.Write(s);
            ++gt;
        }, {});
        rec.Close();
        std::printf("canonical_0.mcap: %llu gt poses, %zu static transforms\n",
                    static_cast<unsigned long long>(gt), tfs.size());
        for (const auto& [t, n] : counts) std::printf("count %s %ld\n", t.c_str(), n);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ariadne_live_canonical: %s\n", e.what());
        return 1;
    }
}
