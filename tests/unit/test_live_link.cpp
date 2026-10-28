// The live runtime's input side (design live-runtime.md D150-D152): runtime/live/{link_frame,live_source}.hpp.
//   - link frames round-trip over a socketpair, control frames included; a bad magic throws;
//   - PoseArray (the command) and String (mover state) CDR round-trip through the runtime's own codec;
//   - PeekStamp reads the header stamp without a full decode;
//   - Reorderer releases in (stamp, topic, arrival) order behind the watermark, and stops waiting for a silent topic
//     after the hold-back time;
//   - ConsumedRecorder: the MCAP replays in the CONSUMED order (log time = sequence) even where stamps tie, and the
//     replay's digest equals the live one;
//   - SimFrameAssembler lenient mode drops and counts an unpaired frame where strict mode throws.
#include "runtime/live/link_frame.hpp"
#include "runtime/live/live_source.hpp"
#include "runtime/replay/ros_cdr_writer.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>

#include <filesystem>
#include <thread>

using namespace ariadne::runtime;
namespace live = ariadne::runtime::live;

namespace {

std::vector<uint8_t> EncodeRgbImage(int64_t t_ns, int w, int h, uint8_t fill) {
    ros_cdr::CdrWriter c;
    c.Write<int32_t>(int32_t(t_ns / 1000000000));
    c.Write<uint32_t>(uint32_t(t_ns % 1000000000));
    c.WriteString("agent_0/left_optical");
    c.Write<uint32_t>(uint32_t(h));
    c.Write<uint32_t>(uint32_t(w));
    c.WriteString("rgb8");
    c.Write<uint8_t>(0);
    c.Write<uint32_t>(uint32_t(3 * w));
    c.Write<uint32_t>(uint32_t(3 * w * h));
    std::vector<uint8_t> b = c.bytes();
    b.insert(b.end(), size_t(3 * w * h), fill);
    return b;
}

live::StreamMessage Msg(const std::string& topic, int64_t t, uint8_t tag = 0) {
    live::StreamMessage m;
    m.topic = topic;
    m.type = "sensor_msgs/msg/Imu";
    m.stamp_ns = t;
    m.data = std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>{0, 1, 0, 0, tag});
    return m;
}

}  // namespace

TEST(LiveLink, FramesRoundTripOverASocket) {
    int sv[2];
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
    std::vector<uint8_t> big(3u << 20);
    for (size_t i = 0; i < big.size(); ++i) big[i] = uint8_t(i * 7);
    std::thread w([&] {
        EXPECT_TRUE(live::WriteFrame(sv[0], "/ariadne/agent_0/left/image_raw", big.data(), big.size()));
        const std::string js = "{\"forwarded\":[]}";
        EXPECT_TRUE(live::WriteFrame(sv[0], "#ready", reinterpret_cast<const uint8_t*>(js.data()), js.size()));
        EXPECT_TRUE(live::WriteFrame(sv[0], "/empty", nullptr, 0));
        ::shutdown(sv[0], SHUT_WR);
    });
    live::LinkFrame f;
    ASSERT_TRUE(live::ReadFrame(sv[1], f));
    EXPECT_EQ(f.topic, "/ariadne/agent_0/left/image_raw");
    EXPECT_EQ(f.payload, big);
    ASSERT_TRUE(live::ReadFrame(sv[1], f));
    EXPECT_EQ(f.topic, "#ready");
    ASSERT_TRUE(live::ReadFrame(sv[1], f));
    EXPECT_TRUE(f.payload.empty());
    EXPECT_FALSE(live::ReadFrame(sv[1], f));   // clean EOF between frames
    w.join();
    ::close(sv[0]);
    ::close(sv[1]);

    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
    const uint8_t junk[12] = {1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0};
    ASSERT_EQ(::send(sv[0], junk, sizeof junk, 0), ssize_t(sizeof junk));
    EXPECT_THROW(live::ReadFrame(sv[1], f), std::runtime_error);
    ::close(sv[0]);
    ::close(sv[1]);
}

TEST(LiveLink, CommandAndMoverStateCdrRoundTrip) {
    ros_cdr::PoseArray m;
    m.header.stamp = {12, 345000000};
    m.header.frame_id = "world";
    for (int i = 0; i < 3; ++i) {
        ros_cdr::Pose p;
        p.position = {1.0 + i, -2.5, 1.0};
        p.orientation = {0, 0, i == 2 ? 0.7071067811865476 : 0.0, i == 2 ? 0.7071067811865476 : 0.0};
        m.poses.push_back(p);
    }
    const auto b = ros_cdr::EncodePoseArray(m);
    const auto d = ros_cdr::DecodePoseArray(b.data(), b.size());
    EXPECT_EQ(d.header.stamp.ns(), 12345000000);
    EXPECT_EQ(d.header.frame_id, "world");
    ASSERT_EQ(d.poses.size(), 3u);
    EXPECT_EQ(d.poses[1].position.x, 2.0);
    EXPECT_EQ(d.poses[2].orientation.w, 0.7071067811865476);
    EXPECT_EQ(d.poses[0].orientation.w, 0.0);   // the zero quaternion ("no final yaw") survives
    const ros_cdr::PoseArray hold{m.header, {}};
    const auto hb = ros_cdr::EncodePoseArray(hold);
    EXPECT_TRUE(ros_cdr::DecodePoseArray(hb.data(), hb.size()).poses.empty());
    const auto s = ros_cdr::EncodeString("{\"accepted\":true}");
    EXPECT_EQ(ros_cdr::DecodeString(s.data(), s.size()), "{\"accepted\":true}");
    EXPECT_EQ(live::PeekStamp("geometry_msgs/msg/PoseArray", b), 12345000000);
    EXPECT_EQ(live::PeekStamp("std_msgs/msg/String", s), -1);   // no applied_ns: untimed
    const auto ans = ros_cdr::EncodeString("{\"cmd_stamp_ns\":5000000000,\"applied_ns\":5000000000,\"accepted\":true}");
    EXPECT_EQ(live::PeekStamp("std_msgs/msg/String", ans), 5000000001);
}

// D166: a timeout skip that lets newer stamps out first, then the straggler, is counted as an order break.
TEST(LiveLink, OrderBreaksAreCounted) {
    live::Reorderer ro({"/a0/imu", "/a1/imu"}, std::chrono::milliseconds(0));   // hold 0: /a1 times out at once
    std::vector<int64_t> out;
    auto rel = [&](live::StreamMessage m) { out.push_back(m.stamp_ns); };
    auto msg = [](const std::string& t, int64_t s) { live::StreamMessage m; m.topic = t; m.stamp_ns = s; return m; };
    ro.Push(msg("/a0/imu", 1000), rel);
    ro.Push(msg("/a0/imu", 2000), rel);          // released past silent /a1 (timeout skip)
    EXPECT_GE(ro.released_on_timeout(), 1);
    EXPECT_EQ(ro.released_out_of_order(), 0);
    ro.Push(msg("/a1/imu", 500), rel);           // the straggler: below what already went out
    ro.Flush(rel);
    EXPECT_EQ(ro.released_out_of_order(), 1);
    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[2], 500);
}

// D165: both agents' answers to one barrier, in EITHER arrival order, are released in the same (stamp, topic) order
// once the watermark passes them (two EGL lockstep runs had recorded them swapped).
TEST(LiveLink, AnswersReleasedInCanonicalOrder) {
    auto run = [](bool agent1_first) {
        live::Reorderer ro({"/a0/imu", "/a1/imu"}, std::chrono::hours(1));
        std::vector<std::string> out;
        auto rel = [&](live::StreamMessage m) { out.push_back(m.topic + "@" + std::to_string(m.stamp_ns)); };
        auto msg = [](const std::string& t, int64_t s) { live::StreamMessage m; m.topic = t; m.stamp_ns = s; return m; };
        ro.Push(msg("/a0/imu", 5000000000), rel);
        ro.Push(msg("/a1/imu", 5000000000), rel);   // watermark = 5 s: the world paused at the barrier
        if (agent1_first) ro.Push(msg("/a1/mover_state", 5000000001), rel), ro.Push(msg("/a0/mover_state", 5000000001), rel);
        else ro.Push(msg("/a0/mover_state", 5000000001), rel), ro.Push(msg("/a1/mover_state", 5000000001), rel);
        EXPECT_EQ(out.size(), 2u);                    // the answers are HELD while paused
        ro.Push(msg("/a0/imu", 5005000000), rel);
        ro.Push(msg("/a1/imu", 5005000000), rel);   // resumed: the watermark passes them
        return out;
    };
    const auto a = run(false), b = run(true);
    EXPECT_EQ(a, b);
    ASSERT_GE(a.size(), 4u);
    EXPECT_EQ(a[2], "/a0/mover_state@5000000001");
    EXPECT_EQ(a[3], "/a1/mover_state@5000000001");
}

TEST(LiveLink, ReordererReleasesBehindTheWatermark) {
    live::Reorderer ro({"/a", "/b"}, std::chrono::milliseconds(200));
    std::vector<std::string> out;
    auto rel = [&](live::StreamMessage m) { out.push_back(m.topic + "@" + std::to_string(m.stamp_ns)); };
    ro.Push(Msg("/a", 10), rel);
    ro.Push(Msg("/a", 20), rel);
    EXPECT_TRUE(out.empty());                     // /b has not spoken: nothing is safe yet
    ro.Push(Msg("/b", 15), rel);
    EXPECT_EQ(out, (std::vector<std::string>{"/a@10", "/b@15"}));
    ro.Push(Msg("/b", 20), rel);                  // tie at 20: topic order, then arrival
    EXPECT_EQ(out.back(), "/b@20");
    EXPECT_EQ(out[out.size() - 2], "/a@20");
    // /a falls silent: after the hold-back /b's messages flow without it.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ro.Push(Msg("/b", 30), rel);
    EXPECT_EQ(out.back(), "/b@30");
    EXPECT_GE(ro.released_on_timeout(), 1);
}

TEST(LiveLink, RecordingReplaysInConsumedOrderWithTies) {
    const std::string path = (std::filesystem::temp_directory_path() / "ariadne_live_rec_test.mcap").string();
    const std::vector<std::pair<std::string, int64_t>> seq{{"/r", 50}, {"/l", 50}, {"/i", 45}, {"/l", 100}, {"/r", 100}};
    uint64_t digest_live;
    {
        live::ConsumedRecorder rec;
        rec.Open(path);
        uint64_t n = 0;
        for (const auto& [t, s] : seq) {
            auto m = Msg(t, s, uint8_t(n));
            m.seq = ++n;
            rec.Write(m);
        }
        digest_live = rec.digest();
        rec.Close();
    }
    RosbagReader bag;
    bag.Open(path);
    std::vector<std::string> got;
    live::ConsumedRecorder replay;   // not opened: digest only
    bag.Replay([&](const RosMessage& m) {
        got.push_back(std::string(m.topic));
        live::StreamMessage s;
        s.topic = std::string(m.topic);
        s.data = std::make_shared<std::vector<uint8_t>>(m.data, m.data + m.size);
        s.stamp_ns = int64_t(m.publish_time_ns);
        replay.Absorb(s);
    });
    EXPECT_EQ(got, (std::vector<std::string>{"/r", "/l", "/i", "/l", "/r"}));   // consumed order, not stamp order
    EXPECT_EQ(replay.digest(), digest_live);
    std::filesystem::remove(path);
}

TEST(LiveLink, AssemblerLenientDropsWhereStrictThrows) {
    ariadne::perception::LensModel lens;
    lens.width = 8;
    lens.height = 6;
    lens.fx = lens.fy = lens.render_fx = lens.render_fy = 10;
    lens.cx = lens.render_cx = 3.5;
    lens.cy = lens.render_cy = 2.5;
    const auto map = ariadne::perception::BuildLensMap(lens);
    const std::string L = "/l", R = "/r";
    auto img = [](int64_t t) { return EncodeRgbImage(t, 8, 6, 100); };
    for (const bool strict : {true, false}) {
        SimFrameAssembler a(&map, 0, L, R, strict);
        std::vector<int64_t> emitted;
        auto on = [&](const StereoFrame& f) { emitted.push_back(f.t_ns); };
        const auto l1 = img(50), r2 = img(100), l2 = img(100);
        a.Feed(L, l1.data(), l1.size(), on);   // its right never arrives
        a.Feed(R, r2.data(), r2.size(), on);
        if (strict) {
            EXPECT_THROW(a.Feed(L, l2.data(), l2.size(), on), std::runtime_error);
        } else {
            a.Feed(L, l2.data(), l2.size(), on);
            EXPECT_EQ(emitted, std::vector<int64_t>{100});
            EXPECT_EQ(a.unpaired(), 1u);
            const auto late = img(50);   // the missing right, too late: dropped and counted
            a.Feed(R, late.data(), late.size(), on);
            EXPECT_EQ(a.unpaired(), 2u);
        }
    }
}

// D229: the lane ingest pipeline's guard. Ingest on a real second thread through a depth-1 in-order slot, Apply on
// this thread, against the single-threaded Feed on an identical message sequence: the emitted frames must be byte
// identical (gray + left RGB), ids and stamps equal, and the unpaired accounting the same. The sequence exercises
// pairs, an unpaired left, a late right (dropped) and varying pixel content.
TEST(LiveLink, PipelinedIngestMatchesFeedExactly) {
    ariadne::perception::LensModel lens;
    lens.width = 8;
    lens.height = 6;
    lens.fx = lens.fy = 11;   // output focal > render: barrel margin, the map actually resamples
    lens.render_fx = lens.render_fy = 10;
    lens.cx = lens.render_cx = 3.5;
    lens.cy = lens.render_cy = 2.5;
    lens.k1 = -0.05;
    const auto map = ariadne::perception::BuildLensMap(lens);
    const std::string L = "/l", R = "/r";
    // (topic, t_ns, fill): two clean pairs, one unpaired left (60), one late right (60) after 100 emitted.
    const std::vector<std::tuple<const std::string*, int64_t, uint8_t>> seq = {
        {&L, 50, 10}, {&R, 50, 20}, {&L, 60, 30}, {&L, 100, 40}, {&R, 100, 50}, {&R, 60, 60}, {&L, 150, 70}, {&R, 150, 80},
    };
    auto run = [&](bool pipelined) {
        SimFrameAssembler a(&map, 0, L, R, /*strict=*/false);
        std::vector<ariadne::runtime::StereoFrame> out;
        auto on = [&](const ariadne::runtime::StereoFrame& f) {
            ariadne::runtime::StereoFrame c = f;
            c.left = f.left.clone();
            c.right = f.right.clone();
            c.left_rgb = f.left_rgb.clone();
            out.push_back(c);
        };
        if (!pipelined) {
            for (const auto& [topic, t, fill] : seq) {
                const auto b = EncodeRgbImage(t, 8, 6, fill);
                a.Feed(*topic, b.data(), b.size(), on);
            }
        } else {
            std::mutex mu;
            std::condition_variable cv;
            std::optional<SimFrameAssembler::Ingested> slot;
            bool eof = false;
            std::thread ingest([&] {
                for (const auto& [topic, t, fill] : seq) {
                    const auto b = EncodeRgbImage(t, 8, 6, fill);
                    auto in = a.Ingest(*topic, b.data(), b.size());
                    std::unique_lock<std::mutex> lk(mu);
                    cv.wait(lk, [&] { return !slot.has_value(); });
                    slot.emplace(std::move(in));
                    cv.notify_all();
                }
                std::lock_guard<std::mutex> lk(mu);
                eof = true;
                cv.notify_all();
            });
            for (;;) {
                std::optional<SimFrameAssembler::Ingested> in;
                {
                    std::unique_lock<std::mutex> lk(mu);
                    cv.wait(lk, [&] { return slot.has_value() || eof; });
                    if (!slot.has_value()) break;
                    in.emplace(std::move(*slot));
                    slot.reset();
                }
                cv.notify_all();
                a.Apply(std::move(*in), on);
            }
            ingest.join();
        }
        return std::make_tuple(std::move(out), a.unpaired(), a.pending(), a.emitted());
    };
    const auto [f0, u0, p0, e0] = run(false);
    const auto [f1, u1, p1, e1] = run(true);
    EXPECT_EQ(u0, u1);
    EXPECT_EQ(p0, p1);
    EXPECT_EQ(e0, e1);
    ASSERT_EQ(f0.size(), f1.size());
    ASSERT_EQ(f0.size(), 3u);   // 50, 100, 150
    for (size_t i = 0; i < f0.size(); ++i) {
        EXPECT_EQ(f0[i].t_ns, f1[i].t_ns);
        EXPECT_EQ(f0[i].id, f1[i].id);
        auto same = [](const cv::Mat& a, const cv::Mat& b) {
            ASSERT_EQ(a.size(), b.size());
            ASSERT_EQ(a.type(), b.type());
            ASSERT_TRUE(a.isContinuous() && b.isContinuous());
            EXPECT_EQ(0, std::memcmp(a.data, b.data, a.total() * a.elemSize()));
        };
        same(f0[i].left, f1[i].left);
        same(f0[i].right, f1[i].right);
        same(f0[i].left_rgb, f1[i].left_rgb);
    }
}

TEST(LiveLink, DemoArrivalOrderDoesNotWaitForAnotherCameraOrReorderImu) {
    live::Reorderer ro({"/a/imu", "/a/image", "/b/image"}, std::chrono::hours(1), true);
    std::vector<std::pair<std::string, int64_t>> seen;
    const auto accept = [&](live::StreamMessage m) { seen.emplace_back(m.topic, m.stamp_ns); };
    ro.Push(Msg("/a/imu", 100), accept);
    ASSERT_EQ(seen.size(), 1u);  // no camera has published
    ro.Push(Msg("/a/imu", 110), accept);
    ro.Push(Msg("/a/image", 90), accept);  // delayed image cannot delay future IMU
    ro.Push(Msg("/a/imu", 120), accept);
    ASSERT_EQ(seen.size(), 4u);
    EXPECT_EQ(seen[1].second, 110);
    EXPECT_EQ(seen[2].second, 90);
    EXPECT_EQ(seen[3].second, 120);
    ro.Flush(accept);
    EXPECT_EQ(seen.size(), 4u);  // no hidden backlog
    EXPECT_EQ(ro.released_on_timeout(), 0);
}
