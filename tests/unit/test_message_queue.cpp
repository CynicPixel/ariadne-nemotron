#include "runtime/live/message_queue.hpp"
#include "runtime/live/sensor_horizon.hpp"
#include <gtest/gtest.h>
#include <future>
#include <thread>
using namespace ariadne::runtime::live;
namespace {
StreamMessage M(std::string topic, int64_t stamp, size_t bytes = 1) {
    return {std::move(topic), "", stamp, std::make_shared<const std::vector<uint8_t>>(bytes), 0};
}
void Pair(MessageQueue& q, int64_t stamp) { q.Push(M("L", stamp)); q.Push(M("R", stamp)); }
}
TEST(SensorQueue, LateImuCanPassMoreThan64ImagesWithoutAConsumer) {
    MessageQueue images(64), imu(512);
    images.ConfigureSensor(4096); imu.ConfigureSensor(4096);
    // Old blocking Push cannot reach the IMU sample until a consumer drains images;
    // that consumer is precisely the estimator waiting for this IMU sample.
    auto reader = std::async(std::launch::async, [&] {
        for (int i = 0; i < 130; ++i) images.Push(M("L", i));
        imu.Push(M("imu", 130));
    });
    const auto status = reader.wait_for(std::chrono::seconds(1));
    // Always release an old/broken implementation before the future destructor joins.
    if (status != std::future_status::ready) { images.Close(); imu.Close(); }
    ASSERT_EQ(status, std::future_status::ready);
    reader.get();
    ASSERT_EQ(imu.Pop()->stamp_ns, 130);
}
TEST(SensorQueue, LatestRetainsTwoWholePairs) {
    MessageQueue q; q.ConfigureSensor(4096, "L", "R");
    for (int i = 0; i < 10; ++i) Pair(q, i);
    q.Close();
    for (int i : {8, 9}) {
        EXPECT_EQ(q.Pop()->stamp_ns, i); EXPECT_EQ(q.Pop()->stamp_ns, i);
    }
    EXPECT_FALSE(q.Pop()); EXPECT_EQ(q.dropped_pairs(), 8);
}
TEST(SensorQueue, InFlightPartnerSurvivesProducerOverload) {
    MessageQueue q; q.ConfigureSensor(4096, "L", "R");
    Pair(q, 1); EXPECT_EQ(q.Pop()->topic, "L");
    for (int i = 2; i < 20; ++i) Pair(q, i);
    auto partner = q.Pop(); ASSERT_TRUE(partner);
    EXPECT_EQ(partner->stamp_ns, 1); EXPECT_EQ(partner->topic, "R");
    EXPECT_EQ(q.Pop()->stamp_ns, 18);
}
TEST(SensorQueue, BoundedIncompleteAndNoLateResurrection) {
    MessageQueue q; q.ConfigureSensor(4096, "L", "R");
    for (int i = 1; i < 10; ++i) q.Push(M("L", i));
    EXPECT_EQ(q.dropped_incomplete(), 7);
    q.Push(M("R", 9)); EXPECT_EQ(q.Pop()->stamp_ns, 9); EXPECT_EQ(q.Pop()->stamp_ns, 9);
    Pair(q, 8); q.Close(); EXPECT_FALSE(q.Pop()); EXPECT_EQ(q.late_images(), 2);
}
TEST(SensorQueue, MixedInboxPreservesAllImuWhileDroppingImages) {
    MessageQueue q; q.ConfigureSensor(4096, "L", "R");
    for (int i = 1; i <= 100; ++i) { Pair(q, i); q.Push(M("imu", i)); }
    for (int i = 1; i <= 100; ++i) EXPECT_EQ(q.Pop()->stamp_ns, i);
    EXPECT_EQ(q.dropped_pairs(), 98);
    EXPECT_EQ(q.Pop()->stamp_ns, 99);
}
TEST(SensorQueue, LosslessOverflowIsExplicitAndCloseRejectsPush) {
    MessageQueue q; q.ConfigureSensor(4);
    q.Push(M("imu", 1, 4));
    EXPECT_THROW(q.Push(M("imu", 2)), std::runtime_error);
    EXPECT_EQ(q.Pop()->stamp_ns, 1);
    q.Close(); q.Push(M("imu", 3)); EXPECT_FALSE(q.Pop());
}

TEST(SensorQueue, FairSenderCanPollIncompleteStereoWithoutBlockingControl) {
    MessageQueue image, control;
    image.ConfigureSensor(4096, "L", "R"); control.ConfigureSensor(4096);
    image.Push(M("L", 1));
    EXPECT_FALSE(image.TryPop());
    control.Push(M("imu", 1)); EXPECT_EQ(control.TryPop()->topic, "imu");
    image.Push(M("R", 1));
    EXPECT_EQ(image.TryPop()->topic, "L"); EXPECT_EQ(image.TryPop()->topic, "R");
    EXPECT_FALSE(image.TryPop());
}

TEST(SensorQueue, StereoCannotOvertakeItsTransmittedImu) {
    MessageQueue q; q.ConfigureSensor(4096, "L", "R", true);
    Pair(q, 78550);
    q.AdvanceStereoWatermark(60670);
    EXPECT_FALSE(q.TryPop());  // representative interleaving: image precedes its IMU bracket
    q.AdvanceStereoWatermark(78549);
    EXPECT_FALSE(q.TryPop());
    q.AdvanceStereoWatermark(78550);
    ASSERT_TRUE(q.TryPop());
    auto right = q.TryPop(); ASSERT_TRUE(right); EXPECT_EQ(right->stamp_ns, 78550);
}
TEST(SensorQueue, ImuGateIsPerAgentAndRetainsBoundedFreshPairs) {
    MessageQueue a, b;
    a.ConfigureSensor(4096, "L", "R", true); b.ConfigureSensor(4096, "L", "R", true);
    for (int i = 0; i < 100; ++i) Pair(a, i);
    Pair(b, 99); b.AdvanceStereoWatermark(99);
    EXPECT_FALSE(a.TryPop());
    ASSERT_TRUE(b.TryPop());  // the slow IMU of A cannot hold B
    EXPECT_EQ(a.dropped_pairs(), 98);
    a.AdvanceStereoWatermark(99);
    EXPECT_EQ(a.TryPop()->stamp_ns, 98);
    EXPECT_EQ(a.TryPop()->stamp_ns, 98);
    EXPECT_EQ(a.TryPop()->stamp_ns, 99);
}
TEST(SensorHorizon, FastCameraCannotCloseLaggingImuOrAnotherAgent) {
    SensorHorizon h(2, 100);
    EXPECT_FALSE(h.Admit(0, SensorHorizon::Left, 101));
    EXPECT_FALSE(h.Admit(0, SensorHorizon::Right, 101));
    EXPECT_FALSE(h.Complete());  // the previous any-stream cutoff closed here
    EXPECT_TRUE(h.Admit(0, SensorHorizon::Imu, 79));
    EXPECT_TRUE(h.Admit(0, SensorHorizon::Imu, 101));
    EXPECT_FALSE(h.Admit(0, SensorHorizon::Imu, 102));
    EXPECT_FALSE(h.Complete());
    EXPECT_TRUE(h.Admit(1, SensorHorizon::Left, 100));
    EXPECT_TRUE(h.Admit(1, SensorHorizon::Right, 100));
    EXPECT_TRUE(h.Admit(1, SensorHorizon::Imu, 101));
    EXPECT_FALSE(h.Admit(1, SensorHorizon::Left, 101));
    EXPECT_FALSE(h.Complete());
    EXPECT_FALSE(h.Admit(1, SensorHorizon::Right, 101));
    EXPECT_TRUE(h.Complete());
}
TEST(SensorHorizon, LastImuBracketIsAdmittedBeforeCompletion) {
    SensorHorizon h(1, 100);
    EXPECT_FALSE(h.Admit(0, SensorHorizon::Left, 101));
    EXPECT_FALSE(h.Admit(0, SensorHorizon::Right, 101));
    EXPECT_FALSE(h.Complete());
    EXPECT_TRUE(h.Admit(0, SensorHorizon::Imu, 101));
    EXPECT_TRUE(h.Complete());
}
