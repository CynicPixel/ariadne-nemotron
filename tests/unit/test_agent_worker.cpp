#include "runtime/live/agent_worker.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace ariadne::runtime::live;

TEST(AgentWorker, RunsJobsInPostOrderOnOneThread) {
    AgentWorker w;
    std::vector<int> seen;   // touched only by the worker thread: no lock, as for planner_k
    std::thread::id tid;
    for (int i = 0; i < 1000; ++i)
        w.Post([&, i] {
            if (i == 0) tid = std::this_thread::get_id();
            EXPECT_EQ(std::this_thread::get_id(), tid);
            seen.push_back(i);
        });
    w.Wait();
    ASSERT_EQ(seen.size(), 1000u);
    for (int i = 0; i < 1000; ++i) EXPECT_EQ(seen[size_t(i)], i);
    EXPECT_NE(tid, std::this_thread::get_id());
    EXPECT_EQ(w.Pending(), 0u);
}

TEST(AgentWorker, WaitCoversJobsPostedFromAnotherThread) {
    AgentWorker w;
    std::atomic<int> n{0};
    std::thread producer([&] { for (int i = 0; i < 200; ++i) w.Post([&] { ++n; }); });
    producer.join();
    w.Wait();
    EXPECT_EQ(n.load(), 200);
}

TEST(AgentWorker, RethrowsTheFirstJobErrorAndSkipsLaterJobs) {
    AgentWorker w;
    std::atomic<int> after{0};
    w.Post([] { throw std::runtime_error("integrate failed"); });
    w.Post([&] { ++after; });
    EXPECT_THROW(w.Wait(), std::runtime_error);
    EXPECT_EQ(after.load(), 0);
}

TEST(AgentWorker, DestructorDrainsPostedJobs) {
    std::atomic<int> n{0};
    {
        AgentWorker w;
        for (int i = 0; i < 50; ++i) w.Post([&] { std::this_thread::yield(); ++n; });
    }
    EXPECT_EQ(n.load(), 50);
}

TEST(AgentWorker, WorkersRunInParallel) {
    AgentWorker a, b;
    std::atomic<int> in{0};
    std::atomic<bool> both{false};
    auto job = [&] {
        ++in;
        for (int i = 0; i < 2000 && !both; ++i) {
            if (in.load() == 2) both = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    a.Post(job), b.Post(job);
    a.Wait(), b.Wait();
    EXPECT_TRUE(both.load());
}
