#include "runtime/live/agent_worker.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <future>
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

TEST(AgentWorker, UrgentJobRunsAfterTheJobInProgressAheadOfTheBacklog) {
    AgentWorker w;
    std::vector<int> seen;   // worker thread only
    std::promise<void> started, release;
    std::shared_future<void> go = release.get_future().share();
    w.Post([&] { started.set_value(); go.wait(); seen.push_back(0); });   // the integration in progress
    started.get_future().wait();
    for (int i = 1; i <= 5; ++i) w.Post([&, i] { seen.push_back(i); });   // the backlog
    EXPECT_EQ(w.Pending(), 6u);
    std::future<void> plan = w.PostUrgent([&] { seen.push_back(-1); });
    std::future<void> plan2 = w.PostUrgent([&] { seen.push_back(-2); });
    release.set_value();
    plan.get();
    plan2.get();
    w.Wait();
    EXPECT_EQ(seen, (std::vector<int>{0, -1, -2, 1, 2, 3, 4, 5}));
    EXPECT_EQ(w.Pending(), 0u);
}

TEST(AgentWorker, UrgentJobCarriesItsOwnOrAnEarlierError) {
    AgentWorker w;
    std::future<void> bad = w.PostUrgent([] { throw std::runtime_error("plan failed"); });
    EXPECT_THROW(bad.get(), std::runtime_error);
    std::future<void> later = w.PostUrgent([] {});
    EXPECT_THROW(later.get(), std::runtime_error);   // skipped: the worker is poisoned, as for FIFO jobs
    EXPECT_THROW(w.Wait(), std::runtime_error);
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
