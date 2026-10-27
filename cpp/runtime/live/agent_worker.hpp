#pragma once
// ---------------------------------------------------------------------------
// One persistent thread per agent that OWNS that agent's planner state (map, ESDF, frontiers, last plan).
// Every touch of planner_k - keyframe integration, the per-decision plan, the command route - is a job on
// worker_k, executed in post order. That gives:
//   - no lock on the map: one thread writes and reads it;
//   - determinism: agent k's integrations run in exactly the order they were posted, so a lockstep run that
//     posts its sorted per-decision batch integrates the same keyframes in the same order as the old inline loop;
//   - overlap: agents integrate and plan in parallel, and in demo mode a keyframe is integrated when it is
//     delivered rather than at the next decision (the decision then waits only for the residual backlog).
// Jobs are few (a keyframe every ~0.2-1 s, a plan per decision), so a mutex + condvar FIFO is the right tool;
// lock wait on the existing queues measured ~1 us.
// A job's exception is kept and rethrown by the next Wait/Run on the posting side; later jobs are skipped.
// ---------------------------------------------------------------------------
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace ariadne::runtime::live {

class AgentWorker {
public:
    AgentWorker() : th_([this] { Loop(); }) {}
    AgentWorker(const AgentWorker&) = delete;
    AgentWorker& operator=(const AgentWorker&) = delete;
    ~AgentWorker() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_all();
        th_.join();   // drains every posted job first (Loop exits only on an empty queue)
    }

    /// Enqueue `job`; returns at once. Its sequence number orders it after every job posted before.
    uint64_t Post(std::function<void()> job) {
        uint64_t seq;
        {
            std::lock_guard<std::mutex> lk(mu_);
            q_.push_back(std::move(job));
            seq = ++posted_;
        }
        cv_.notify_all();
        return seq;
    }

    /// Block until every job posted so far has run; rethrow the first job exception, if any.
    void Wait() {
        std::unique_lock<std::mutex> lk(mu_);
        const uint64_t target = posted_;
        done_cv_.wait(lk, [&] { return done_ >= target; });
        if (err_) std::rethrow_exception(err_);
    }

    /// Post `job` and wait for it (and everything before it).
    void Run(std::function<void()> job) {
        Post(std::move(job));
        Wait();
    }

    /// Jobs posted but not yet finished (a backlog gauge for the latency profile).
    uint64_t Pending() const {
        std::lock_guard<std::mutex> lk(mu_);
        return posted_ - done_;
    }

private:
    void Loop() {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&] { return stop_ || !q_.empty(); });
                if (q_.empty()) return;   // stop_ and drained
                job = std::move(q_.front());
                q_.pop_front();
            }
            bool skip;
            {
                std::lock_guard<std::mutex> lk(mu_);
                skip = bool(err_);
            }
            if (!skip) {
                try {
                    job();
                } catch (...) {
                    std::lock_guard<std::mutex> lk(mu_);
                    if (!err_) err_ = std::current_exception();
                }
            }
            {
                std::lock_guard<std::mutex> lk(mu_);
                ++done_;
            }
            done_cv_.notify_all();
        }
    }

    mutable std::mutex mu_;
    std::condition_variable cv_, done_cv_;
    std::deque<std::function<void()>> q_;
    uint64_t posted_ = 0, done_ = 0;
    std::exception_ptr err_;
    bool stop_ = false;
    std::thread th_;   // last: starts after every member above is constructed
};

}  // namespace ariadne::runtime::live
