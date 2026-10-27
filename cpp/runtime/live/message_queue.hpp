#pragma once
#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ariadne::runtime::live {
struct StreamMessage {
    std::string topic, type;
    int64_t stamp_ns = -1;   // header stamp; -1 = untimed (no header: std_msgs/String)
    std::shared_ptr<const std::vector<uint8_t>> data;
    uint64_t seq = 0;        // release order (1-based), = log time in the recording
};

// Ordinary bounded FIFOs retain blocking semantics. Sensor queues instead use a
// byte-limited, nonblocking producer: a demultiplexer must remain able to deliver
// IMU while the estimator is waiting for the IMU bracket of an image.
class MessageQueue {
public:
    explicit MessageQueue(size_t capacity = 0) : cap_(capacity) {}
    void ConfigureSensor(size_t byte_limit, std::string left = {}, std::string right = {}, bool require_imu = false) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!q_.empty() || closed_ || !byte_limit) throw std::logic_error("configure an empty open sensor queue");
        if (left.empty() != right.empty() || (!left.empty() && left == right))
            throw std::invalid_argument("stereo queue needs distinct left/right topics");
        byte_limit_ = byte_limit;
        left_ = std::move(left); right_ = std::move(right);
        require_imu_ = require_imu;
        if (require_imu_ && left_.empty()) throw std::invalid_argument("IMU gate requires stereo topics");
    }
    // Advance only after the corresponding IMU was successfully transmitted.
    // This bounds camera lead without waiting for any other agent's camera.
    void AdvanceStereoWatermark(int64_t stamp) {
        { std::lock_guard<std::mutex> lk(mu_); imu_watermark_ = std::max(imu_watermark_, stamp); }
        cv_.notify_one();
    }
    // Installed before producers start. Callback must be short, nonthrowing and
    // must not re-enter this queue. Used to journal demo pair selection/drop decisions.
    void Observe(std::function<void(const char*, int64_t)> observer) {
        std::lock_guard<std::mutex> lk(mu_); observer_ = std::move(observer);
    }
    void Push(StreamMessage m) {
        std::unique_lock<std::mutex> lk(mu_);
        if (!byte_limit_ && cap_) space_.wait(lk, [&] { return q_.size() < cap_ || closed_; });
        if (closed_) return;
        const size_t n = Bytes(m);
        if (!left_.empty() && (m.topic == left_ || m.topic == right_)) {
            if (m.stamp_ns < 0) throw std::invalid_argument("stereo queue requires a timestamp");
            // Once a pair has been selected, late duplicates cannot resurrect it.
            if (m.stamp_ns <= selected_stamp_) { ++late_images_; Event("late_image", m.stamp_ns); return; }
            auto& pair = pairs_[m.stamp_ns];
            auto& slot = m.topic == left_ ? pair.left : pair.right;
            if (slot) bytes_ -= Bytes(*slot);
            bytes_ += n; slot = std::move(m);
            TrimPairs();
        } else {
            if (byte_limit_ && (n > byte_limit_ || bytes_ > byte_limit_ - n))
                throw std::runtime_error("sensor queue byte limit exceeded; refusing silent sensor loss");
            bytes_ += n; q_.push_back(std::move(m));
        }
        if (byte_limit_ && bytes_ > byte_limit_)
            throw std::runtime_error("sensor queue byte limit exceeded; refusing silent sensor loss");
        max_depth_ = std::max(max_depth_, Depth());
        max_bytes_ = std::max(max_bytes_, bytes_);
        lk.unlock(); cv_.notify_one();
    }
    std::optional<StreamMessage> Pop() {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&] { return !q_.empty() || partner_ || Complete() != pairs_.end() || closed_; });
        return PopReady(lk);
    }
    std::optional<StreamMessage> TryPop() {
        std::unique_lock<std::mutex> lk(mu_);
        if (q_.empty() && !partner_ && Complete() == pairs_.end() && !closed_) return std::nullopt;
        return PopReady(lk);
    }
    void Close() {
        { std::lock_guard<std::mutex> lk(mu_); closed_ = true; }
        cv_.notify_all(); space_.notify_all();
    }
    size_t max_depth() const { std::lock_guard<std::mutex> lk(mu_); return max_depth_; }
    size_t max_bytes() const { std::lock_guard<std::mutex> lk(mu_); return max_bytes_; }
    size_t dropped_pairs() const { std::lock_guard<std::mutex> lk(mu_); return dropped_pairs_; }
    size_t dropped_incomplete() const { std::lock_guard<std::mutex> lk(mu_); return dropped_incomplete_; }
    size_t late_images() const { std::lock_guard<std::mutex> lk(mu_); return late_images_; }
private:
    std::optional<StreamMessage> PopReady(std::unique_lock<std::mutex>& lk) {
        std::optional<StreamMessage> out;
        // An already selected pair is delivered together, even if newer images arrive.
        if (partner_) { out = std::move(partner_); partner_.reset(); }
        else if (!q_.empty()) { out = std::move(q_.front()); q_.pop_front(); }
        else if (auto it = Complete(); it != pairs_.end()) {
            selected_stamp_ = it->first;
            Event("selected_pair", selected_stamp_);
            out = std::move(it->second.left); partner_ = std::move(it->second.right);
            pairs_.erase(it);
            // Incomplete older timestamps can no longer be emitted in order.
            for (auto j = pairs_.begin(); j != pairs_.end() && j->first <= selected_stamp_; ) {
                Drop(j->second); j = pairs_.erase(j);
            }
        } else {
            for (auto& [stamp, pair] : pairs_) Drop(pair);
            pairs_.clear();
            return std::nullopt;
        }
        bytes_ -= Bytes(*out);
        lk.unlock(); space_.notify_one();
        return out;
    }
    struct Pair {
        std::optional<StreamMessage> left, right;
        bool complete() const { return left && right; }
    };
    static size_t Bytes(const StreamMessage& m) { return m.data ? m.data->size() : 0; }
    std::map<int64_t, Pair>::iterator Complete() {
        return std::find_if(pairs_.begin(), pairs_.end(), [this](const auto& p) {
            return p.second.complete() && (!require_imu_ || p.first <= imu_watermark_);
        });
    }
    size_t Depth() const {
        size_t n = q_.size() + bool(partner_);
        for (const auto& [stamp, p] : pairs_) n += bool(p.left) + bool(p.right);
        return n;
    }
    void Event(const char* kind, int64_t stamp) {
        if (observer_) observer_(kind, stamp);
    }
    void Drop(const Pair& p) {
        Event(p.complete() ? "dropped_pair" : "dropped_incomplete", p.left ? p.left->stamp_ns : p.right->stamp_ns);
        if (p.complete()) ++dropped_pairs_; else ++dropped_incomplete_;
        if (p.left) bytes_ -= Bytes(*p.left);
        if (p.right) bytes_ -= Bytes(*p.right);
    }
    void TrimPairs() {
        size_t complete = 0, incomplete = 0;
        for (const auto& [stamp, p] : pairs_) p.complete() ? ++complete : ++incomplete;
        for (auto it = pairs_.begin(); it != pairs_.end();) {
            auto& count = it->second.complete() ? complete : incomplete;
            if (count > 2) { Drop(it->second); --count; it = pairs_.erase(it); }
            else ++it;
        }
    }
    size_t cap_ = 0, byte_limit_ = 0, bytes_ = 0, max_bytes_ = 0, max_depth_ = 0;
    size_t dropped_pairs_ = 0, dropped_incomplete_ = 0, late_images_ = 0;
    int64_t selected_stamp_ = -1, imu_watermark_ = -1;
    bool require_imu_ = false;
    std::string left_, right_;
    mutable std::mutex mu_;
    std::condition_variable cv_, space_;
    std::deque<StreamMessage> q_;
    std::map<int64_t, Pair> pairs_;
    std::optional<StreamMessage> partner_;
    std::function<void(const char*, int64_t)> observer_;
    bool closed_ = false;
};
}
