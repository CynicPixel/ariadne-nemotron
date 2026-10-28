#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace ariadne::runtime::live {
// Arrival-ordered demo input cannot infer fleet EOF from one fast topic. Each
// camera must cross the horizon, and each IMU supplies its first sample beyond
// it so that a frame exactly at the horizon still has an inertial bracket.
class SensorHorizon {
public:
    enum Stream { Left = 0, Right = 1, Imu = 2 };
    SensorHorizon(size_t agents, int64_t end_ns) : crossed_(agents), end_(end_ns) {
        if (!agents || end_ns < 0) throw std::invalid_argument("invalid sensor horizon");
    }
    bool Admit(size_t agent, Stream stream, int64_t stamp) {
        auto& crossed = crossed_.at(agent).at(static_cast<size_t>(stream));
        if (stamp <= end_) return true;
        const bool first = !crossed;
        crossed = true;
        return stream == Imu && first;
    }
    bool Complete() const {
        return std::all_of(crossed_.begin(), crossed_.end(), [](const auto& a) {
            return a[Left] && a[Right] && a[Imu];
        });
    }
private:
    std::vector<std::array<bool, 3>> crossed_;
    int64_t end_;
};
}
