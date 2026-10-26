#pragma once
// ---------------------------------------------------------------------------
// D225: the lockstep barrier's gate predicates over N lanes, factored out of ariadne_live so a unit test can
// hold the line. The pre-D225 inline checks were hardcoded to lanes 0 and 1: agent 2's estimator was never
// waited on and its keyframes slid past barrier cuts under load (the ma200 replay-to-replay trace divergence).
// A race reproduction cannot trip that deterministically; a predicate test does, on every run (M, 2026-10-24).
// Works over any range whose elements expose `done` and `progress` (AgentLane, or the test's LaneGate).
// ---------------------------------------------------------------------------
#include <cstdint>
#include <limits>

namespace ariadne::runtime::live {

struct LaneGate {
    bool done = false;
    int64_t progress = -1;   // stamp of the last frame the estimator finished
};

/// Every lane finished or past T: the barrier may run. False for an empty fleet (nothing can have reached).
template <typename Lanes>
inline bool AllReached(const Lanes& lanes, int64_t T) {
    bool any = false;
    for (const auto& l : lanes) {
        if (!(l.done || l.progress >= T)) return false;
        any = true;
    }
    return any;
}

/// Demo scheduling advances with any healthy agent. Done lanes cannot repeatedly
/// trigger horizons after their final frame. Evidence mode still uses AllReached.
template <typename Lanes>
inline bool AnyReached(const Lanes& lanes, int64_t T) {
    for (const auto& l : lanes) if (l.progress >= T) return true;
    return false;
}

/// Collaborative maps belong to the mission thread. The central demo integrator
/// must not consume their keyframes into the unrelated central planner.
inline bool IntegrateAtDecision(bool demo, bool collaborative) { return !demo || collaborative; }

/// Every lane finished BEFORE T: the mission ended early, nothing more will arrive.
template <typename Lanes>
inline bool AllDoneShort(const Lanes& lanes, int64_t T) {
    bool any = false;
    for (const auto& l : lanes) {
        if (!l.done || l.progress >= T) return false;
        any = true;
    }
    return any;
}

/// The pacing floor (D166): the slowest unfinished lane's progress; INT64_MAX when every lane finished.
template <typename Lanes>
inline int64_t MinProgress(const Lanes& lanes) {
    int64_t m = std::numeric_limits<int64_t>::max();
    for (const auto& l : lanes)
        if (!l.done && l.progress < m) m = l.progress;
    return m;
}

}  // namespace ariadne::runtime::live
