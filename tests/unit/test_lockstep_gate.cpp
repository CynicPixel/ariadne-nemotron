// D225: the lockstep barrier gate predicates (runtime/live/lockstep_gate.hpp). The pre-D225 barrier checked
// lanes 0 and 1 only; this test is the deterministic guard M asked for - the old shape fails these assertions
// on every run, no timing involved (a race reproduction could not trip it: replay's answer wait hard-syncs all
// lanes, so the divergence only showed live on the instance, ma200).
#include "runtime/live/lockstep_gate.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace live = ariadne::runtime::live;
using live::LaneGate;

namespace {

// The pre-D225 predicate, verbatim in shape: lanes 0 and 1 only. Kept here as the documented trip target.
bool PreD225Reached(const std::vector<LaneGate>& l, int64_t T) {
    return (l[0].done || l[0].progress >= T) && (l[1].done || l[1].progress >= T);
}

std::vector<LaneGate> Lanes(std::initializer_list<int64_t> progress, std::initializer_list<int> done = {}) {
    std::vector<LaneGate> v;
    for (int64_t p : progress) v.push_back({false, p});
    for (int k : done) v[size_t(k)].done = true;
    return v;
}

}  // namespace

TEST(LockstepGate, WaitsOnEveryLane) {
    const int64_t T = 5000;
    for (int n : {3, 4}) {
        for (int lag = 0; lag < n; ++lag) {   // ANY lagging lane holds the barrier, not just 0 or 1
            std::vector<LaneGate> l(size_t(n), {false, T});
            l[size_t(lag)].progress = T - 1;
            EXPECT_FALSE(live::AllReached(l, T)) << "n=" << n << " lagging lane " << lag;
            l[size_t(lag)].done = true;   // a finished lane is exempt
            EXPECT_TRUE(live::AllReached(l, T));
        }
        EXPECT_TRUE(live::AllReached(std::vector<LaneGate>(size_t(n), {false, T}), T));
    }
    EXPECT_FALSE(live::AllReached(std::vector<LaneGate>{}, T)) << "an empty fleet never 'reaches'";
}

TEST(LockstepGate, PreD225ShapeTripsHere) {
    // The documented trip: with lane 2 lagging, the old predicate says GO, the fixed one says WAIT.
    const int64_t T = 5000;
    const auto l = Lanes({T, T, T - 1});
    EXPECT_TRUE(PreD225Reached(l, T)) << "the old shape ignores lane 2";
    EXPECT_FALSE(live::AllReached(l, T)) << "the fixed gate must not";
}

TEST(LockstepGate, AllDoneShortNeedsEveryLaneDoneBeforeT) {
    const int64_t T = 5000;
    EXPECT_TRUE(live::AllDoneShort(Lanes({T - 2, T - 3, T - 1}, {0, 1, 2}), T));
    EXPECT_FALSE(live::AllDoneShort(Lanes({T - 2, T - 3, T - 1}, {0, 1}), T)) << "lane 2 still running";
    EXPECT_FALSE(live::AllDoneShort(Lanes({T - 2, T - 3, T + 1}, {0, 1, 2}), T)) << "lane 2 passed T";
    EXPECT_FALSE(live::AllDoneShort(std::vector<LaneGate>{}, T));
}

TEST(LockstepGate, MinProgressIsTheSlowestUnfinishedLane) {
    EXPECT_EQ(live::MinProgress(Lanes({100, 40, 70})), 40);
    EXPECT_EQ(live::MinProgress(Lanes({100, 40, 70}, {1})), 70) << "a done lane never paces";
    EXPECT_EQ(live::MinProgress(Lanes({100, 40, 70}, {0, 1, 2})), std::numeric_limits<int64_t>::max());
}

TEST(LockstepGate, DemoProgressDoesNotWaitForStalledPeer) {
    const auto lanes = Lanes({100, 1, 50});
    EXPECT_TRUE(live::AnyReached(lanes, 100));
    EXPECT_FALSE(live::AllReached(lanes, 100));
    EXPECT_FALSE(live::AnyReached(lanes, 101));
    EXPECT_FALSE(live::AnyReached(std::vector<LaneGate>{}, 100));
}
TEST(LockstepGate, CollaborativeDemoIntegratesItsOwnMaps) {
    EXPECT_TRUE(live::IntegrateAtDecision(true, true));
    EXPECT_TRUE(live::IntegrateAtDecision(false, true));
    EXPECT_TRUE(live::IntegrateAtDecision(false, false));
    EXPECT_FALSE(live::IntegrateAtDecision(true, false));
}
