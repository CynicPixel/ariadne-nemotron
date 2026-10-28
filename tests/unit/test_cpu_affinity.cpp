#include "runtime/live/cpu_affinity.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <thread>
using namespace ariadne::runtime::live;

TEST(CpuAffinity, ParsesRangesAndSingles) {
    const cpu_set_t s = ParseCpuList("0-2,5");
    EXPECT_EQ(CPU_COUNT(&s), 4);
    EXPECT_TRUE(CPU_ISSET(0, &s) && CPU_ISSET(1, &s) && CPU_ISSET(2, &s) && CPU_ISSET(5, &s));
    EXPECT_FALSE(CPU_ISSET(3, &s));
    EXPECT_THROW(ParseCpuList("3-1"), std::invalid_argument);
    EXPECT_THROW(ParseCpuList(""), std::invalid_argument);
    EXPECT_THROW(ParseCpuList("x"), std::invalid_argument);
}

TEST(CpuAffinity, PinAppliesToLaterThreadsAndUnsetIsANoOp) {
    cpu_set_t before;
    ASSERT_EQ(sched_getaffinity(0, sizeof before, &before), 0);
    ::unsetenv("ARIADNE_TEST_CPUSET");
    PinFromEnv("ARIADNE_TEST_CPUSET", "test");   // unset: nothing changes
    cpu_set_t same;
    sched_getaffinity(0, sizeof same, &same);
    EXPECT_TRUE(CPU_EQUAL(&before, &same));
    int first = -1;
    for (int c = 0; c < CPU_SETSIZE && first < 0; ++c) if (CPU_ISSET(c, &before)) first = c;
    ASSERT_GE(first, 0);
    // Pin a helper thread (so the test runner's own mask is untouched) and check a thread it starts inherits it.
    std::thread([&] {
        ::setenv("ARIADNE_TEST_CPUSET", std::to_string(first).c_str(), 1);
        PinFromEnv("ARIADNE_TEST_CPUSET", "test");
        std::thread([&] {
            cpu_set_t now;
            sched_getaffinity(0, sizeof now, &now);
            EXPECT_EQ(CPU_COUNT(&now), 1);
            EXPECT_TRUE(CPU_ISSET(first, &now));
        }).join();
    }).join();
    ::unsetenv("ARIADNE_TEST_CPUSET");
}
