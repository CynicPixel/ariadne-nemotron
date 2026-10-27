#pragma once
// ---------------------------------------------------------------------------
// Live tuning knob: pin this process to the CPU list in an environment variable ("0-3,6"), Linux only. Call it first
// thing in a process, before any thread exists: every thread created later (OpenCV's pool, the agent workers)
// inherits the mask. Unset or empty = no change. Logged to stderr so the run log records what was applied.
// bd3d13 (8 vCPU): the runtime's main process took 102k involuntary context switches/s at ~8.7 busy threads while
// each estimator got 0.38 of a core; pinning and pool caps (live.sh ARIADNE_CPUSET_* / *_CV_THREADS) let a run
// reserve cores for the estimators and the decision path. A run without them behaves exactly as before.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ariadne::runtime::live {

/// Parse "0-3,6" into a cpu_set_t; throws on a malformed list.
inline cpu_set_t ParseCpuList(const std::string& list) {
    cpu_set_t set;
    CPU_ZERO(&set);
    std::stringstream ss(list);
    std::string part;
    while (std::getline(ss, part, ',')) {
        if (part.empty()) continue;
        const size_t dash = part.find('-');
        const int lo = std::stoi(part.substr(0, dash)), hi = dash == std::string::npos ? lo : std::stoi(part.substr(dash + 1));
        if (lo < 0 || hi < lo || hi >= CPU_SETSIZE) throw std::invalid_argument("cpu list '" + list + "'");
        for (int c = lo; c <= hi; ++c) CPU_SET(c, &set);
    }
    if (CPU_COUNT(&set) == 0) throw std::invalid_argument("cpu list '" + list + "' is empty");
    return set;
}

/// Pin the calling process (its current thread and every thread it creates later) to $var, if set.
inline void PinFromEnv(const char* var, const char* who) {
    const char* v = std::getenv(var);
    if (!v || !*v) return;
    const cpu_set_t set = ParseCpuList(v);
    if (sched_setaffinity(0, sizeof set, &set) != 0) throw std::runtime_error(std::string("sched_setaffinity ") + var);
    std::fprintf(stderr, "[affinity] %s pinned to %s (%s)\n", who, v, var);
}

}  // namespace ariadne::runtime::live
