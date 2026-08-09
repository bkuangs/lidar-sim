#pragma once
#include "simulation.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * HEADLESS HARNESS
 *
 * Runs the simulation with no viewer and no real-time pacing, aggregating
 * per-frame metrics. This is the shared unlock for both benchmark layers:
 * deterministic, fast, and free of any Open3D dependency.
 */
struct HeadlessConfig
{
    unsigned int seed = 42;
    int frames = 900;
    double dt = 1.0 / 30.0;
    QueryMode query_mode = QueryMode::BruteForce;
    bool verify = true;      // correctness-gate accelerated modes against brute
    int verify_interval = 30; // frames between correctness checks
};

struct HeadlessResult
{
    QueryMode query_mode = QueryMode::BruteForce;
    int frames = 0;
    double distance = 0.0;
    int collisions = 0;
    int verification_mismatches = 0;
    bool verified = true;

    double scan_mean_ms = 0.0;
    double scan_p50_ms = 0.0;
    double scan_p95_ms = 0.0;
    double scan_max_ms = 0.0;
};

inline double percentile(std::vector<double> values, double p)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const double idx = p * (values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

inline HeadlessResult runHeadless(const HeadlessConfig &config)
{
    SimulationState sim = makeSimulation(config.seed);
    sim.query_mode = config.query_mode;

    std::vector<double> scan_ms;
    scan_ms.reserve(config.frames);

    Vec3 prev = sim.lidar.pose.translation();
    double distance = 0.0;
    int mismatches = 0;

    const bool accelerated = config.query_mode != QueryMode::BruteForce;

    for (int i = 0; i < config.frames; ++i)
    {
        stepSimulation(sim, config.dt);
        scan_ms.push_back(sim.metrics.scan_ms);

        const Vec3 pos = sim.lidar.pose.translation();
        distance += (pos - prev).norm();
        prev = pos;

        if (config.verify && accelerated && config.verify_interval > 0 &&
            i % config.verify_interval == 0)
            mismatches += verifyAcceleratedQueries(sim);
    }

    HeadlessResult result;
    result.query_mode = config.query_mode;
    result.frames = config.frames;
    result.distance = distance;
    result.collisions = sim.metrics.collision_count;
    result.verification_mismatches = mismatches;
    result.verified = mismatches == 0;

    double sum = 0.0;
    double max = 0.0;
    for (double v : scan_ms)
    {
        sum += v;
        max = std::max(max, v);
    }
    result.scan_mean_ms = scan_ms.empty() ? 0.0 : sum / scan_ms.size();
    result.scan_p50_ms = percentile(scan_ms, 0.50);
    result.scan_p95_ms = percentile(scan_ms, 0.95);
    result.scan_max_ms = max;

    return result;
}
