#include "harness.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

/**
 * Headless benchmark driver.
 *
 * Runs the same seeded course through every query mode and prints aggregated
 * scan latency plus behavior metrics. This is the Layer-1 seed: a fair,
 * reproducible brute vs buckets vs BVH comparison with no viewer.
 *
 * Usage: lidar_benchmark [--seed N] [--frames N] [--dt S] [--no-verify]
 */
int main(int argc, char **argv)
{
    HeadlessConfig config;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : "0"; };

        if (arg == "--seed")
            config.seed = static_cast<unsigned int>(std::strtoul(next(), nullptr, 10));
        else if (arg == "--frames")
            config.frames = std::atoi(next());
        else if (arg == "--dt")
            config.dt = std::atof(next());
        else if (arg == "--no-verify")
            config.verify = false;
        else
        {
            std::cerr << "unknown argument: " << arg << '\n';
            return 1;
        }
    }

    std::cout << "seed=" << config.seed << " frames=" << config.frames
              << " dt=" << config.dt << "\n\n";

    std::cout << std::left << std::setw(12) << "mode"
              << std::right << std::setw(11) << "scan_mean"
              << std::setw(10) << "scan_p50"
              << std::setw(10) << "scan_p95"
              << std::setw(10) << "scan_max"
              << std::setw(7) << "coll"
              << std::setw(10) << "dist_m"
              << std::setw(11) << "verified" << '\n';

    const QueryMode modes[] = {
        QueryMode::BruteForce,
        QueryMode::XBuckets,
        QueryMode::SceneBVH};

    for (QueryMode mode : modes)
    {
        config.query_mode = mode;
        const HeadlessResult r = runHeadless(config);

        std::cout << std::fixed << std::setprecision(3)
                  << std::left << std::setw(12) << queryModeName(mode)
                  << std::right << std::setw(11) << r.scan_mean_ms
                  << std::setw(10) << r.scan_p50_ms
                  << std::setw(10) << r.scan_p95_ms
                  << std::setw(10) << r.scan_max_ms
                  << std::setw(7) << r.collisions
                  << std::setw(10) << r.distance
                  << std::setw(11) << (mode == QueryMode::BruteForce
                                           ? "n/a"
                                           : (r.verified ? "ok" : "MISMATCH"))
                  << '\n';
    }

    return 0;
}
