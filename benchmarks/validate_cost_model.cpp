#include "layer2.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

/**
 * Validates Layer 2's baked cost model against the real triangle-soup and
 * scene-BVH query paths. World construction, ray generation, and planner work
 * are outside the timed region because CostModel represents query cost only.
 *
 * Usage:
 *   validate_cost_model [--seeds 3] [--poses 6] [--repeats 5] [--csv]
 */

namespace
{
using Clock = std::chrono::steady_clock;

struct BudgetSamples
{
    std::vector<double> modeled_k;
    std::vector<double> measured_k;
    std::vector<double> model_ns;
    std::vector<double> next_ns;
    int model_misses = 0;
    int next_meets = 0;
};

struct ModeSamples
{
    Layer2Mode mode;
    std::vector<BudgetSamples> budgets;
};

double mean(const std::vector<double> &values)
{
    double sum = 0.0;
    for (double value : values)
        sum += value;
    return values.empty() ? 0.0 : sum / values.size();
}

double percentile(std::vector<double> values, double quantile)
{
    if (values.empty())
        return 0.0;

    std::sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        std::floor(quantile * static_cast<double>(values.size() - 1)));
    return values[index];
}

std::vector<Ray> makeRays(const Eigen::Isometry3d &pose, int count)
{
    std::vector<Ray> rays;
    rays.reserve(count);
    for (int h = 0; h < count; ++h)
    {
        const double fraction = static_cast<double>(h) / count;
        const double azimuth = -geom::pi / 2.0 + fraction * geom::pi;

        Ray ray;
        ray.ori = pose.translation();
        ray.dir = pose.rotation() * Vec3(std::cos(azimuth), std::sin(azimuth), 0.0);
        rays.push_back(ray);
    }
    return rays;
}

template <typename Intersector>
double timeQueries(const std::vector<Ray> &rays, Intersector intersect, volatile double &checksum)
{
    const auto start = Clock::now();
    for (const Ray &ray : rays)
    {
        const Hit hit = intersect(ray);
        checksum += hit.hit ? hit.t + 1e-6 * hit.objId : 0.0;
    }
    const auto end = Clock::now();
    return std::chrono::duration<double, std::nano>(end - start).count();
}

void printResults(const std::vector<ModeSamples> &all_samples, bool csv)
{
    if (csv)
    {
        std::printf("mode,budget_ms,modeled_k_mean,measured_k_p05,"
                    "model_p95_ms,model_miss_pct,next_p95_ms,next_meet_pct,samples\n");
    }
    else
    {
        std::printf("Query-only timing; measured_K is the conservative p05 ray capacity\n"
                    "estimated from each full-resolution scan's measured throughput.\n"
                    "model/next are p95 scan times at modeled K and K+1.\n\n");
        std::printf("%-11s %7s %9s %11s %11s %9s %11s %10s %8s\n",
                    "mode", "budget", "model_K", "measured_K", "model_p95",
                    "miss%", "next_p95", "next_meet%", "samples");
    }

    for (const ModeSamples &mode_samples : all_samples)
    {
        for (size_t b = 0; b < layer2BudgetsMs.size(); ++b)
        {
            const BudgetSamples &samples = mode_samples.budgets[b];
            const int count = static_cast<int>(samples.model_ns.size());
            const double miss_pct = count > 0
                                        ? 100.0 * samples.model_misses / count
                                        : 0.0;
            const double next_meet_pct = !samples.next_ns.empty()
                                             ? 100.0 * samples.next_meets / samples.next_ns.size()
                                             : 0.0;
            const double model_p95_ms = percentile(samples.model_ns, 0.95) / 1e6;
            const double next_p95_ms = percentile(samples.next_ns, 0.95) / 1e6;

            if (csv)
            {
                std::printf("%s,%.2f,%.2f,%.0f,%.6f,%.2f,",
                            layer2ModeName(mode_samples.mode), layer2BudgetsMs[b],
                            mean(samples.modeled_k), percentile(samples.measured_k, 0.05),
                            model_p95_ms, miss_pct);
                if (samples.next_ns.empty())
                    std::printf(",,%d\n", count);
                else
                    std::printf("%.6f,%.2f,%d\n", next_p95_ms, next_meet_pct, count);
            }
            else
            {
                std::printf("%-11s %6.2fms %9.2f %11.0f %9.3fms %8.1f%% ",
                            layer2ModeName(mode_samples.mode), layer2BudgetsMs[b],
                            mean(samples.modeled_k), percentile(samples.measured_k, 0.05),
                            model_p95_ms, miss_pct);
                if (samples.next_ns.empty())
                    std::printf("%11s %10s %8d\n", "n/a", "n/a", count);
                else
                    std::printf("%9.3fms %9.1f%% %8d\n",
                                next_p95_ms, next_meet_pct, count);
            }
        }
    }
}
} // namespace

int main(int argc, char **argv)
{
    int seeds = 3;
    int poses = 6;
    int repeats = 5;
    bool csv = false;

    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc)
            seeds = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--poses") && i + 1 < argc)
            poses = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--repeats") && i + 1 < argc)
            repeats = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv"))
            csv = true;
        else
        {
            std::fprintf(stderr,
                         "usage: validate_cost_model [--seeds N] [--poses N] "
                         "[--repeats N] [--csv]\n");
            return 2;
        }
    }

    if (seeds <= 0 || poses <= 0 || repeats <= 0)
    {
        std::fprintf(stderr, "seeds, poses, and repeats must all be positive\n");
        return 2;
    }

    std::vector<ModeSamples> all_samples = {
        {Layer2Mode::TrueBrute, std::vector<BudgetSamples>(layer2BudgetsMs.size())},
        {Layer2Mode::SceneBVH, std::vector<BudgetSamples>(layer2BudgetsMs.size())},
    };

    volatile double checksum = 0.0;
    VehicleConfig vehicle_config;

    for (int seed_index = 0; seed_index < seeds; ++seed_index)
    {
        Layer2Config config;
        config.seed = 1000u + static_cast<unsigned>(seed_index);
        const Layer2World world = makeLayer2World(config);

        CostModel cost;
        cost.tris_per_obstacle = world.tris_per_obstacle;

        std::mt19937 rng(7000u + static_cast<unsigned>(seed_index));
        std::uniform_real_distribution<double> x_dist(0.0, config.course_length);
        std::uniform_real_distribution<double> y_dist(
            -0.7 * config.corridor_half_width, 0.7 * config.corridor_half_width);
        std::uniform_real_distribution<double> heading_dist(
            -10.0 * geom::deg, 10.0 * geom::deg);

        for (int pose_index = 0; pose_index < poses; ++pose_index)
        {
            VehicleState vehicle;
            vehicle.x = x_dist(rng);
            vehicle.y = y_dist(rng);
            vehicle.heading = heading_dist(rng);
            const Eigen::Isometry3d pose = lidarPoseFromVehicle(vehicle, vehicle_config);

            for (ModeSamples &mode_samples : all_samples)
            {
                auto intersect = [&](const Ray &ray) {
                    if (mode_samples.mode == Layer2Mode::TrueBrute)
                        return layer2ReferenceHit(world, ray, config.max_range);
                    return world.bvh.intersect(ray, config.max_range);
                };

                const std::vector<Ray> warmup_rays = makeRays(pose, 16);
                for (const Ray &ray : warmup_rays)
                {
                    const Hit hit = intersect(ray);
                    checksum += hit.hit ? hit.t : 0.0;
                }

                const int obstacle_count = static_cast<int>(world.obstacles.size());
                const double modeled_cost =
                    cost.costRayNs(mode_samples.mode, obstacle_count);

                struct RaySets
                {
                    int modeled_k;
                    std::vector<Ray> modeled;
                    std::vector<Ray> next;
                };
                std::vector<RaySets> ray_sets;
                ray_sets.reserve(layer2BudgetsMs.size());
                for (double budget_ms : layer2BudgetsMs)
                {
                    const int modeled_k =
                        affordableRayCount(budget_ms * 1e6, modeled_cost, config.k_max);
                    ray_sets.push_back({
                        modeled_k,
                        makeRays(pose, modeled_k),
                        modeled_k < config.k_max ? makeRays(pose, modeled_k + 1)
                                                 : std::vector<Ray>{},
                    });
                }

                const std::vector<Ray> probe_rays = makeRays(pose, config.k_max);
                for (int repeat = 0; repeat < repeats; ++repeat)
                {
                    const double probe_ns = timeQueries(probe_rays, intersect, checksum);
                    const double measured_cost = probe_ns / probe_rays.size();

                    for (size_t b = 0; b < layer2BudgetsMs.size(); ++b)
                    {
                        const double budget_ns = layer2BudgetsMs[b] * 1e6;
                        const RaySets &rays = ray_sets[b];
                        BudgetSamples &samples = mode_samples.budgets[b];

                        const double model_ns =
                            timeQueries(rays.modeled, intersect, checksum);
                        samples.modeled_k.push_back(rays.modeled_k);
                        samples.measured_k.push_back(
                            affordableRayCount(budget_ns, measured_cost, config.k_max));
                        samples.model_ns.push_back(model_ns);
                        samples.model_misses += model_ns > budget_ns ? 1 : 0;

                        if (!rays.next.empty())
                        {
                            const double next_ns =
                                timeQueries(rays.next, intersect, checksum);
                            samples.next_ns.push_back(next_ns);
                            samples.next_meets += next_ns <= budget_ns ? 1 : 0;
                        }
                    }
                }
            }
        }
    }

    printResults(all_samples, csv);
    return checksum < 0.0 ? 1 : 0;
}
