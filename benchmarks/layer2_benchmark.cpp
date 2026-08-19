#include "layer2.hpp"
#include "metrics.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <utility>
#include <vector>

/**
 * Layer 2 headline benchmark: collisions-per-100m vs per-frame perception budget.
 *
 * Each mode traces progressive rays with its real implementation until the live
 * per-frame deadline. Only completed rays reach the planner; the crossing ray is
 * discarded and its execution overrun is reported alongside safety.
 *
 * Collisions are normalised per 100m travelled: a blind mode drives further per
 * frame, so absolute counts and per-obstacle rates are confounded by distance.
 *
 * Usage:
 *   layer2_benchmark [--speed 3.5] [--seeds 32] [--csv]
 *                    [--with-mesh-bvh] [--no-verify] [--trials-csv PATH]
 */

namespace
{
double p95(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        std::ceil(0.95 * values.size()) - 1.0);
    return values[index];
}

void consumeScan(const ScanResult &scan, volatile double &checksum)
{
    checksum += scan.rays_completed;
    for (const ScanPoint &point : scan.points)
        checksum += point.hit ? point.range : -1.0;
}

double measureScanCostNs(const Layer2Config &cfg, int poses = 6, int repeats = 3)
{
    const Layer2World world = makeLayer2World(cfg);

    VehicleConfig vehicle_config;
    Lidar lidar;
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.maxRange = cfg.max_range;
    const std::vector<int> azimuth_order = progressiveAzimuthOrder(cfg.k_max);

    std::mt19937 rng(cfg.seed + 7000u);
    std::uniform_real_distribution<double> x_dist(0.0, cfg.course_length);
    std::uniform_real_distribution<double> y_dist(
        -0.7 * cfg.corridor_half_width, 0.7 * cfg.corridor_half_width);
    std::uniform_real_distribution<double> heading_dist(
        -10.0 * geom::deg, 10.0 * geom::deg);

    volatile double checksum = 0.0;
    std::vector<double> scan_times_ns;
    scan_times_ns.reserve(poses * repeats);
    for (int pose_index = 0; pose_index < poses; ++pose_index)
    {
        VehicleState vehicle;
        vehicle.x = x_dist(rng);
        vehicle.y = y_dist(rng);
        vehicle.heading = heading_dist(rng);
        lidar.pose = lidarPoseFromVehicle(vehicle, vehicle_config);

        lidar.azimuthSamples = 16;
        const ScanResult warmup =
            scanLayer2(world, cfg, lidar, azimuth_order);
        consumeScan(warmup, checksum);

        lidar.azimuthSamples = cfg.k_max;
        for (int repeat = 0; repeat < repeats; ++repeat)
        {
            const auto start = std::chrono::steady_clock::now();
            const ScanResult scan =
                scanLayer2(world, cfg, lidar, azimuth_order);
            const auto end = std::chrono::steady_clock::now();
            scan_times_ns.push_back(
                std::chrono::duration<double, std::nano>(end - start).count());
            consumeScan(scan, checksum);
        }
    }

    (void)checksum;
    return p95(std::move(scan_times_ns)) / cfg.k_max;
}

struct AggregateReport
{
    const char *mode;
    double budget;
    double speed;
    double collision_mean;
    double collision_sd;
    int seeds;
    PairedBinaryBootstrap success;
    double reach_frac;
    double wall_contact_frac;
    double avg_progress;
    double mean_first_collision_distance;
    int first_collision_n;
    double predicted_k;
    double avg_k;
    int min_k;
    int max_k;
    double k_delta;
    double avg_view;
    double avg_cost;
    double avg_scan_ms;
    double avg_p95_ms;
    double cutoff_pct;
    double overrun_pct;
    double mean_overrun_ms;
    double max_overrun_ms;
};

void writeTrialHeader(FILE *output)
{
    std::fprintf(
        output,
        "mode,budget_ms,seed,speed,collision_free_completion,reached_end,"
        "wall_contact,progress,distance_to_first_collision,collisions,distance,"
        "collisions_per_100m,predicted_k,avg_k,min_k,max_k,execution_overrun_pct\n");
}

void writeTrial(
    FILE *output,
    Layer2Mode mode,
    double budget,
    unsigned seed,
    double speed,
    const Layer2Result &trial)
{
    const double overrun_pct =
        trial.frames > 0
            ? 100.0 * trial.execution_overruns / trial.frames
            : 0.0;
    std::fprintf(
        output,
        "%s,%.2f,%u,%.1f,%d,%d,%d,%.6f,%.6f,%d,%.6f,%.6f,"
        "%.2f,%.6f,%d,%d,%.6f\n",
        layer2ModeName(mode),
        budget,
        seed,
        speed,
        trial.collision_free_completion ? 1 : 0,
        trial.reached_end ? 1 : 0,
        trial.wall_contact ? 1 : 0,
        trial.progress,
        trial.distance_to_first_collision,
        trial.collisions,
        trial.distance,
        trial.collisions_per_100m,
        trial.predicted_k,
        trial.avg_k,
        trial.min_k,
        trial.max_k,
        overrun_pct);
}

void writeAggregateHeader()
{
    std::printf(
        "mode,budget_ms,speed,coll_per_100m_mean,coll_per_100m_std,n_seeds,"
        "success_pct,success_ci_low_pct,success_ci_high_pct,"
        "success_delta_pp,success_delta_ci_low_pp,success_delta_ci_high_pp,"
        "reach_frac,wall_contact_frac,avg_progress,mean_first_collision_distance,"
        "first_collision_n,predicted_k,avg_k,min_k,max_k,k_delta,avg_in_view,"
        "cost_ns_per_ray,avg_scan_ms,mean_run_p95_ms,deadline_cutoff_pct,"
        "execution_overrun_pct,mean_overrun_ms,max_overrun_ms\n");
}

void writeTextHeader()
{
    std::printf("Primary safety = collision-free course completion over ALL paired\n"
                "seeds. Delta and 95%% CI are paired bootstrap percentage-point\n"
                "differences versus true-brute. Reach and collisions/100m remain\n"
                "secondary outcomes; no failed or stuck seed is filtered out.\n\n");
    std::printf("Each mode traces until the live deadline; avg_K is the completed\n"
                "ray count. The p95 calibration is diagnostic only. overrun%% is\n"
                "the fraction whose final discarded ray crossed the budget.\n\n");
    std::printf("%-11s %8s %6s %12s %20s %9s %8s %8s %10s %8s\n",
                "mode", "budget", "n", "success", "delta (95% CI)", "reach%",
                "pred_K", "avg_K", "scan_p95", "overrun%");
}

void writeAggregate(const AggregateReport &report, bool csv)
{
    if (csv)
    {
        std::printf(
            "%s,%.2f,%.1f,%.4f,%.4f,%d,"
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.0f,%.0f,%.6f,%.6f,%d,%.1f,%.1f,%d,%d,"
            "%.1f,%.1f,%.1f,%.4f,%.4f,%.3f,%.3f,%.6f,%.6f\n",
            report.mode,
            report.budget,
            report.speed,
            report.collision_mean,
            report.collision_sd,
            report.seeds,
            report.success.rate_pct,
            report.success.rate_low_pct,
            report.success.rate_high_pct,
            report.success.delta_pct,
            report.success.delta_low_pct,
            report.success.delta_high_pct,
            report.reach_frac,
            report.wall_contact_frac,
            report.avg_progress,
            report.mean_first_collision_distance,
            report.first_collision_n,
            report.predicted_k,
            report.avg_k,
            report.min_k,
            report.max_k,
            report.k_delta,
            report.avg_view,
            report.avg_cost,
            report.avg_scan_ms,
            report.avg_p95_ms,
            report.cutoff_pct,
            report.overrun_pct,
            report.mean_overrun_ms,
            report.max_overrun_ms);
        return;
    }

    std::printf(
        "%-11s %8.2f %6d %10.1f%% %+6.1f [%+5.1f,%+5.1f] "
        "%8.0f%% %8.1f %8.1f %8.3fms %7.2f%%\n",
        report.mode,
        report.budget,
        report.seeds,
        report.success.rate_pct,
        report.success.delta_pct,
        report.success.delta_low_pct,
        report.success.delta_high_pct,
        report.reach_frac,
        report.predicted_k,
        report.avg_k,
        report.avg_p95_ms,
        report.overrun_pct);
}
} // namespace

int main(int argc, char **argv)
{
    double speed = 3.5;
    int seeds = 32;
    bool csv = false;
    bool skip_verify = false;
    bool with_mesh_bvh = false;
    std::string trials_csv_path;

    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            speed = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc)
            seeds = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv"))
            csv = true;
        else if (!std::strcmp(argv[i], "--with-mesh-bvh"))
            with_mesh_bvh = true;
        else if (!std::strcmp(argv[i], "--no-verify"))
            skip_verify = true;
        else if (!std::strcmp(argv[i], "--trials-csv") && i + 1 < argc)
            trials_csv_path = argv[++i];
        else
        {
            std::fprintf(stderr,
                         "usage: layer2_benchmark [--speed N] [--seeds N] [--csv] "
                         "[--with-mesh-bvh] [--no-verify] [--trials-csv PATH]\n");
            return 2;
        }
    }

    if (seeds <= 0)
    {
        std::fprintf(stderr, "seeds must be positive\n");
        return 2;
    }

    FILE *trials_csv = nullptr;
    if (!trials_csv_path.empty())
    {
        trials_csv = std::fopen(trials_csv_path.c_str(), "w");
        if (!trials_csv)
        {
            std::perror(("cannot open " + trials_csv_path).c_str());
            return 2;
        }
        writeTrialHeader(trials_csv);
    }

    // Verify that the accelerated tracer returns the same hits as the actual
    // triangle-soup path before comparing their trajectories.
    if (!skip_verify)
    {
        std::fprintf(stderr, "[layer2] verifying %d paired seeds\n", seeds);
        for (int s = 0; s < seeds; ++s)
        {
            Layer2Config cfg;
            cfg.seed = 1000u + static_cast<unsigned>(s);
            const Layer2Verification v = verifyLayer2Course(cfg);
            if (!v.passed())
            {
                std::fprintf(stderr,
                             "SCENE VERIFICATION FAILED (seed %u): %d/%d rays mismatched, "
                             "worst |dt|=%.3e; first: ref(obj=%d,t=%.4f) vs bvh(obj=%d,t=%.4f)\n",
                             cfg.seed, v.mismatches, v.rays, v.worst_t_err,
                             v.ex_ref_obj, v.ex_ref_t, v.ex_bvh_obj, v.ex_bvh_t);
                return 1;
            }
            if (!csv && s == 0)
                std::printf("scene verification: %d rays/seed x %d seeds, 0 mismatches (scene-BVH == triangle-soup ground truth) -- PASS\n\n",
                            v.rays, seeds);
        }
    }

    const std::vector<Layer2Mode> modes =
        with_mesh_bvh
            ? std::vector<Layer2Mode>{Layer2Mode::TrueBrute, Layer2Mode::MeshBVH, Layer2Mode::SceneBVH}
            : std::vector<Layer2Mode>{Layer2Mode::TrueBrute, Layer2Mode::SceneBVH};

    std::vector<std::vector<double>> measured_costs(
        modes.size(), std::vector<double>(seeds));
    for (size_t m = 0; m < modes.size(); ++m)
    {
        std::fprintf(
            stderr,
            "[layer2] calibrating %s (%d seeds)\n",
            layer2ModeName(modes[m]),
            seeds);
        for (int s = 0; s < seeds; ++s)
        {
            Layer2Config cfg;
            cfg.seed = 1000u + static_cast<unsigned>(s);
            cfg.mode = modes[m];
            measured_costs[m][s] = measureScanCostNs(cfg);
        }
    }

    csv ? writeAggregateHeader() : writeTextHeader();

    // Run every (mode, seed) once per budget and score all paired courses.
    for (size_t budget_index = 0; budget_index < layer2BudgetsMs.size(); ++budget_index)
    {
        const double budget = layer2BudgetsMs[budget_index];
        std::vector<std::vector<Layer2Result>> results(modes.size(),
                                                       std::vector<Layer2Result>(seeds));
        for (size_t m = 0; m < modes.size(); ++m)
        {
            std::fprintf(
                stderr,
                "[layer2] budget %.2f ms: %s (%d seeds)\n",
                budget,
                layer2ModeName(modes[m]),
                seeds);
            for (int s = 0; s < seeds; ++s)
            {
                Layer2Config cfg;
                cfg.seed = 1000u + static_cast<unsigned>(s);
                cfg.mode = modes[m];
                cfg.target_speed = speed;
                cfg.budget_ms = budget;
                results[m][s] = runLayer2(cfg);
                results[m][s].cost_ns_per_ray = measured_costs[m][s];
                results[m][s].predicted_k = affordableRayCount(
                    budget * 1e6, measured_costs[m][s], cfg.k_max);
            }
        }

        if (trials_csv)
        {
            for (size_t m = 0; m < modes.size(); ++m)
                for (int s = 0; s < seeds; ++s)
                {
                    writeTrial(
                        trials_csv,
                        modes[m],
                        budget,
                        1000u + static_cast<unsigned>(s),
                        speed,
                        results[m][s]);
                }
        }

        std::vector<char> reference_success(seeds, 0);
        for (int s = 0; s < seeds; ++s)
            reference_success[s] =
                results[0][s].collision_free_completion ? 1 : 0;

        for (size_t m = 0; m < modes.size(); ++m)
        {
            std::vector<char> success(seeds, 0);
            for (int s = 0; s < seeds; ++s)
                success[s] = results[m][s].collision_free_completion ? 1 : 0;
            const PairedBinaryBootstrap bootstrap = pairedBinaryBootstrap(
                success,
                reference_success,
                10000,
                9000u + static_cast<unsigned>(100 * budget_index + m));

            int reached = 0;
            int wall_contacts = 0;
            int first_collision_n = 0;
            int total_frames = 0;
            int total_cutoffs = 0;
            int total_misses = 0;
            int min_k = modes.empty() ? 0 : 361;
            int max_k = 0;
            double predicted_k_sum = 0.0, k_sum = 0.0;
            double view_sum = 0.0, cost_sum = 0.0;
            double weighted_scan_ms = 0.0, p95_sum = 0.0;
            double total_overrun_ms = 0.0, max_overrun_ms = 0.0;
            double progress_sum = 0.0, first_collision_sum = 0.0;
            for (int s = 0; s < seeds; ++s)
            {
                reached += results[m][s].reached_end ? 1 : 0;
                wall_contacts += results[m][s].wall_contact ? 1 : 0;
                progress_sum += results[m][s].progress;
                if (results[m][s].distance_to_first_collision >= 0.0)
                {
                    first_collision_sum +=
                        results[m][s].distance_to_first_collision;
                    ++first_collision_n;
                }
                predicted_k_sum += results[m][s].predicted_k;
                k_sum += results[m][s].avg_k;
                min_k = std::min(min_k, results[m][s].min_k);
                max_k = std::max(max_k, results[m][s].max_k);
                view_sum += results[m][s].avg_in_view;
                cost_sum += results[m][s].cost_ns_per_ray;
                weighted_scan_ms += results[m][s].avg_scan_ms * results[m][s].frames;
                p95_sum += results[m][s].p95_scan_ms;
                total_frames += results[m][s].frames;
                total_cutoffs += results[m][s].deadline_cutoffs;
                total_misses += results[m][s].execution_overruns;
                total_overrun_ms +=
                    results[m][s].mean_overrun_ms *
                    results[m][s].execution_overruns;
                max_overrun_ms =
                    std::max(max_overrun_ms, results[m][s].max_overrun_ms);
            }
            const double reach_frac = 100.0 * reached / seeds;
            const double wall_contact_frac = 100.0 * wall_contacts / seeds;
            const double avg_progress = progress_sum / seeds;
            const double mean_first_collision_distance =
                first_collision_n > 0
                    ? first_collision_sum / first_collision_n
                    : -1.0;
            const double predicted_k = predicted_k_sum / seeds;
            const double avg_k = k_sum / seeds;
            const double k_delta = avg_k - predicted_k;
            const double avg_view = view_sum / seeds;
            const double avg_cost = cost_sum / seeds;
            const double avg_scan_ms =
                total_frames > 0 ? weighted_scan_ms / total_frames : 0.0;
            const double avg_p95_ms = p95_sum / seeds;
            const double cutoff_pct =
                total_frames > 0 ? 100.0 * total_cutoffs / total_frames : 0.0;
            const double miss_pct =
                total_frames > 0 ? 100.0 * total_misses / total_frames : 0.0;
            const double mean_overrun_ms =
                total_misses > 0 ? total_overrun_ms / total_misses : 0.0;

            double mean = 0.0, sd = 0.0;
            for (int s = 0; s < seeds; ++s)
                mean += results[m][s].collisions_per_100m;
            mean /= seeds;
            if (seeds > 1)
            {
                for (int s = 0; s < seeds; ++s)
                {
                    const double d =
                        results[m][s].collisions_per_100m - mean;
                    sd += d * d;
                }
                sd = std::sqrt(sd / (seeds - 1));
            }

            writeAggregate(
                AggregateReport{
                    layer2ModeName(modes[m]),
                    budget,
                    speed,
                    mean,
                    sd,
                    seeds,
                    bootstrap,
                    reach_frac,
                    wall_contact_frac,
                    avg_progress,
                    mean_first_collision_distance,
                    first_collision_n,
                    predicted_k,
                    avg_k,
                    min_k,
                    max_k,
                    k_delta,
                    avg_view,
                    avg_cost,
                    avg_scan_ms,
                    avg_p95_ms,
                    cutoff_pct,
                    miss_pct,
                    mean_overrun_ms,
                    max_overrun_ms,
                },
                csv);
        }
        if (!csv)
            std::printf("\n");
    }

    if (trials_csv)
        std::fclose(trials_csv);
    std::fprintf(stderr, "[layer2] complete\n");
    return 0;
}
