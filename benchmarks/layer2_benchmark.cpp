#include "layer2.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/**
 * Layer 2 headline benchmark: collisions-per-100m vs per-frame perception budget.
 *
 * All modes trace with the same (correct) scene-BVH hits; the mode's baked cost
 * model decides how many azimuth rays it could AFFORD in the budget. As the
 * budget shrinks, true-brute (O(N.tris) per ray) is starved to a coarse ray fan,
 * aliases small pillars, and its collision rate climbs. Scene-BVH (O(log N))
 * always affords full angular resolution, so it holds a flat control-limited
 * floor independent of budget. The gap between the two curves is the pure
 * value of the accelerator, expressed as autonomy.
 *
 * Collisions are normalised per 100m travelled: a blind mode drives further per
 * frame, so absolute counts and per-obstacle rates are confounded by distance.
 *
 * Usage:
 *   layer2_benchmark [--speed 3.5] [--seeds 8] [--csv] [--with-mesh-bvh]
 */

int main(int argc, char **argv)
{
    double speed = 3.5;
    int seeds = 32;
    bool csv = false;
    bool skip_verify = false;
    bool with_mesh_bvh = false; // mesh-BVH ablation caps K identically to scene-BVH
                                // at these N, so it is off by default (opt-in row)

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
    }

    // Correctness gate: the sim always traces with the scene BVH, so verify it
    // returns ground-truth hits over the actual pillar course before trusting
    // any collision number. Fail fast if not.
    if (!skip_verify)
    {
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
    const std::vector<double> budgets = {1.0, 1.5, 2.0, 3.0, 5.0, 8.0};

    if (csv)
        std::printf("mode,budget_ms,speed,coll_per_100m_mean,coll_per_100m_std,n_scored,"
                    "reach_frac,avg_k,avg_in_view\n");
    else
    {
        std::printf("Safety = collisions/100m, mean +/- sample std over the COMMON set of\n"
                    "seeds every mode completes at that budget (apples-to-apples courses).\n"
                    "reach%% is each mode's own completion rate and is reported separately:\n"
                    "stuck / frame-capped seeds are excluded from the safety metric, never\n"
                    "blended into it. n = size of the common scored set.\n\n");
        std::printf("%-11s %8s %6s %20s %9s %8s %9s\n",
                    "mode", "budget", "n", "coll/100m (mean+/-sd)", "reach%", "avg_K", "avg_view");
    }

    // Run every (mode, seed) once per budget so we can build the common finisher
    // set and score all modes on identical courses.
    for (double budget : budgets)
    {
        std::vector<std::vector<Layer2Result>> results(modes.size(),
                                                       std::vector<Layer2Result>(seeds));
        for (size_t m = 0; m < modes.size(); ++m)
            for (int s = 0; s < seeds; ++s)
            {
                Layer2Config cfg;
                cfg.seed = 1000u + static_cast<unsigned>(s);
                cfg.mode = modes[m];
                cfg.target_speed = speed;
                cfg.budget_ms = budget;
                results[m][s] = runLayer2(cfg);
            }

        // Common finisher set: seeds reached_end == true for ALL modes.
        std::vector<int> common;
        for (int s = 0; s < seeds; ++s)
        {
            bool all = true;
            for (size_t m = 0; m < modes.size(); ++m)
                all = all && results[m][s].reached_end;
            if (all)
                common.push_back(s);
        }

        for (size_t m = 0; m < modes.size(); ++m)
        {
            int reached = 0;
            double k_sum = 0.0, view_sum = 0.0;
            for (int s = 0; s < seeds; ++s)
            {
                reached += results[m][s].reached_end ? 1 : 0;
                k_sum += results[m][s].avg_k;
                view_sum += results[m][s].avg_in_view;
            }
            const double reach_frac = 100.0 * reached / seeds;
            const double avg_k = k_sum / seeds;
            const double avg_view = view_sum / seeds;

            // Per-seed safety over the common set (equal-weighted, no distance pool).
            double mean = 0.0, sd = 0.0;
            const int n = static_cast<int>(common.size());
            if (n > 0)
            {
                for (int s : common)
                    mean += results[m][s].collisions_per_100m;
                mean /= n;
                if (n > 1)
                {
                    for (int s : common)
                    {
                        const double d = results[m][s].collisions_per_100m - mean;
                        sd += d * d;
                    }
                    sd = std::sqrt(sd / (n - 1));
                }
            }

            if (csv)
                std::printf("%s,%.2f,%.1f,%.4f,%.4f,%d,%.0f,%.1f,%.1f\n",
                            layer2ModeName(modes[m]), budget, speed, mean, sd, n,
                            reach_frac, avg_k, avg_view);
            else if (n > 0)
                std::printf("%-11s %8.2f %6d %13.2f +/-%4.2f %8.0f%% %8.1f %9.1f\n",
                            layer2ModeName(modes[m]), budget, n, mean, sd,
                            reach_frac, avg_k, avg_view);
            else
                std::printf("%-11s %8.2f %6d %20s %8.0f%% %8.1f %9.1f\n",
                            layer2ModeName(modes[m]), budget, n, "n/a (no common)",
                            reach_frac, avg_k, avg_view);
        }
        if (!csv)
            std::printf("\n");
    }

    return 0;
}
