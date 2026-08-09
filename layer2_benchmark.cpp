#include "layer2.hpp"
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
 *   layer2_benchmark [--speed 3.5] [--seeds 8] [--csv]
 */

int main(int argc, char **argv)
{
    double speed = 3.5;
    int seeds = 8;
    bool csv = false;
    bool skip_verify = false;

    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            speed = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc)
            seeds = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv"))
            csv = true;
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

    const std::vector<Layer2Mode> modes = {
        Layer2Mode::TrueBrute, Layer2Mode::MeshBVH, Layer2Mode::SceneBVH};
    const std::vector<double> budgets = {1.0, 1.5, 2.0, 3.0, 5.0, 8.0};

    if (csv)
        std::printf("mode,budget_ms,speed,coll_per_100m,collisions,distance,avg_k,avg_in_view,reached_frac\n");
    else
        std::printf("%-11s %8s %13s %8s %9s %8s %9s %9s\n",
                    "mode", "budget", "coll/100m", "coll", "dist", "avg_K", "avg_view", "reach%");

    for (Layer2Mode mode : modes)
    {
        for (double budget : budgets)
        {
            long coll = 0;
            double dist = 0.0, k_sum = 0.0, view_sum = 0.0;
            int reached = 0;

            for (int s = 0; s < seeds; ++s)
            {
                Layer2Config cfg;
                cfg.seed = 1000u + static_cast<unsigned>(s);
                cfg.mode = mode;
                cfg.target_speed = speed;
                cfg.budget_ms = budget;

                Layer2Result r = runLayer2(cfg);
                coll += r.collisions;
                dist += r.distance;
                k_sum += r.avg_k;
                view_sum += r.avg_in_view;
                reached += r.reached_end ? 1 : 0;
            }

            const double per100 = dist > 1.0 ? 100.0 * coll / dist : 0.0;
            const double avg_k = k_sum / seeds;
            const double avg_view = view_sum / seeds;
            const double reach_frac = 100.0 * reached / seeds;

            if (csv)
                std::printf("%s,%.2f,%.1f,%.4f,%ld,%.0f,%.1f,%.1f,%.0f\n",
                            layer2ModeName(mode), budget, speed, per100, coll, dist,
                            avg_k, avg_view, reach_frac);
            else
                std::printf("%-11s %8.2f %13.3f %8ld %9.0f %8.1f %9.1f %8.0f%%\n",
                            layer2ModeName(mode), budget, per100, coll, dist,
                            avg_k, avg_view, reach_frac);
        }
        if (!csv)
            std::printf("\n");
    }

    return 0;
}
