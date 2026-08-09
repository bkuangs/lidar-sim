#include "accelerated_scene.hpp"
#include "scene_bvh.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

/**
 * LAYER 1 — Raytracing scaling benchmark (headless, Open3D-free).
 *
 * Builds a dense field of N obstacles in a fixed-length corridor and measures,
 * per query mode, the accelerator build time and the per-ray query cost as N
 * scales. The baseline "linear-scan" mode is the O(N) scene walk
 * (Scene::intersect tests the ray against every object; each object still uses
 * its own per-mesh BVH / analytic form), contrasted against the O(log N)
 * scene-level SceneBVH. This shows the top-level tree turning an O(N) scan into
 * O(log N): the per-ray cost stays flat as N grows.
 *
 * NOTE: "linear-scan" here is NOT triangle-soup brute force. Obstacles are
 * analytic spheres / 12-tri cubes, so per-object cost is tiny — this isolates
 * scene-level scaling (N vs log N), not per-object triangle cost. Layer 2's
 * "true-brute" is the separate triangle-soup baseline.
 *
 * Output: a table plus a CSV (N, mode, build_ms, query_ms, ns_per_ray, ...).
 */

namespace
{
constexpr double kMaxRange = 50.0;
constexpr double kCorridorLength = 50.0;
constexpr double kHalfWidth = 2.0;
constexpr double kHeight = 3.0;

struct DenseScene
{
    std::vector<ActiveObstacle> obstacles;
    std::vector<std::shared_ptr<Geometry>> static_objects;
    Scene scene; // linear-scan reference: O(N) walk over all objects (each with its own per-mesh BVH)
};

std::vector<std::shared_ptr<Geometry>> makeStaticCorridor()
{
    std::vector<std::shared_ptr<Geometry>> objects;
    objects.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0));
    objects.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, -1), Vec3(0, 0, kHeight), 1));
    objects.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 1, 0), Vec3(0, -kHalfWidth, 0), 2));
    objects.push_back(std::make_shared<PlaneGeometry>(Vec3(0, -1, 0), Vec3(0, kHalfWidth, 0), 3));
    return objects;
}

DenseScene makeDenseScene(int n, unsigned int seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x_dist(2.0, kCorridorLength - 2.0);
    std::uniform_real_distribution<double> size_dist(0.25, 0.75);
    std::uniform_int_distribution<int> shape_dist(0, 1);

    DenseScene ds;
    ds.static_objects = makeStaticCorridor();
    ds.obstacles.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        const double size = size_dist(rng);
        const double half = size * 0.5;
        std::uniform_real_distribution<double> y_dist(-kHalfWidth + half, kHalfWidth - half);
        const Vec3 center(x_dist(rng), y_dist(rng), half);
        const int object_id = 10 + i;
        const ObstacleShape shape = shape_dist(rng) == 0 ? ObstacleShape::Sphere : ObstacleShape::Cube;

        std::shared_ptr<Geometry> geometry =
            shape == ObstacleShape::Sphere
                ? std::static_pointer_cast<Geometry>(std::make_shared<SphereGeometry>(center, half, object_id))
                : std::static_pointer_cast<Geometry>(makeCube(center, size, object_id));

        ActiveObstacle obstacle;
        obstacle.x = center.x();
        obstacle.center = center;
        obstacle.size = size;
        obstacle.object_id = object_id;
        obstacle.shape = shape;
        obstacle.bounds = geometry->bounds();
        obstacle.geometry = geometry;
        ds.obstacles.push_back(obstacle);
    }

    ds.scene.objects = ds.static_objects;
    for (const ActiveObstacle &o : ds.obstacles)
        ds.scene.objects.push_back(o.geometry);

    return ds;
}

std::vector<Ray> makeRayField()
{
    std::vector<Ray> rays;
    const int poses = 8;
    const int azimuth_samples = 181; // forward arc
    for (int p = 0; p < poses; ++p)
    {
        const double px = kCorridorLength * (0.1 + 0.8 * p / (poses - 1));
        const Vec3 ori(px, 0.0, 1.0);
        for (double elevation : {-3.0 * geom::deg, 0.0, 3.0 * geom::deg})
        {
            for (int a = 0; a < azimuth_samples; ++a)
            {
                const double t = static_cast<double>(a) / (azimuth_samples - 1);
                const double azimuth = -60.0 * geom::deg + t * (120.0 * geom::deg);
                Ray ray;
                ray.ori = ori;
                ray.dir = Vec3(std::cos(elevation) * std::cos(azimuth),
                               std::cos(elevation) * std::sin(azimuth),
                               std::sin(elevation))
                              .normalized();
                rays.push_back(ray);
            }
        }
    }
    return rays;
}

double msSince(const std::chrono::steady_clock::time_point &start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

struct ModeResult
{
    double build_ms = 0.0;
    double query_ms = 0.0;
    double ns_per_ray = 0.0;
    int mismatches = 0;
};

// Time a query pass; return checksum to prevent dead-code elimination.
template <typename Intersect>
double queryPass(const std::vector<Ray> &rays, Intersect intersect)
{
    double checksum = 0.0;
    for (const Ray &ray : rays)
    {
        const Hit hit = intersect(ray);
        if (hit.hit && hit.t <= kMaxRange)
            checksum += hit.t;
    }
    return checksum;
}
} // namespace

int main(int argc, char **argv)
{
    std::vector<int> sweep = {10, 25, 50, 100, 200, 400, 800, 1600, 3200};
    unsigned int seed = 42;
    int passes = 3;
    std::string csv_path = "scaling.csv";

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : "0"; };
        if (arg == "--seed")
            seed = static_cast<unsigned int>(std::strtoul(next(), nullptr, 10));
        else if (arg == "--passes")
            passes = std::atoi(next());
        else if (arg == "--csv")
            csv_path = next();
        else
        {
            std::cerr << "unknown argument: " << arg << '\n';
            return 1;
        }
    }

    const std::vector<Ray> rays = makeRayField();

    std::ofstream csv(csv_path);
    csv << "N,mode,build_ms,query_ms,ns_per_ray,rays,mismatches\n";

    std::cout << "rays/pass=" << rays.size() << " passes=" << passes << " seed=" << seed << "\n\n";
    std::cout << std::right << std::setw(6) << "N"
              << std::setw(12) << "mode"
              << std::setw(11) << "build_ms"
              << std::setw(11) << "query_ms"
              << std::setw(12) << "ns/ray"
              << std::setw(12) << "bvh_vs_scan"
              << std::setw(10) << "verified" << '\n';

    for (int n : sweep)
    {
        DenseScene ds = makeDenseScene(n, seed);

        XBucketScene bucket;
        SceneBVH bvh;

        ModeResult scan_r, buckets_r, bvh_r;

        // ---- build times ----
        scan_r.build_ms = 0.0; // no acceleration structure
        {
            auto t = std::chrono::steady_clock::now();
            bucket.rebuild(ds.static_objects, ds.obstacles);
            buckets_r.build_ms = msSince(t);
        }
        {
            auto t = std::chrono::steady_clock::now();
            bvh.rebuild(ds.static_objects, ds.obstacles);
            bvh_r.build_ms = msSince(t);
        }

        auto scan_fn = [&](const Ray &r) { return ds.scene.intersect(r); };
        auto bucket_fn = [&](const Ray &r) { return bucket.intersect(r, kMaxRange); };
        auto bvh_fn = [&](const Ray &r) { return bvh.intersect(r, kMaxRange); };

        // ---- warmup ----
        volatile double sink = 0.0;
        sink += queryPass(rays, scan_fn);
        sink += queryPass(rays, bucket_fn);
        sink += queryPass(rays, bvh_fn);
        (void)sink;

        // ---- timed query passes ----
        auto timeMode = [&](auto fn, ModeResult &out) {
            auto t = std::chrono::steady_clock::now();
            volatile double cs = 0.0;
            for (int p = 0; p < passes; ++p)
                cs += queryPass(rays, fn);
            (void)cs;
            out.query_ms = msSince(t) / passes;
            out.ns_per_ray = out.query_ms * 1e6 / rays.size();
        };
        timeMode(scan_fn, scan_r);
        timeMode(bucket_fn, buckets_r);
        timeMode(bvh_fn, bvh_r);

        // ---- correctness gate ----
        constexpr double tol = 1e-6;
        auto countMismatches = [&](auto fn) {
            int m = 0;
            for (const Ray &ray : rays)
            {
                const Hit ref = ds.scene.intersect(ray);
                const Hit ac = fn(ray);
                const bool ref_valid = ref.hit && ref.t <= kMaxRange;
                const bool a_valid = ac.hit && ac.t <= kMaxRange;
                if ((ref_valid != a_valid) ||
                    (ref_valid && (ref.objId != ac.objId || std::abs(ref.t - ac.t) > tol)))
                    ++m;
            }
            return m;
        };
        buckets_r.mismatches = countMismatches(bucket_fn);
        bvh_r.mismatches = countMismatches(bvh_fn);

        const double bvh_speedup = bvh_r.ns_per_ray > 0 ? scan_r.ns_per_ray / bvh_r.ns_per_ray : 0.0;
        const bool verified = buckets_r.mismatches == 0 && bvh_r.mismatches == 0;

        auto row = [&](const char *mode, const ModeResult &r) {
            std::cout << std::right << std::setw(6) << n
                      << std::setw(12) << mode
                      << std::fixed << std::setprecision(3)
                      << std::setw(11) << r.build_ms
                      << std::setw(11) << r.query_ms
                      << std::setw(12) << r.ns_per_ray;
            if (std::string(mode) == "scene-bvh")
                std::cout << std::setw(12) << std::setprecision(2) << bvh_speedup << "x";
            else
                std::cout << std::setw(12) << "-";
            std::cout << std::setw(10) << (std::string(mode) == "linear-scan"
                                               ? "n/a"
                                               : (r.mismatches == 0 ? "ok" : "MISMATCH"))
                      << '\n';
            csv << n << ',' << mode << ',' << r.build_ms << ',' << r.query_ms << ','
                << r.ns_per_ray << ',' << rays.size() << ',' << r.mismatches << '\n';
        };
        row("linear-scan", scan_r);
        row("x-buckets", buckets_r);
        row("scene-bvh", bvh_r);
        std::cout << '\n';
        (void)verified;
    }

    std::cout << "wrote " << csv_path << '\n';
    return 0;
}
