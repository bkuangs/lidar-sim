#include "meshes.hpp"
#include "scene.hpp"
#include "scene_bvh.hpp"
#include "vehicle.hpp"
#include "lidar.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

/**
 * COST-MODEL CALIBRATION — measures the per-ray cost of each Layer 2 tracer on
 * the ACTUAL pillar geometry, over a grid of (obstacle count N, triangles/obstacle),
 * then fits the assumed functional forms and reports fit quality (R^2).
 *
 * This replaces hand-transcribed single-point constants with fitted, validated
 * ones. The output constants are pasted into layer2.hpp's CostModel (baked, for
 * machine-independent reproducibility) with this run cited as provenance.
 *
 *   true-brute : ns = a + b * (N * tris)     (triangle soup, no acceleration)
 *   mesh-BVH   : ns = a + b * N              (O(N) object walk, per-mesh BVH leaves)
 *   scene-BVH  : ns = a + b * log2(N)        (top-level tree)
 */

namespace
{
constexpr double kMaxRange = 50.0;
constexpr double kCorridor = 50.0;
constexpr double kHalfWidth = 2.0;
constexpr double kHeight = 3.0;
constexpr int kSlices = 28;

struct Fit
{
    double a = 0.0, b = 0.0, r2 = 0.0;
};

// Ordinary least squares for y = a + b*x, plus R^2.
Fit linreg(const std::vector<double> &x, const std::vector<double> &y)
{
    const int n = static_cast<int>(x.size());
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i)
    {
        sx += x[i];
        sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    Fit f;
    const double denom = n * sxx - sx * sx;
    f.b = denom != 0.0 ? (n * sxy - sx * sy) / denom : 0.0;
    f.a = (sy - f.b * sx) / n;
    double ss_tot = 0, ss_res = 0;
    const double ymean = sy / n;
    for (int i = 0; i < n; ++i)
    {
        const double pred = f.a + f.b * x[i];
        ss_res += (y[i] - pred) * (y[i] - pred);
        ss_tot += (y[i] - ymean) * (y[i] - ymean);
    }
    f.r2 = ss_tot != 0.0 ? 1.0 - ss_res / ss_tot : 1.0;
    return f;
}

std::vector<std::shared_ptr<Geometry>> makeStatics()
{
    std::vector<std::shared_ptr<Geometry>> s;
    s.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0));
    s.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, -1), Vec3(0, 0, kHeight), 1));
    s.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 1, 0), Vec3(0, -kHalfWidth, 0), 2));
    s.push_back(std::make_shared<PlaneGeometry>(Vec3(0, -1, 0), Vec3(0, kHalfWidth, 0), 3));
    return s;
}

// N pillars with `stacks` stacks (=> 2*slices*(stacks+1) triangles each).
std::vector<ActiveObstacle> makePillars(int n, int stacks, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x_dist(2.0, kCorridor - 2.0);
    std::uniform_real_distribution<double> size_dist(0.25, 0.75);
    std::vector<ActiveObstacle> obs;
    obs.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const double size = size_dist(rng);
        const double half = size * 0.5;
        std::uniform_real_distribution<double> y_dist(-kHalfWidth + half, kHalfWidth - half);
        const double x = x_dist(rng), y = y_dist(rng);
        auto pillar = makePillar(x, y, half, kHeight, kSlices, stacks, 10 + i);
        ActiveObstacle o;
        o.x = x;
        o.center = Vec3(x, y, kHeight * 0.5);
        o.size = size;
        o.object_id = 10 + i;
        o.bounds = pillar->bounds();
        o.geometry = pillar;
        obs.push_back(o);
    }
    return obs;
}

// Forward-arc rays from random poses (matches the sim's ray usage).
std::vector<Ray> makeRays(int poses, int per_pose, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x_dist(0.0, kCorridor);
    std::uniform_real_distribution<double> y_dist(-kHalfWidth * 0.8, kHalfWidth * 0.8);
    std::uniform_real_distribution<double> h_dist(-geom::pi, geom::pi);
    VehicleConfig vcfg;
    std::vector<Ray> rays;
    rays.reserve(poses * per_pose);
    const int denom = std::max(1, per_pose - 1);
    for (int p = 0; p < poses; ++p)
    {
        VehicleState veh;
        veh.x = x_dist(rng);
        veh.y = y_dist(rng);
        veh.heading = h_dist(rng);
        const Eigen::Isometry3d pose = lidarPoseFromVehicle(veh, vcfg);
        for (int r = 0; r < per_pose; ++r)
        {
            const double az = -geom::pi / 2.0 + geom::pi * r / denom;
            Ray ray;
            ray.ori = pose.translation();
            ray.dir = pose.rotation() * Vec3(std::cos(az), std::sin(az), 0.0);
            rays.push_back(ray);
        }
    }
    return rays;
}

template <typename Fn>
double timeNsPerRay(const std::vector<Ray> &rays, Fn fn, int passes)
{
    volatile double sink = 0.0;
    for (const Ray &r : rays) // warmup
        sink += fn(r).t;
    auto t0 = std::chrono::steady_clock::now();
    volatile double cs = 0.0;
    for (int p = 0; p < passes; ++p)
        for (const Ray &r : rays)
            cs += fn(r).t;
    auto t1 = std::chrono::steady_clock::now();
    (void)sink;
    (void)cs;
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    return ns / (passes * rays.size());
}
} // namespace

int main()
{
    const std::vector<int> Ns = {25, 50, 100, 200};
    const std::vector<int> stacks_list = {4, 8, 16}; // tris = 2*28*(stacks+1) = 280, 504, 952
    const int passes = 3;
    const unsigned seed = 42;

    const std::vector<Ray> rays = makeRays(30, 61, 7); // 1830 rays
    const auto statics = makeStatics();

    std::printf("cost-model calibration  rays=%zu passes=%d seed=%u slices=%d\n\n",
                rays.size(), passes, seed, kSlices);
    std::printf("%6s %6s %8s | %14s %12s %12s\n",
                "N", "stacks", "tris", "brute ns/ray", "mesh ns/ray", "scene ns/ray");

    std::vector<double> x_brute, y_brute, x_mesh, y_mesh, x_scene, y_scene;

    for (int stacks : stacks_list)
    {
        const int tris = 2 * kSlices * (stacks + 1);
        for (int N : Ns)
        {
            auto obs = makePillars(N, stacks, seed);

            Scene scene; // mesh-BVH tier: linear scan, per-mesh BVH leaves
            scene.objects = statics;
            for (auto &o : obs)
                scene.objects.push_back(o.geometry);

            SceneBVH bvh;
            bvh.rebuild(statics, obs);

            auto brute_fn = [&](const Ray &ray) {
                Hit closest;
                for (auto &s : statics)
                {
                    Hit h = s->intersect(ray);
                    if (h.hit && h.t <= kMaxRange && (!closest.hit || h.t < closest.t))
                        closest = h;
                }
                for (auto &o : obs)
                {
                    auto mesh = std::static_pointer_cast<TriangleMeshGeometry>(o.geometry);
                    Hit h = mesh->bruteForceIntersect(ray);
                    if (h.hit && h.t <= kMaxRange && (!closest.hit || h.t < closest.t))
                        closest = h;
                }
                return closest;
            };
            auto mesh_fn = [&](const Ray &ray) { return scene.intersect(ray); };
            auto scene_fn = [&](const Ray &ray) { return bvh.intersect(ray, kMaxRange); };

            const double b_ns = timeNsPerRay(rays, brute_fn, passes);
            const double m_ns = timeNsPerRay(rays, mesh_fn, passes);
            const double s_ns = timeNsPerRay(rays, scene_fn, passes);

            std::printf("%6d %6d %8d | %14.1f %12.1f %12.2f\n", N, stacks, tris, b_ns, m_ns, s_ns);

            x_brute.push_back(static_cast<double>(N) * tris);
            y_brute.push_back(b_ns);
            x_mesh.push_back(static_cast<double>(N));
            y_mesh.push_back(m_ns);
            x_scene.push_back(std::log2(static_cast<double>(N)));
            y_scene.push_back(s_ns);
        }
    }

    const Fit fb = linreg(x_brute, y_brute);
    const Fit fm = linreg(x_mesh, y_mesh);
    const Fit fs = linreg(x_scene, y_scene);

    std::printf("\n--- fitted cost model (paste into layer2.hpp CostModel) ---\n");
    std::printf("brute:  ns = %.4f + %.5f * (N*tris)   R2=%.4f  -> brute_ns_per_tri = %.5f\n",
                fb.a, fb.b, fb.r2, fb.b);
    std::printf("mesh:   ns = %.4f + %.5f * N          R2=%.4f  -> mesh_ns_per_obstacle = %.5f\n",
                fm.a, fm.b, fm.r2, fm.b);
    std::printf("scene:  ns = %.4f + %.5f * log2(N)    R2=%.4f  -> bvh_base_ns = %.4f, bvh_log_ns = %.5f\n",
                fs.a, fs.b, fs.r2, fs.a, fs.b);
    return 0;
}
