#include "accelerated_scene.hpp"
#include "scene_bvh.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

/**
 * High-poly probe: validate that a TRUE brute-force baseline (every triangle,
 * no per-mesh BVH) over high-poly obstacles is starved at a realistic frame
 * budget and realistic obstacle count, while scene+mesh BVH stays flat.
 *
 * Compares three intersectors over identical high-poly meshes:
 *   - true-brute : test every triangle of every obstacle (bruteForceIntersect)
 *   - mesh-bvh   : per-mesh BVH only, still tests every obstacle (Scene-style)
 *   - scene-bvh  : scene BVH + per-mesh BVH
 */
namespace
{
constexpr double kMaxRange = 50.0;
constexpr double kLen = 50.0;
constexpr double kHalf = 2.0;
constexpr double kHeight = 3.0;

std::shared_ptr<TriangleMeshGeometry> makeUVSphere(
    const Vec3 &c, double r, int stacks, int slices, int objId)
{
    std::vector<Vec3> v;
    std::vector<Eigen::Vector3i> t;
    v.push_back(c + Vec3(0, 0, r)); // top pole = 0
    for (int i = 1; i < stacks; ++i)
    {
        const double phi = geom::pi * i / stacks;
        const double z = std::cos(phi), sr = std::sin(phi);
        for (int j = 0; j < slices; ++j)
        {
            const double th = 2.0 * geom::pi * j / slices;
            v.push_back(c + Vec3(r * sr * std::cos(th), r * sr * std::sin(th), r * z));
        }
    }
    const int bottom = static_cast<int>(v.size());
    v.push_back(c + Vec3(0, 0, -r));

    for (int j = 0; j < slices; ++j)
        t.push_back({0, 1 + j, 1 + (j + 1) % slices});
    for (int i = 0; i < stacks - 2; ++i)
    {
        const int row = 1 + i * slices, next = 1 + (i + 1) * slices;
        for (int j = 0; j < slices; ++j)
        {
            const int a = row + j, b = row + (j + 1) % slices;
            const int cc = next + j, d = next + (j + 1) % slices;
            t.push_back({a, cc, b});
            t.push_back({b, cc, d});
        }
    }
    const int last = 1 + (stacks - 2) * slices;
    for (int j = 0; j < slices; ++j)
        t.push_back({bottom, last + (j + 1) % slices, last + j});

    return std::make_shared<TriangleMeshGeometry>(v, t, objId);
}

std::vector<std::shared_ptr<Geometry>> staticCorridor()
{
    std::vector<std::shared_ptr<Geometry>> o;
    o.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0));
    o.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, -1), Vec3(0, 0, kHeight), 1));
    o.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 1, 0), Vec3(0, -kHalf, 0), 2));
    o.push_back(std::make_shared<PlaneGeometry>(Vec3(0, -1, 0), Vec3(0, kHalf, 0), 3));
    return o;
}

std::vector<Ray> rayField()
{
    std::vector<Ray> rays;
    for (int p = 0; p < 4; ++p)
    {
        const Vec3 ori(kLen * (0.15 + 0.7 * p / 3.0), 0.0, 1.0);
        for (double el : {-3.0 * geom::deg, 0.0, 3.0 * geom::deg})
            for (int a = 0; a < 121; ++a)
            {
                const double az = -60.0 * geom::deg + (120.0 * geom::deg) * a / 120.0;
                Ray r;
                r.ori = ori;
                r.dir = Vec3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)).normalized();
                rays.push_back(r);
            }
    }
    return rays;
}

double msSince(std::chrono::steady_clock::time_point s)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s).count();
}

template <typename F>
double timePass(const std::vector<Ray> &rays, int passes, F fn)
{
    volatile double cs = 0.0;
    auto t = std::chrono::steady_clock::now();
    for (int p = 0; p < passes; ++p)
        for (const Ray &r : rays)
        {
            Hit h = fn(r);
            if (h.hit)
                cs += h.t;
        }
    double ms = msSince(t) / passes;
    (void)cs;
    return ms;
}
} // namespace

int main()
{
    const int stacks = 16, slices = 16;
    const std::vector<int> Ns = {100, 200, 400};
    const std::vector<Ray> rays = rayField();
    const int passes = 2;

    // triangle count of one obstacle
    const int tris = 2 * slices + (stacks - 2) * slices * 2;
    std::cout << "high-poly UV sphere: " << tris << " tris/obstacle, rays=" << rays.size()
              << ", passes=" << passes << "\n\n";

    std::cout << std::right << std::setw(6) << "N"
              << std::setw(11) << "totalTris"
              << std::setw(14) << "trueBrute_ns"
              << std::setw(13) << "meshBVH_ns"
              << std::setw(13) << "sceneBVH_ns"
              << std::setw(10) << "K@1ms"
              << std::setw(11) << "K@5ms" << '\n';

    for (int n : Ns)
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> xd(2.0, kLen - 2.0), sd(0.25, 0.75);

        auto statics = staticCorridor();
        std::vector<std::shared_ptr<TriangleMeshGeometry>> meshes;
        std::vector<ActiveObstacle> obstacles;
        meshes.reserve(n);
        obstacles.reserve(n);

        for (int i = 0; i < n; ++i)
        {
            const double size = sd(rng), half = size * 0.5;
            std::uniform_real_distribution<double> yd(-kHalf + half, kHalf - half);
            const Vec3 c(xd(rng), yd(rng), half);
            auto mesh = makeUVSphere(c, half, stacks, slices, 10 + i);
            meshes.push_back(mesh);
            ActiveObstacle o;
            o.center = c;
            o.size = size;
            o.object_id = 10 + i;
            o.bounds = mesh->bounds();
            o.geometry = mesh;
            obstacles.push_back(o);
        }

        SceneBVH bvh;
        bvh.rebuild(statics, obstacles);

        auto true_brute = [&](const Ray &r) {
            Hit c;
            for (auto &s : statics)
            {
                Hit h = s->intersect(r);
                if (h.hit && h.t <= kMaxRange && (!c.hit || h.t < c.t))
                    c = h;
            }
            for (auto &m : meshes)
            {
                Hit h = m->bruteForceIntersect(r);
                if (h.hit && h.t <= kMaxRange && (!c.hit || h.t < c.t))
                    c = h;
            }
            return c;
        };
        auto mesh_bvh = [&](const Ray &r) {
            Hit c;
            for (auto &s : statics)
            {
                Hit h = s->intersect(r);
                if (h.hit && h.t <= kMaxRange && (!c.hit || h.t < c.t))
                    c = h;
            }
            for (auto &m : meshes)
            {
                Hit h = m->intersect(r);
                if (h.hit && h.t <= kMaxRange && (!c.hit || h.t < c.t))
                    c = h;
            }
            return c;
        };
        auto scene_bvh = [&](const Ray &r) { return bvh.intersect(r, kMaxRange); };

        // warmup
        timePass(rays, 1, true_brute);
        timePass(rays, 1, mesh_bvh);
        timePass(rays, 1, scene_bvh);

        const double tb = timePass(rays, passes, true_brute) * 1e6 / rays.size();
        const double mb = timePass(rays, passes, mesh_bvh) * 1e6 / rays.size();
        const double sb = timePass(rays, passes, scene_bvh) * 1e6 / rays.size();

        const double k1 = 1e6 / tb;  // rays affordable by true-brute at 1ms
        const double k5 = 5e6 / tb;  // at 5ms

        std::cout << std::right << std::setw(6) << n
                  << std::setw(11) << n * tris
                  << std::fixed << std::setprecision(1)
                  << std::setw(14) << tb
                  << std::setw(13) << mb
                  << std::setw(13) << sb
                  << std::setw(10) << std::setprecision(0) << k1
                  << std::setw(11) << k5 << '\n';
    }

    std::cout << "\nK = rays a mode can afford in the budget over the 180-deg arc"
              << " (K=23 -> 8 deg/ray, K=180 -> 1 deg/ray)\n";
    return 0;
}
