#pragma once
#include "geometry.hpp"
#include <cmath>
#include <memory>
#include <vector>

/**
 * Mesh generators shared by the benchmarks.
 *
 * Layer 2 obstacles are full-height pillars so a horizontal LiDAR slice reliably
 * detects them regardless of vertical resolution — isolating azimuth aliasing
 * as the only perception variable. High triangle counts make the true
 * triangle-soup brute baseline genuinely expensive (see DESIGN.md). `makeCube`
 * is the low-poly analytic obstacle used by the Layer 1 scaling scenes.
 */

// Axis-aligned cube of edge `size` centered at `center` (12 triangles).
inline std::shared_ptr<TriangleMeshGeometry> makeCube(
    const Vec3 &center, double size, int object_id)
{
    const double h = size * 0.5;
    std::vector<Vec3> v = {
        {center.x() - h, center.y() - h, center.z() - h},
        {center.x() + h, center.y() - h, center.z() - h},
        {center.x() + h, center.y() + h, center.z() - h},
        {center.x() - h, center.y() + h, center.z() - h},
        {center.x() - h, center.y() - h, center.z() + h},
        {center.x() + h, center.y() - h, center.z() + h},
        {center.x() + h, center.y() + h, center.z() + h},
        {center.x() - h, center.y() + h, center.z() + h},
    };
    std::vector<Eigen::Vector3i> t = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
        {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0},
    };
    return std::make_shared<TriangleMeshGeometry>(v, t, object_id);
}

// Tall cylindrical pillar from z=0 to z=height. Triangles ~= slices*stacks*2 + 2*slices.
inline std::shared_ptr<TriangleMeshGeometry> makePillar(
    double cx, double cy, double radius, double height,
    int slices, int stacks, int objId)
{
    std::vector<Vec3> v;
    std::vector<Eigen::Vector3i> t;

    // side ring vertices: (stacks+1) rings of `slices` vertices
    for (int i = 0; i <= stacks; ++i)
    {
        const double z = height * i / stacks;
        for (int j = 0; j < slices; ++j)
        {
            const double th = 2.0 * geom::pi * j / slices;
            v.push_back(Vec3(cx + radius * std::cos(th), cy + radius * std::sin(th), z));
        }
    }
    for (int i = 0; i < stacks; ++i)
    {
        const int row = i * slices, next = (i + 1) * slices;
        for (int j = 0; j < slices; ++j)
        {
            const int a = row + j, b = row + (j + 1) % slices;
            const int c = next + j, d = next + (j + 1) % slices;
            t.push_back({a, c, b});
            t.push_back({b, c, d});
        }
    }

    // caps
    const int bottom_center = static_cast<int>(v.size());
    v.push_back(Vec3(cx, cy, 0.0));
    const int top_center = static_cast<int>(v.size());
    v.push_back(Vec3(cx, cy, height));
    const int top_row = stacks * slices;
    for (int j = 0; j < slices; ++j)
    {
        const int a = j, b = (j + 1) % slices;
        t.push_back({bottom_center, b, a});
        t.push_back({top_center, top_row + a, top_row + b});
    }

    return std::make_shared<TriangleMeshGeometry>(v, t, objId);
}

// UV sphere (kept for probes / general use).
inline std::shared_ptr<TriangleMeshGeometry> makeUVSphere(
    const Vec3 &c, double r, int stacks, int slices, int objId)
{
    std::vector<Vec3> v;
    std::vector<Eigen::Vector3i> t;
    v.push_back(c + Vec3(0, 0, r));
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
