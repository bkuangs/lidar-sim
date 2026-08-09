#pragma once
#include "obstacle.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <vector>

/**
 * SCENE-LEVEL (TOP-LEVEL) BVH
 *
 * Mirrors XBucketScene's interface so it can drop into the same query path:
 *   - rebuild(static_objects, obstacles)
 *   - intersect(ray, max_distance)
 *
 * The per-mesh BVH inside TriangleMeshGeometry still runs underneath this; this
 * structure only decides *which obstacles* a ray needs to test. Infinite static
 * geometry (floor/ceiling/walls -> PlaneGeometry) has no finite AABB, so it is
 * tested brute-force each ray, exactly like XBucketScene does.
 */
class SceneBVH
{
public:
    void rebuild(
        const std::vector<std::shared_ptr<Geometry>> &static_objects,
        const std::vector<ActiveObstacle> &obstacles)
    {
        static_objects_ = static_objects;
        nodes_.clear();
        prims_.clear();

        prims_.reserve(obstacles.size());
        for (const ActiveObstacle &obstacle : obstacles)
            prims_.push_back(Primitive{obstacle.bounds, obstacle.bounds.center(), obstacle.geometry});

        if (!prims_.empty())
            build(0, static_cast<int>(prims_.size()));
    }

    Hit intersect(const Ray &ray, double max_distance) const
    {
        Hit closest = intersectObjects(static_objects_, ray, max_distance);

        if (nodes_.empty())
            return closest;

        traverse(ray, 0, max_distance, closest);
        return closest;
    }

private:
    struct Primitive
    {
        AABB bounds;
        Vec3 centroid;
        std::shared_ptr<Geometry> geometry;
    };

    struct Node
    {
        AABB bbox;
        int left = -1;
        int right = -1;
        int start = 0;
        int count = 0;

        bool isLeaf() const { return count > 0; }
    };

    struct SlabHit
    {
        bool hit = false;
        double tmin = 0.0;
    };

    int build(int start, int count)
    {
        constexpr int LeafSize = 2;
        constexpr int BucketCount = 16;

        const int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{});
        nodes_[nodeIndex].bbox.setEmpty();
        for (int i = start; i < start + count; ++i)
            nodes_[nodeIndex].bbox.extend(prims_[i].bounds);

        if (count <= LeafSize)
        {
            nodes_[nodeIndex].start = start;
            nodes_[nodeIndex].count = count;
            return nodeIndex;
        }

        AABB centroidBox;
        centroidBox.setEmpty();
        for (int i = start; i < start + count; ++i)
            centroidBox.extend(prims_[i].centroid);

        struct Bucket
        {
            Bucket() { bbox.setEmpty(); }
            AABB bbox;
            int count = 0;
        };

        double bestCost = std::numeric_limits<double>::infinity();
        int bestAxis = -1;
        int bestSplit = -1;

        for (int axis = 0; axis < 3; ++axis)
        {
            const double minC = centroidBox.min()[axis];
            const double extent = centroidBox.max()[axis] - minC;
            if (extent <= geom::Epsilon)
                continue;

            std::array<Bucket, BucketCount> buckets;
            for (int i = start; i < start + count; ++i)
            {
                int b = static_cast<int>(BucketCount * (prims_[i].centroid[axis] - minC) / extent);
                b = std::clamp(b, 0, BucketCount - 1);
                buckets[b].bbox.extend(prims_[i].bounds);
                ++buckets[b].count;
            }

            std::array<double, BucketCount - 1> leftArea{};
            std::array<double, BucketCount - 1> rightArea{};
            std::array<int, BucketCount - 1> leftCounts{};
            std::array<int, BucketCount - 1> rightCounts{};

            AABB leftBox;
            leftBox.setEmpty();
            int leftCount = 0;
            for (int s = 0; s < BucketCount - 1; ++s)
            {
                leftBox.extend(buckets[s].bbox);
                leftCount += buckets[s].count;
                leftArea[s] = surfaceArea(leftBox);
                leftCounts[s] = leftCount;
            }

            AABB rightBox;
            rightBox.setEmpty();
            int rightCount = 0;
            for (int s = BucketCount - 2; s >= 0; --s)
            {
                rightBox.extend(buckets[s + 1].bbox);
                rightCount += buckets[s + 1].count;
                rightArea[s] = surfaceArea(rightBox);
                rightCounts[s] = rightCount;
            }

            for (int s = 0; s < BucketCount - 1; ++s)
            {
                if (leftCounts[s] == 0 || rightCounts[s] == 0)
                    continue;
                const double cost = leftArea[s] * leftCounts[s] + rightArea[s] * rightCounts[s];
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = s;
                }
            }
        }

        if (bestAxis < 0)
        {
            nodes_[nodeIndex].start = start;
            nodes_[nodeIndex].count = count;
            return nodeIndex;
        }

        const double minC = centroidBox.min()[bestAxis];
        const double extent = centroidBox.max()[bestAxis] - minC;
        auto mid = std::partition(
            prims_.begin() + start,
            prims_.begin() + start + count,
            [=](const Primitive &p)
            {
                int b = static_cast<int>(BucketCount * (p.centroid[bestAxis] - minC) / extent);
                b = std::clamp(b, 0, BucketCount - 1);
                return b <= bestSplit;
            });

        int leftCount = static_cast<int>(mid - (prims_.begin() + start));
        if (leftCount == 0 || leftCount == count)
        {
            std::sort(
                prims_.begin() + start,
                prims_.begin() + start + count,
                [bestAxis](const Primitive &a, const Primitive &b)
                { return a.centroid[bestAxis] < b.centroid[bestAxis]; });
            leftCount = count / 2;
        }

        const int left = build(start, leftCount);
        const int right = build(start + leftCount, count - leftCount);
        nodes_[nodeIndex].left = left;
        nodes_[nodeIndex].right = right;
        return nodeIndex;
    }

    void traverse(const Ray &ray, int nodeIndex, double max_distance, Hit &closest) const
    {
        const Node &node = nodes_[nodeIndex];

        const SlabHit box = intersectAABB(ray, node.bbox, max_distance);
        if (!box.hit)
            return;
        if (closest.hit && box.tmin > closest.t)
            return;

        if (node.isLeaf())
        {
            for (int i = node.start; i < node.start + node.count; ++i)
            {
                Hit h = prims_[i].geometry->intersect(ray);
                if (h.hit && h.t <= max_distance && (!closest.hit || h.t < closest.t))
                    closest = h;
            }
            return;
        }

        const SlabHit left = intersectAABB(ray, nodes_[node.left].bbox, max_distance);
        const SlabHit right = intersectAABB(ray, nodes_[node.right].bbox, max_distance);

        if (left.hit && right.hit)
        {
            const int first = left.tmin <= right.tmin ? node.left : node.right;
            const int second = left.tmin <= right.tmin ? node.right : node.left;
            const double secondTmin = std::max(left.tmin, right.tmin);

            traverse(ray, first, max_distance, closest);
            if (!closest.hit || secondTmin < closest.t)
                traverse(ray, second, max_distance, closest);
        }
        else if (left.hit)
        {
            traverse(ray, node.left, max_distance, closest);
        }
        else if (right.hit)
        {
            traverse(ray, node.right, max_distance, closest);
        }
    }

    static SlabHit intersectAABB(const Ray &ray, const AABB &bbox, double max_distance)
    {
        double tmin = -std::numeric_limits<double>::infinity();
        double tmax = std::numeric_limits<double>::infinity();

        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(ray.dir[axis]) < geom::Epsilon)
            {
                if (ray.ori[axis] < bbox.min()[axis] || ray.ori[axis] > bbox.max()[axis])
                    return SlabHit{};
                continue;
            }
            double t1 = (bbox.min()[axis] - ray.ori[axis]) / ray.dir[axis];
            double t2 = (bbox.max()[axis] - ray.ori[axis]) / ray.dir[axis];
            tmin = std::max(tmin, std::min(t1, t2));
            tmax = std::min(tmax, std::max(t1, t2));
            if (tmin > tmax)
                return SlabHit{};
        }

        if (tmax <= geom::Epsilon || tmin > max_distance)
            return SlabHit{};

        return SlabHit{true, std::max(tmin, 0.0)};
    }

    static double surfaceArea(const AABB &bbox)
    {
        const Vec3 s = bbox.sizes();
        return 2.0 * (s.x() * s.y() + s.x() * s.z() + s.y() * s.z());
    }

    static Hit intersectObjects(
        const std::vector<std::shared_ptr<Geometry>> &objects,
        const Ray &ray,
        double max_distance)
    {
        Hit closest;
        for (const auto &object : objects)
        {
            Hit hit = object->intersect(ray);
            if (hit.hit && hit.t <= max_distance && (!closest.hit || hit.t < closest.t))
                closest = hit;
        }
        return closest;
    }

    std::vector<std::shared_ptr<Geometry>> static_objects_;
    std::vector<Node> nodes_;
    std::vector<Primitive> prims_;
};
