#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <vector>
#include <cmath>

using Vec3 = Eigen::Vector3d;
using AABB = Eigen::AlignedBox3d;

namespace geom
{
    inline constexpr double Epsilon = 1e-9;
    inline constexpr double pi = 3.14159265358979323846;
    inline constexpr double deg = pi / 180.0;
}

struct Ray
{
    Vec3 ori;
    Vec3 dir;
};

struct Hit
{
    bool hit = false;
    double t = -1.0;
    int objId = -1;
};

struct Primitive
{
    Vec3 v0;
    Vec3 v1;
    Vec3 v2;
    int objId;

    AABB bounds() const // bbox that encloses triangle vertices
    {
        AABB bbox;
        bbox.setEmpty();

        bbox.extend(v0); // grow the box to contain all triangle corners
        bbox.extend(v1);
        bbox.extend(v2);

        return bbox;
    }

    Vec3 centroid() const
    {
        return (v0 + v1 + v2) / 3.0;
    }
};

struct BVHNode
{
    AABB bbox;
    int left = -1; // child
    int right = -1;
    int start = 0; // each leaf stores a range from the original primitive array
    int count = 0; // simple ownership, less memory

    bool isLeaf() const { return count > 0; }
};

struct AABBHit
{
    bool hit = false;
    double tmin = 0.0;
    double tmax = 0.0;
};

/**
 * BASE GEOMETRY CLASS
 * explicitly define virtual destructor whenever a base class uses virtual functions
 * if not, derived objects will not be deleted when the base class destructor is called
 * virtual destructor ensures deletion looks up derived destructors in vtable
 */
class Geometry
{
public:
    virtual ~Geometry() = default;
    virtual Hit intersect(const Ray &ray) const = 0; // "= 0" -> pure virtual function, derived classes MUST override
    virtual AABB bounds() const
    {
        AABB bbox;
        bbox.setEmpty();
        return bbox;
    }
};

/**
 * DERIVED GEOMETRY CLASSES: Sphere, Plane, Triangle Mesh
 */
class SphereGeometry : public Geometry
{
public:
    SphereGeometry(Vec3 center, double radius, int objId)
        : center_(center), radius_(radius), objId_(objId) {}

    Hit intersect(const Ray &ray) const override
    {
        Vec3 v = ray.ori - center_;

        double a = ray.dir.dot(ray.dir);
        double b = 2.0 * v.dot(ray.dir);
        double c = v.dot(v) - (radius_ * radius_);

        double delta = (b * b) - 4.0 * a * c;
        if (delta < 0.0)
            return Hit{};

        double t = ((-1.0 * b) - sqrt(delta)) / (2.0 * a);
        if (t <= geom::Epsilon)
            return Hit{};

        return Hit{true, t, objId_};
    }

    AABB bounds() const override
    {
        AABB bbox;
        bbox.extend(center_ - Vec3::Constant(radius_));
        bbox.extend(center_ + Vec3::Constant(radius_));
        return bbox;
    }

private:
    Vec3 center_;
    double radius_;
    int objId_;
};

class PlaneGeometry : public Geometry
{
public:
    PlaneGeometry(Vec3 normal, Vec3 point, int objId)
        : normal_(normal), point_(point), objId_(objId)
        {
            normal_.normalize();
        };

    Hit intersect(const Ray &ray) const override
    {
        double denominator = ray.dir.dot(normal_);
        if (std::abs(denominator) < geom::Epsilon)
            return Hit{};

        double t = normal_.dot(point_ - ray.ori) / denominator;
        if (t <= geom::Epsilon)
            return Hit{};

        return Hit{true, t, objId_};
    }

private:
    Vec3 normal_;
    Vec3 point_;
    int objId_;
};

class TriangleMeshGeometry : public Geometry
{
public:
    TriangleMeshGeometry(const std::vector<Vec3> &vertices, const std::vector<Eigen::Vector3i> &triangles, int objId)
        : vertices_(vertices), triangles_(triangles), objId_(objId)
    {
        buildPrimitives();
        rootNodeIndex_ = primitives_.empty()
                             ? -1
                             : buildBVH(0, static_cast<int>(primitives_.size()));
    }

    Hit intersect(const Ray &ray) const override
    {
        Hit closest;

        if (rootNodeIndex_ < 0)
            return closest;

        hit(ray, rootNodeIndex_, closest);

        return closest;
    }

    AABB bounds() const override
    {
        AABB bbox;
        bbox.setEmpty();

        for (const Vec3 &vertex : vertices_)
            bbox.extend(vertex);

        return bbox;
    }


    Hit bruteForceIntersect(const Ray &ray) const
    {
        Hit closest;

        for (const Primitive &p : primitives_)
        {
            Hit h = intersectTriangle(ray, p);

            if (h.hit && (!closest.hit || h.t < closest.t))
                closest = h;
        }

        return closest;
    }

private:
    void buildPrimitives()
    {
        primitives_.clear();
        primitives_.reserve(triangles_.size());

        for (const Eigen::Vector3i &tri : triangles_)
        {
            Primitive p;
            p.v0 = vertices_[tri[0]];
            p.v1 = vertices_[tri[1]];
            p.v2 = vertices_[tri[2]];
            p.objId = objId_;

            primitives_.push_back(p);
        }
    }

    /**
     * build BVH using SAH and bucketing
     * TODO: Your BVH is now binned SAH-ish, but it does not include traversal/intersection costs 
     * or compare split cost against leaf cost. Fine for now, just worth knowing.
     */
    int buildBVH(int start, int count)
    {
        constexpr int LeafSize = 4;
        constexpr int BucketCount = 32;

        int nodeIndex = static_cast<int>(bvh_.size());
        bvh_.push_back(BVHNode{});
        bvh_[nodeIndex].bbox.setEmpty();

        for (int i = start; i < start + count; ++i)
            bvh_[nodeIndex].bbox.extend(primitives_[i].bounds());

        if (count <= LeafSize)
        {
            bvh_[nodeIndex].start = start;
            bvh_[nodeIndex].count = count;
            return nodeIndex;
        }

        AABB centroidBox;
        centroidBox.setEmpty();
        for (int i = start; i < start + count; ++i)
            centroidBox.extend(primitives_[i].centroid());

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
            const double minCentroid = centroidBox.min()[axis];
            const double maxCentroid = centroidBox.max()[axis];
            const double extent = maxCentroid - minCentroid;

            if (extent <= geom::Epsilon)
                continue;

            std::array<Bucket, BucketCount> buckets;
            for (int i = start; i < start + count; ++i)
            {
                int bucketIndex = static_cast<int>(
                    BucketCount * (primitives_[i].centroid()[axis] - minCentroid) / extent);
                bucketIndex = std::clamp(bucketIndex, 0, BucketCount - 1);

                buckets[bucketIndex].bbox.extend(primitives_[i].bounds());
                ++buckets[bucketIndex].count;
            }

            std::array<AABB, BucketCount - 1> leftBBoxes;
            std::array<AABB, BucketCount - 1> rightBBoxes;
            std::array<int, BucketCount - 1> leftCounts{};
            std::array<int, BucketCount - 1> rightCounts{};

            AABB leftBBox;
            leftBBox.setEmpty();
            int leftCount = 0;
            for (int split = 0; split < BucketCount - 1; ++split)
            {
                leftBBox.extend(buckets[split].bbox);
                leftCount += buckets[split].count;

                leftBBoxes[split] = leftBBox;
                leftCounts[split] = leftCount;
            }

            AABB rightBBox;
            rightBBox.setEmpty();
            int rightCount = 0;
            for (int split = BucketCount - 2; split >= 0; --split)
            {
                rightBBox.extend(buckets[split + 1].bbox);
                rightCount += buckets[split + 1].count;

                rightBBoxes[split] = rightBBox;
                rightCounts[split] = rightCount;
            }

            for (int split = 0; split < BucketCount - 1; ++split)
            {
                if (leftCounts[split] == 0 || rightCounts[split] == 0)
                    continue;

                const double cost =
                    surfaceArea(leftBBoxes[split]) * leftCounts[split] +
                    surfaceArea(rightBBoxes[split]) * rightCounts[split];

                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = split;
                }
            }
        }

        if (bestAxis < 0)
        {
            bvh_[nodeIndex].start = start;
            bvh_[nodeIndex].count = count;
            return nodeIndex;
        }

        const double minCentroid = centroidBox.min()[bestAxis];
        const double maxCentroid = centroidBox.max()[bestAxis];
        const double extent = maxCentroid - minCentroid;

        auto mid = std::partition(
            primitives_.begin() + start,
            primitives_.begin() + start + count,
            [=](const Primitive &primitive)
            {
                int bucketIndex = static_cast<int>(
                    BucketCount * (primitive.centroid()[bestAxis] - minCentroid) / extent);
                bucketIndex = std::clamp(bucketIndex, 0, BucketCount - 1);
                return bucketIndex <= bestSplit;
            });

        int leftCount = static_cast<int>(mid - (primitives_.begin() + start));
        if (leftCount == 0 || leftCount == count)
        {
            std::sort(
                primitives_.begin() + start,
                primitives_.begin() + start + count,
                [bestAxis](const Primitive &a, const Primitive &b)
                {
                    return a.centroid()[bestAxis] < b.centroid()[bestAxis];
                });

            leftCount = count / 2;
        }

        const int left = buildBVH(start, leftCount);
        const int right = buildBVH(start + leftCount, count - leftCount);

        bvh_[nodeIndex].left = left;
        bvh_[nodeIndex].right = right;

        return nodeIndex;
    }

    static double surfaceArea(const AABB &bbox)
    {
        const Vec3 size = bbox.sizes();
        return 2.0 * (size.x() * size.y() + size.x() * size.z() + size.y() * size.z());
    }

    /**
     * ray-bbox interesection using kay's slab method
     * "should i enter this BVH node?"
     */
    AABBHit intersectAABB(const Ray &ray, const AABB &bbox) const
    {
        double tmin = -std::numeric_limits<double>::infinity();
        double tmax = std::numeric_limits<double>::infinity();

        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(ray.dir[axis]) < geom::Epsilon)
            {
                if (ray.ori[axis] < bbox.min()[axis] ||
                    ray.ori[axis] > bbox.max()[axis])
                    return AABBHit{};

                continue;
            }

            double t1 = (bbox.min()[axis] - ray.ori[axis]) / ray.dir[axis];
            double t2 = (bbox.max()[axis] - ray.ori[axis]) / ray.dir[axis];

            tmin = std::max(tmin, std::min(t1, t2));
            tmax = std::min(tmax, std::max(t1, t2));

            if (tmin > tmax)
                return AABBHit{};
        }

        if (tmax <= geom::Epsilon)
            return AABBHit{};

        return AABBHit{true, tmin, tmax};
    }

    /**
     * BVH TRAVERSAL
     * find the closest triangle hit
     */
    void hit(const Ray &ray, int nodeIndex, Hit &closest) const
    {
        const BVHNode &node = bvh_[nodeIndex];

        AABBHit bboxHit = intersectAABB(ray, node.bbox);
        if (!bboxHit.hit)
            return;

        // box cannot contain a better hit, skip
        if (closest.hit && bboxHit.tmin > closest.t)
            return;

        // once we reach a leaf, record hits
        if (node.isLeaf())
        {
            for (int i = node.start; i < node.start + node.count; ++i)
            {
                Hit h = intersectTriangle(ray, primitives_[i]);

                if (h.hit && (!closest.hit || h.t < closest.t))
                {
                    closest = h;
                }
            }

            return;
        }

        AABBHit left = intersectAABB(ray, bvh_[node.left].bbox);
        AABBHit right = intersectAABB(ray, bvh_[node.right].bbox);

        // OPTIMIZATION
        // before recursing on both children, make sure the second child
        // can actually be closer than the first child
        if (left.hit && right.hit)
        {
            int first = left.tmin <= right.tmin ? node.left : node.right;
            int second = left.tmin <= right.tmin ? node.right : node.left;

            double secondTmin = left.tmin <= right.tmin ? right.tmin : left.tmin;

            hit(ray, first, closest);

            if (!closest.hit || secondTmin < closest.t)
                hit(ray, second, closest);
        }
        else if (left.hit)
        {
            if (!closest.hit || left.tmin < closest.t)
                hit(ray, node.left, closest);
        }
        else if (right.hit)
        {
            if (!closest.hit || right.tmin < closest.t)
                hit(ray, node.right, closest);
        }
        else
        {
            return;
        }
    }

    /**
     * ray-triangle intersection using moller-trumbore
     * produce the final Hit
     */
    Hit intersectTriangle(const Ray &ray, const Primitive &p) const
    {
        const Vec3 edge1 = p.v1 - p.v0;
        const Vec3 edge2 = p.v2 - p.v0;

        const Vec3 rayCrossEdge2 = ray.dir.cross(edge2);
        const double det = edge1.dot(rayCrossEdge2);

        if (std::abs(det) < geom::Epsilon)
            return Hit{};

        const double invDet = 1.0 / det;
        const Vec3 s = ray.ori - p.v0;
        const double u = invDet * s.dot(rayCrossEdge2);

        if (u < 0.0 || u > 1.0)
            return Hit{};

        const Vec3 sCrossEdge1 = s.cross(edge1);
        const double v = invDet * ray.dir.dot(sCrossEdge1);

        if (v < 0.0 || u + v > 1.0)
            return Hit{};

        const double t = invDet * edge2.dot(sCrossEdge1);
        if (t <= geom::Epsilon)
            return Hit{};

        return Hit{true, t, p.objId};
    }

    std::vector<Vec3> vertices_;
    std::vector<Eigen::Vector3i> triangles_;
    std::vector<BVHNode> bvh_;
    std::vector<Primitive> primitives_;
    int rootNodeIndex_ = -1;
    int objId_ = -1;
};
