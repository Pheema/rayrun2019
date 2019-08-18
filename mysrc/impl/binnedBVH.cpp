#include "binnedBVH.h"

std::optional<BVHNodeHitInfo>
BinnedBVH::Node::Intersect(const RayInternal& ray) const
{
    const Vector3f invRayDir = Vector3f::One() / ray.dir;
    const Vector3f t0 = (m_bounds.lower - ray.o) * invRayDir;
    const Vector3f t1 = (m_bounds.upper - ray.o) * invRayDir;

    constexpr int idxArray[] = { 0, 1, 2, 0 };

    for (int cnt = 0; cnt < 3; cnt++)
    {
        const int axis0 = idxArray[cnt];
        const int axis1 = idxArray[cnt + 1];

        if (ray.dir[axis0] == 0 || ray.dir[axis1] == 0)
        {
            // 軸に平行なレイが入ってきた場合
            continue;
        }

        const float tMinMax = std::max(std::min(t0[axis0], t1[axis0]),
                                       std::min(t0[axis1], t1[axis1]));
        const float tMaxMin = std::min(std::max(t0[axis0], t1[axis0]),
                                       std::max(t0[axis1], t1[axis1]));
        if (tMaxMin < tMinMax)
        {
            return std::nullopt;
        }
    }

    float distance = std::numeric_limits<float>::max();
    for (int axis = 0; axis < 3; ++axis)
    {
        if (t0[axis] > 0.0f)
        {
            distance = std::min(distance, t0[axis]);
        }

        if (t1[axis] > 0.0f)
        {
            distance = std::min(distance, t1[axis]);
        }
    }

    const BVHNodeHitInfo hitInfo = [&] {
        BVHNodeHitInfo x;
        x.distance = distance;
        return x;
    }();
    return hitInfo;
}

void
BinnedBVH::Build(const std::vector<std::array<uint32_t, 3>> vertexIndices,
                 const std::vector<Vector3f>& vertexPositions)
{
    // #TODO: 実装
}

std::optional<Ray>
BinnedBVH::Intersect(Ray* rays, size_t numRay, bool hitany)
{
    // #TODO: 実装
    return std::nullopt;
}
