#pragma once

#include "hitInfo.h"
#include "rayInternal.h"
#include <optional>

inline std::optional<SimpleHitInfo>
IntersectRayTriangle(const Vector3f& v0,
                     const Vector3f& v1,
                     const Vector3f& v2,
                     const RayInternal& ray)
{
    const Vector3f e1 = v1 - v0;
    const Vector3f e2 = v2 - v0;

    const Vector3f crossEdges = Cross(e1, e2);
    const float invDet = -1.0f / Dot(ray.dir, crossEdges);

    const Vector3f vec = ray.o - v0;

    const float weightE1 = Dot(vec, Cross(ray.dir, e2)) * invDet;
    if (weightE1 < 0.0f || 1.0f <= weightE1)
    {
        return std::nullopt;
    }

    const float weightE2 = Dot(vec, Cross(e1, ray.dir)) * invDet;
    if (weightE2 < 0.0f || 1.0f <= weightE2)
    {
        return std::nullopt;
    }

    if (weightE1 + weightE2 >= 1.0f)
    {
        return std::nullopt;
    }

    const float distance = Dot(vec, crossEdges) * invDet;
    if (distance >= 0.0f)
    {
        return [&] {
            SimpleHitInfo h;
            h.distance = distance;
            return h;
        }();
    }
    else
    {
        return std::nullopt;
    }
}

inline Vector3f
CalcShadingNormal(Vector3f pos,
                  const Vector3f& v0,
                  const Vector3f& v1,
                  const Vector3f& v2,
                  const Vector3f& n0,
                  const Vector3f& n1,
                  const Vector3f& n2)
{
    const Vector3f p = pos - v0;
    const Vector3f e1 = v1 - v0;
    const Vector3f e2 = v2 - v0;
    const Vector3f crossEdges = Cross(e1, e2);

    const float inv = 1.0f / crossEdges.SquaredLength();
    const float weightE1 = Dot(crossEdges, Cross(e1, p)) * inv;
    const float weightE2 = Dot(crossEdges, Cross(p, e2)) * inv;

    const Vector3f shadingNormal =
      weightE1 * (n1 - n0) + weightE2 * (n2 - n0) + n0;

    return shadingNormal;
}
