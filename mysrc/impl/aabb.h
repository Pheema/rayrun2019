#pragma once

#include "vector3f.h"
#include <algorithm>
#include <limits>

struct AABB
{
    AABB() = default;

    AABB(const Vector3f& lower, const Vector3f& upper)
      : lower(lower)
      , upper(upper)
    {
    }

    Vector3f
    GetCentroid() const
    {
        return 0.5f * (lower + upper);
    }

    void
    Merge(const AABB& aabb)
    {
        lower = MinElements(lower, aabb.lower);
        upper = MaxElements(upper, aabb.upper);
    }

    void
    Merge(const Vector3f& point)
    {
        lower = MinElements(lower, point);
        upper = MaxElements(upper, point);
    }

    bool
    Contains(const Vector3f& point) const
    {
        // clang-format off
        if (lower.x < point.x && point.x < upper.x &&
            lower.y < point.y && point.y < upper.y &&
            lower.z < point.z && point.z < upper.z)
        {
            return true;
        }
        // clang-format on

        return false;
    }

    int
    GetWidestAxis() const
    {
        float widestWidth = 0.0f;
        int widestAxis = 0;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float width = upper[axis] - lower[axis];
            if (width > widestWidth)
            {
                widestWidth = width;
                widestAxis = axis;
            }
        }

        return widestAxis;
    }

    float
    GetSurfaceArea() const
    {
        const auto diff = upper - lower;
        return 2.0f * (diff.x * diff.y + diff.y * diff.z + diff.z * diff.x);
    }

    Vector3f lower = Vector3f::One() * std::numeric_limits<float>::max();
    Vector3f upper = Vector3f::One() * std::numeric_limits<float>::lowest();
};
