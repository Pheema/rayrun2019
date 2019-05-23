#pragma once

#include "vector3f.h"
#include <algorithm>
#include <limits>

class AABB
{
public:
    constexpr AABB()
      : m_lower(Vector3f::Identity() * std::numeric_limits<float>::min())
      , m_upper(Vector3f::Identity() * std::numeric_limits<float>::max())
    {
    }

    constexpr AABB(const Vector3f& lower, const Vector3f& upper)
      : m_lower(lower)
      , m_upper(upper)
    {
    }

    Vector3f
    GetCentroid() const
    {
        return 0.5f * (m_lower + m_upper);
    }

    void
    Merge(const AABB& aabb)
    {
        m_lower = MinElements(m_lower, aabb.m_lower);
        m_upper = MaxElements(m_upper, aabb.m_upper);
    }

    void
    Merge(const Vector3f& point)
    {
        m_lower = MinElements(m_lower, point);
        m_upper = MaxElements(m_upper, point);
    }

    int
    GetWidestAxis() const
    {
        float widestWidth = 0.0f;
        int widestAxis = 0;

        for (int axis = 0; axis < 3; ++axis)
        {
            const float width = m_upper[axis] - m_lower[axis];
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
        const auto diff = m_upper - m_lower;
        return 2.0f * (diff.x * diff.y + diff.y * diff.z + diff.z * diff.x);
    }

private:
    Vector3f m_lower{};
    Vector3f m_upper{};
};
