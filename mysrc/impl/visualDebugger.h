#pragma once

#include "vector3f.h"
#include <cstdint>

class VisualDebugger
{
public:
    static void
    DrawPoint(const Vector3f& pos, int32_t group);

    static void
    DrawPoint(const Vector3f& pos, const Vector3f& color, int32_t group);

    static void
    DrawLine(const Vector3f& v0, const Vector3f& v1, int32_t group);

    static void
    DrawLine(const Vector3f& v0,
             const Vector3f& v1,
             const Vector3f& color,
             int32_t group);

    static void
    DrawLine(const Vector3f& v0,
             const Vector3f& v1,
             const Vector3f& color0,
             const Vector3f& color1,
             int32_t group);

    static void
    DrawTriangle(const Vector3f& v0,
                 const Vector3f& v1,
                 const Vector3f v2,
                 int32_t group);

    static void
    DrawTriangle(const Vector3f& v0,
                 const Vector3f& v1,
                 const Vector3f v2,
                 const Vector3f& color,
                 int32_t group);

    static void
    DrawCube(const Vector3f& lower, const Vector3f& upper, int32_t group);

    static void
    DrawCube(const Vector3f& lower,
             const Vector3f& upper,
             const Vector3f& color,
             int32_t group);

    static void
    DrawRay(const Vector3f& o,
            const Vector3f& dir,
            const Vector3f& color,
            int32_t group);
};

class VisualDebuggerEx
{
public:
    void
    DrawPoint(const Vector3f& pos)
    {
        VisualDebugger::DrawPoint(pos, GetColorFromGroup(m_group), m_group);
        m_group++;
    }

    void
    DrawLine(const Vector3f& v0, const Vector3f& v1)
    {
        VisualDebugger::DrawLine(v0, v1, GetColorFromGroup(m_group), m_group);
        m_group++;
    }

    void
    DrawTriangle(const Vector3f& v0, const Vector3f& v1, const Vector3f v2)
    {
        VisualDebugger::DrawTriangle(
          v0, v1, v2, GetColorFromGroup(m_group), m_group);
        m_group++;
    }

    void
    DrawCube(const Vector3f& lower, const Vector3f& upper)
    {
        VisualDebugger::DrawCube(
          lower, upper, GetColorFromGroup(m_group), m_group);
        m_group++;
    }

    void
    DrawRay(const Vector3f& o, const Vector3f& dir)
    {
        VisualDebugger::DrawRay(o, dir, GetColorFromGroup(m_group), m_group);
        m_group++;
    }

private:
    static Vector3f
    GetColorFromGroup(int32_t g)
    {
        float group = static_cast<float>(g);
        return 0.5f * (Vector3f::One() +
                       Vector3f(sinf(group), sinf(group + 1), sinf(group + 2)));
    }

private:
    int32_t m_group = 0;
};
