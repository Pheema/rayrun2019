#include "visualDebugger.h"

#define NOMINMAX
#define RDB_IMPLIMATATION
#include "rdb.hpp"

void
VisualDebugger::DrawPoint(const Vector3f& pos, int32_t group)
{
    DrawPoint(pos, Vector3f::One(), group);
}

void
VisualDebugger::DrawPoint(const Vector3f& pos,
                          const Vector3f& color,
                          int32_t group)
{
    rdbPoint(pos.x, pos.y, pos.z, color.x, color.y, color.z, group);
}

void
VisualDebugger::DrawLine(const Vector3f& v0, const Vector3f& v1, int32_t group)
{
    DrawLine(v0, v1, Vector3f::One(), group);
}

void
VisualDebugger::DrawLine(const Vector3f& v0,
                         const Vector3f& v1,
                         const Vector3f& color,
                         int32_t group)
{
    DrawLine(v0, v1, color, color, group);
}

void
VisualDebugger::DrawLine(const Vector3f& v0,
                         const Vector3f& v1,
                         const Vector3f& color0,
                         const Vector3f& color1,
                         int32_t group)
{
    rdbLine(v0.x,
            v0.y,
            v0.z,
            v1.x,
            v1.y,
            v1.z,
            color0.x,
            color0.y,
            color0.z,
            color1.x,
            color1.y,
            color1.z,
            group);
}

void
VisualDebugger::DrawTriangle(const Vector3f& v0,
                             const Vector3f& v1,
                             const Vector3f v2,
                             int32_t group)
{
    DrawTriangle(v0, v1, v2, Vector3f::One(), group);
}

void
VisualDebugger::DrawTriangle(const Vector3f& v0,
                             const Vector3f& v1,
                             const Vector3f v2,
                             const Vector3f& color,
                             int32_t group)
{
    rdbTriangle(v0.x,
                v0.y,
                v0.z,
                v1.x,
                v1.y,
                v1.z,
                v2.x,
                v2.y,
                v2.z,
                color.x,
                color.y,
                color.z,
                group);
}

void
VisualDebugger::DrawCube(const Vector3f& lower,
                         const Vector3f& upper,
                         int32_t group)
{
    DrawCube(lower, upper, Vector3f::One(), group);
}

void
VisualDebugger::DrawCube(const Vector3f& lower,
                         const Vector3f& upper,
                         const Vector3f& color,
                         int32_t group)
{
    const auto v000 = Vector3f(lower.x, lower.y, lower.z);
    const auto v100 = Vector3f(upper.x, lower.y, lower.z);
    const auto v010 = Vector3f(lower.x, upper.y, lower.z);
    const auto v110 = Vector3f(upper.x, upper.y, lower.z);
    const auto v001 = Vector3f(lower.x, lower.y, upper.z);
    const auto v101 = Vector3f(upper.x, lower.y, upper.z);
    const auto v011 = Vector3f(lower.x, upper.y, upper.z);
    const auto v111 = Vector3f(upper.x, upper.y, upper.z);

    DrawLine(v000, v100, color, group);
    DrawLine(v010, v110, color, group);
    DrawLine(v001, v101, color, group);
    DrawLine(v011, v111, color, group);

    DrawLine(v000, v010, color, group);
    DrawLine(v100, v110, color, group);
    DrawLine(v001, v011, color, group);
    DrawLine(v101, v111, color, group);

    DrawLine(v000, v001, color, group);
    DrawLine(v100, v101, color, group);
    DrawLine(v010, v011, color, group);
    DrawLine(v110, v111, color, group);
}

void
VisualDebugger::DrawRay(const Vector3f& o,
                        const Vector3f& dir,
                        const Vector3f& color,
                        int32_t group)
{
    DrawLine(o, o + 1e3f * dir, Vector3f::Zero(), color, group);
}
