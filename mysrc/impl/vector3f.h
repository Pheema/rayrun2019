#pragma once

#include <cassert>

struct Vector3f
{
    Vector3f() = default;

    Vector3f(float x, float y, float z)
      : x(x)
      , y(y)
      , z(z)
    {
    }

    float operator[](int i)
    {
        assert(0 <= i && i < 3);
        return *(&x + i);
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

Vector3f
operator+(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}
