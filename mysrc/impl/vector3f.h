#pragma once

#include <algorithm>
#include <cassert>

struct Vector3f
{
    constexpr Vector3f()
      : x(0.0f)
      , y(0.0f)
      , z(0.0f)
    {
    }

    constexpr Vector3f(float x, float y, float z)
      : x(x)
      , y(y)
      , z(z)
    {
    }

    static constexpr Vector3f
    One()
    {
        return Vector3f(1.0f, 1.0f, 1.0f);
    }

    constexpr float operator[](int i) const
    {
        assert(0 <= i && i < 3);
        return *(&x + i);
    }

    constexpr float& operator[](int i) { return *(&x + i); }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

constexpr Vector3f
operator+(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}

constexpr Vector3f
operator-(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

constexpr Vector3f operator*(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z);
}

constexpr Vector3f operator*(const Vector3f& v, float s)
{
    return Vector3f(s * v.x, s * v.y, s * v.z);
}

constexpr Vector3f operator*(float s, const Vector3f& v)
{
    return v * s;
}

constexpr Vector3f
operator/(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z);
}

constexpr inline Vector3f
MinElements(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(
      std::min(v0.x, v1.x), std::min(v0.y, v1.y), std::min(v0.z, v1.z));
}

constexpr inline Vector3f
MaxElements(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(
      std::max(v0.x, v1.x), std::max(v0.x, v1.x), std::max(v0.z, v1.z));
}
