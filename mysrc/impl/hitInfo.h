#pragma once

#include "vector3f.h"

struct SimpleHitInfo
{
    float distance = std::numeric_limits<float>::max();
};

struct HitInfo
{
    float distance = std::numeric_limits<float>::max();
    uint32_t faceIndex = 0;
};

struct ShadingInfo
{
    Vector3f position; //!< 交差位置
    Vector3f normal;   //!< 交差位置での法線
};
