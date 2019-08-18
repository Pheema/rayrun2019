#pragma once

#include "vector3f.h"
#include <optional>

struct RayInternal
{
    Vector3f o{};
    Vector3f dir{};
    std::optional<uint32_t> lastFaceIndex; //!< 一番最後に衝突した面番号
};
