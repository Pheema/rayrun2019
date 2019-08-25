#pragma once

#include "vector3f.h"
#include <cstdint>
#include <optional>

struct RayInternal
{
    Vector3f o{};
    Vector3f dir{};
    Vector3f invDir{};
    int8_t sign[3] = {};
    std::optional<uint32_t> lastFaceIndex; //!< 一番最後に衝突した面番号
};
