#pragma once

#include "binnedBVH.h"
#include "vector3f.h"
#include <array>
#include <cstdint>
#include <vector>

struct HitInfo
{
    uint32_t idxFace = 0;
};

class Scene
{
public:
    void
    Build(const float* vertices,
          size_t numVerts,
          const float* normals,
          size_t numNormals,
          const uint32_t* indices,
          size_t numFaces);

    std::optional<HitInfo>
    Intersect(const Ray& ray);

private:
    std::vector<Vector3f> m_vertexPositions;
    std::vector<Vector3f> m_vertexNormals;
    std::vector<std::array<uint32_t, 3>>
      m_vertexIndices; // 面に対する各頂点のインデックス
    std::vector<std::array<uint32_t, 3>>
      m_normalIndices; // 面に対する各法線のインデックス
    BinnedBVH m_accel;
};
