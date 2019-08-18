#pragma once

#include "binnedBVH.h"
#include "vector3f.h"
#include <array>
#include <cstdint>
#include <vector>

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

    std::optional<ShadingInfo>
    Intersect(const Ray& ray);

    size_t
    GetNumFaces() const
    {
        return m_vertexIndicesInFace.size();
    }

    std::array<std::reference_wrapper<const Vector3f>, 3>
    GetFaceVertices(uint32_t faceIndex) const
    {
        const auto indices = m_vertexIndicesInFace[faceIndex];

        return { std::ref(m_vertexPositions[indices[0]]),
                 std::ref(m_vertexPositions[indices[1]]),
                 std::ref(m_vertexPositions[indices[2]]) };
    }

private:
    std::vector<Vector3f> m_vertexPositions;
    std::vector<Vector3f> m_vertexNormals;
    std::vector<std::array<uint32_t, 3>>
      m_vertexIndicesInFace; // 面に対する各頂点のインデックス
    std::vector<std::array<uint32_t, 3>>
      m_normalIndices; // 面に対する各法線のインデックス
    BinnedBVH m_accel;
};
