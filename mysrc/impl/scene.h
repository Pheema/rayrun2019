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

    void
    Intersect(Ray& ray) const;

    size_t
    GetNumFaces() const
    {
        return m_vertexIndicesInFace.size();
    }

    std::array<std::reference_wrapper<const Vector3f>, 3>
    GetFaceVertices(uint32_t faceIndex) const
    {
        const auto indices = m_vertexIndicesInFace[faceIndex];

        return { std::cref(m_vertexPositions[indices[0]]),
                 std::cref(m_vertexPositions[indices[1]]),
                 std::cref(m_vertexPositions[indices[2]]) };
    }

    inline const Vector3f&
    GetFaceVertice(uint32_t faceIndex, int i) const
    {
        assert(0 <= i && i < 3);
        const auto indices = m_vertexIndicesInFace[faceIndex];
        return m_vertexPositions[indices[i]];
    }

private:
    std::vector<Vector3f> m_vertexPositions;
    std::vector<Vector3f> m_vertexNormals;
    std::vector<std::array<uint32_t, 3>>
      m_vertexIndicesInFace; // 面に対する各頂点のインデックス
    std::vector<std::array<uint32_t, 3>>
      m_normalIndicesInFace; // 面に対する各法線のインデックス
    BinnedBVH m_accel;
};
