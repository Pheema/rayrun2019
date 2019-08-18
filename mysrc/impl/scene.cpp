#include "scene.h"

void
Scene::Build(const float* vertices,
             size_t numVerts,
             const float* normals,
             size_t numNormals,
             const uint32_t* indices,
             size_t numFaces)
{
    // 頂点
    m_vertexPositions.clear();
    m_vertexPositions.resize(numVerts);
    for (size_t idxVertex = 0; idxVertex < numVerts; idxVertex++)
    {
        const size_t offset = idxVertex * 3;
        m_vertexPositions[idxVertex] = Vector3f(
          vertices[offset + 0], vertices[offset + 1], vertices[offset + 2]);
    }

    // 法線
    m_vertexNormals.clear();
    m_vertexNormals.resize(numNormals);
    for (size_t idxNormal = 0; idxNormal < numNormals; idxNormal++)
    {
        const size_t offset = idxNormal * 3;
        m_vertexNormals[idxNormal] = Vector3f(
          normals[offset + 0], normals[offset + 1], normals[offset + 2]);
    }

    // 頂点インデックス
    m_vertexIndicesInFace.clear();
    m_vertexIndicesInFace.resize(numFaces);
    m_normalIndices.clear();
    m_normalIndices.resize(numFaces);
    for (size_t idxFace = 0; idxFace < numFaces; idxFace++)
    {
        const size_t offset = idxFace * 6;
        m_vertexIndicesInFace[idxFace] = { indices[offset + 0],
                                           indices[offset + 2],
                                           indices[offset + 4] };
        m_normalIndices[idxFace] = { indices[offset + 1],
                                     indices[offset + 3],
                                     indices[offset + 5] };
    }

    m_accel = BinnedBVH();
    m_accel.Build(*this);
}

std::optional<ShadingInfo>
Scene::Intersect(const Ray& ray)
{
    const RayInternal rayInternal = [&] {
        RayInternal r{};
        r.o = Vector3f(ray.pos[0], ray.pos[1], ray.pos[2]);
        r.dir = Vector3f(ray.dir[0], ray.dir[1], ray.dir[2]);
        return r;
    }();

    // #TODO: std::optional<ShadingInfo> を返す
    auto hitInfo = m_accel.Intersect(rayInternal, *this, ray.tnear, ray.tfar);
    if (!hitInfo)
    {
        return std::nullopt;
    }

    ShadingInfo shadingInfo;
    shadingInfo.position = rayInternal.o + rayInternal.dir * hitInfo->distance;
    return shadingInfo;
}
