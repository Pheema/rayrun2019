#include "scene.h"

#include "intersection.h"

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
    m_normalIndicesInFace.clear();
    m_normalIndicesInFace.resize(numFaces);
    for (size_t idxFace = 0; idxFace < numFaces; idxFace++)
    {
        const size_t offset = idxFace * 6;
        m_vertexIndicesInFace[idxFace] = { indices[offset + 0],
                                           indices[offset + 2],
                                           indices[offset + 4] };
        m_normalIndicesInFace[idxFace] = { indices[offset + 1],
                                           indices[offset + 3],
                                           indices[offset + 5] };
    }

    m_accel = BinnedBVH();
    m_accel.Build(*this);
}

void
Scene::Intersect(Ray& ray) const
{
    const RayInternal rayInternal = [&] {
        RayInternal r{};
        r.o = Vector3f(ray.pos[0], ray.pos[1], ray.pos[2]);
        r.dir = Vector3f(ray.dir[0], ray.dir[1], ray.dir[2]);

        if (ray.faceid >= 0)
        {
            r.lastFaceIndex = ray.faceid;
        }
        return r;
    }();

    const auto hitInfo =
      m_accel.Intersect(rayInternal, *this, ray.tnear, ray.tfar);
    if (hitInfo)
    {
        ray.isisect = true;

        const auto pos = rayInternal.o + rayInternal.dir * hitInfo->distance;
        ray.isect[0] = pos.x;
        ray.isect[1] = pos.y;
        ray.isect[2] = pos.z;

        {
            const auto vertexIndices =
              m_vertexIndicesInFace[hitInfo->faceIndex];
            const auto normalIndices =
              m_normalIndicesInFace[hitInfo->faceIndex];

            auto verties = m_vertexPositions[hitInfo->faceIndex];
            auto normals = m_vertexNormals[hitInfo->faceIndex];
            const auto shadingNormal =
              CalcShadingNormal(pos,
                                m_vertexPositions[vertexIndices[0]],
                                m_vertexPositions[vertexIndices[1]],
                                m_vertexPositions[vertexIndices[2]],
                                m_vertexNormals[normalIndices[0]],
                                m_vertexNormals[normalIndices[1]],
                                m_vertexNormals[normalIndices[2]]);

            ray.ns[0] = shadingNormal.x;
            ray.ns[1] = shadingNormal.y;
            ray.ns[2] = shadingNormal.z;
        }

        ray.faceid = hitInfo->faceIndex;

        return;
    }
    else
    {
        ray.isisect = false;
        return;
    }
}
