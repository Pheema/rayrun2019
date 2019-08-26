#include "scene.h"

#include "intersection.h"
#include "visualDebugger.h"

void
Scene::Build(const float* vertices,
             size_t numVerts,
             const float* normals,
             size_t numNormals,
             const uint32_t* indices,
             size_t numFaces)
{
    VisualDebuggerEx visualDebugger;

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

    /* for (const auto& vertex : m_vertexPositions)
     {
         visualDebugger.DrawPoint(vertex);
     }
 */
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
        r.invDir = Vector3f::One() / r.dir;
        r.sign[0] = (r.invDir[0] < 0.0f);
        r.sign[1] = (r.invDir[1] < 0.0f);
        r.sign[2] = (r.invDir[2] < 0.0f);
        return r;
    }();

    const auto hitInfo =
      m_accel.Intersect(rayInternal, *this, ray.tnear, ray.tfar);
    if (hitInfo)
    {
        ray.isisect = true;

        const auto pos =
          rayInternal.o + rayInternal.dir.Normalized() * hitInfo->distance;
        ray.isect[0] = pos.x;
        ray.isect[1] = pos.y;
        ray.isect[2] = pos.z;
        ray.faceid = hitInfo->faceIndex;
        {
            const auto vertexIndices =
              m_vertexIndicesInFace[hitInfo->faceIndex];

            if (!m_vertexNormals.empty())
            {
                const auto normalIndices =
                  m_normalIndicesInFace[hitInfo->faceIndex];

                const auto shadingNormal =
                  CalcShadingNormal(rayInternal,
                                    m_vertexPositions[vertexIndices[0]],
                                    m_vertexPositions[vertexIndices[1]],
                                    m_vertexPositions[vertexIndices[2]],
                                    m_vertexNormals[normalIndices[0]],
                                    m_vertexNormals[normalIndices[1]],
                                    m_vertexNormals[normalIndices[2]]);

                const float sign = 1.0f;
                // std::copysign(1.0f, -Dot(rayInternal.dir, shadingNormal));
                ray.ns[0] = shadingNormal.x * sign;
                ray.ns[1] = shadingNormal.y * sign;
                ray.ns[2] = shadingNormal.z * sign;

                // VisualDebuggerEx visualDebugger;
                // visualDebugger.DrawPoint(pos);
            }
            else
            {
                const auto faceNormal =
                  CalcFaceNormal(m_vertexPositions[vertexIndices[0]],
                                 m_vertexPositions[vertexIndices[1]],
                                 m_vertexPositions[vertexIndices[2]]);

                const float sign =
                  std::copysign(1.0f, -Dot(rayInternal.dir, faceNormal));

                ray.ns[0] = faceNormal.x * sign;
                ray.ns[1] = faceNormal.y * sign;
                ray.ns[2] = faceNormal.z * sign;
            }
        }
        return;
    }
    else
    {
        ray.isisect = false;
        return;
    }
}
