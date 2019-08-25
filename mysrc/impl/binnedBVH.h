#pragma once

#include "aabb.h"
#include "hitInfo.h"
#include "rayInternal.h"
#include "rayrun.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

class Scene;

class BinnedBVH
{
private:
    //! BVHノード
    class BVHNode
    {

    public:
        BVHNode(const AABB& boundary, uint32_t indexBegin, uint32_t indexEnd)
          : m_boundary(boundary)
          , m_indexBegin(indexBegin)
          , m_indexEnd(indexEnd)
        {
        }

        BVHNode(const AABB& boundary,
                const std::array<uint32_t, 2>& childIndicies,
                uint32_t indexBegin,
                uint32_t indexEnd,
                bool isLeaf)
          : m_boundary(boundary)
          , m_childIndicies(childIndicies)
          , m_indexBegin(indexBegin)
          , m_indexEnd(indexEnd)
          , m_isLeaf(isLeaf)
        {
        }

        inline std::optional<SimpleHitInfo>
        Intersect(const RayInternal& ray) const
        {
            float tMin = ((ray.hasPositiveDir[0] ? m_boundary.lower.x
                                                 : m_boundary.upper.x) -
                          ray.o.x) *
                         ray.invDir.x;
            float tMax = ((ray.hasPositiveDir[0] ? m_boundary.upper.x
                                                 : m_boundary.lower.x) -
                          ray.o.x) *
                         ray.invDir.x;

            {
                const float tyMin =
                  ((ray.hasPositiveDir[1] ? m_boundary.lower.y
                                          : m_boundary.upper.y) -
                   ray.o.y) *
                  ray.invDir.y;
                const float tyMax =
                  ((ray.hasPositiveDir[1] ? m_boundary.upper.y
                                          : m_boundary.lower.y) -
                   ray.o.y) *
                  ray.invDir.y;

                if (tyMax < tMin || tMax < tyMin)
                {
                    return std::nullopt;
                }

                if (tMin < tyMin)
                {
                    tMin = tyMin;
                }

                if (tyMax < tMax)
                {
                    tMax = tyMax;
                }
            }

            {
                const float tzMin =
                  ((ray.hasPositiveDir[2] ? m_boundary.lower.z
                                          : m_boundary.upper.z) -
                   ray.o.z) *
                  ray.invDir.z;
                const float tzMax =
                  ((ray.hasPositiveDir[2] ? m_boundary.upper.z
                                          : m_boundary.lower.z) -
                   ray.o.z) *
                  ray.invDir.z;

                if (tzMax < tMin || tMax < tzMin)
                {
                    return std::nullopt;
                }

                if (tMin < tzMin)
                {
                    tMin = tzMin;
                }

                if (tzMax < tMax)
                {
                    tMax = tzMax;
                }
            }

            float distance = std::numeric_limits<float>::max();

            const bool isNearCollisionValid = (0.0f <= tMin && tMin < distance);
            if (isNearCollisionValid)
            {
                distance = tMin;
            }

            const bool isFarCollisionValid = (0.0f <= tMax && tMax < distance);
            if (isFarCollisionValid)
            {
                distance = tMax;
            }

            if (!(isNearCollisionValid || isFarCollisionValid))
            {
                return std::nullopt;
            }

            const SimpleHitInfo hitInfo = [&] {
                SimpleHitInfo h;
                h.distance = distance;
                return h;
            }();
            return hitInfo;
        }

        //! 子のノードインデックスを取得
        const std::array<uint32_t, 2>&
        GetChildIndices() const
        {
            return m_childIndicies;
        }

        //! 子のノードインデックスを設定
        void
        SetChildIndices(uint32_t leftChildIndex, uint32_t rightChildIndex)
        {
            m_childIndicies = { leftChildIndex, rightChildIndex };
        }

        void
        SetCentroidDiff(const Vector3f& v)
        {
            m_childCentroidDiffL2R = v;
        }

        bool
        IsLeftChildNodeNear(const Vector3f& dir) const
        {
            return Dot(m_childCentroidDiffL2R, dir) > 0;
        }

        //! 葉であるかどうかを設定する
        void
        SetLeaf(bool isLeaf)
        {
            m_isLeaf = isLeaf;
        }

        //! 葉であるかどうかを取得する
        bool
        IsLeaf() const
        {
            return m_isLeaf;
        }

        uint32_t
        GetIndexBegin() const
        {
            return m_indexBegin;
        }

        uint32_t
        GetIndexEnd() const
        {
            return m_indexEnd;
        }

        Vector3f
        GetCentroid() const
        {
            return m_boundary.GetCentroid();
        }

        const AABB&
        GetBoundary() const
        {
            return m_boundary;
        }

        bool
        Contains(Vector3f point) const
        {
            return m_boundary.Contains(point);
        }

    private:
        AABB m_boundary;
        std::array<uint32_t, 2> m_childIndicies{ 0, 0 };
        uint32_t m_indexBegin = 0;
        uint32_t m_indexEnd = 0;
        Vector3f m_childCentroidDiffL2R;
        bool m_isLeaf = false;
    };

    //! 各プリミティブが保持する前処理データ
    struct PrecomputedPrimitiveData
    {
        int32_t binID = -1;
        AABB aabb;
        Vector3f centroid;
    };

public:
    BinnedBVH() = default;

    //! BVHの構築
    void
    Build(const Scene& scene);

    //! レイとBVHの交差判定
    std::optional<HitInfo>
    Intersect(const RayInternal& ray,
              const Scene& scene,
              float distMin,
              float distMax) const;

private:
    //!
    //! @param binPartitionIndex どのビン番号でノードを左右に分割するか
    //! @return [コスト, 左のノードに含まれるプリミティブ数]
    std::pair<float, int>
    GetSAHCost(int binPartitionIndex, const BVHNode& currentNode) const;

private:
    constexpr static int kNumBins = 8; //!< ビンの分割数

    std::vector<PrecomputedPrimitiveData>
      m_precomputedFaceData; //!< 各プリミティブに対して事前計算されたデータ
    std::vector<BVHNode> m_bvhNodes;
    std::vector<uint32_t> m_faceIndicies;
};
