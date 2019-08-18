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

        std::optional<SimpleHitInfo>
        Intersect(const RayInternal& ray) const;

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
              float distMax);

private:
    //!
    //! @param binPartitionIndex どのビン番号でノードを左右に分割するか
    //! @return [コスト, 左のノードに含まれるプリミティブ数]
    std::pair<float, int>
    GetSAHCost(int binPartitionIndex, const BVHNode& currentNode) const;

private:
    constexpr static int kNumBins = 4; //!< ビンの分割数

    std::vector<PrecomputedPrimitiveData>
      m_precomputedFaceData; //!< 各プリミティブに対して事前計算されたデータ
    std::vector<BVHNode> m_bvhNodes;
    std::vector<uint32_t> m_faceIndicies;
};
