#pragma once

#include "aabb.h"
#include "rayrun.hpp"
#include <array>
#include <cstdint>
#include <optional>

// #TODO: あとで別ファイルに移す
struct RayInternal
{
    Vector3f o;
    Vector3f dir;
};

//! BVHノードにヒットした際の情報
struct BVHNodeHitInfo
{
    float distance = std::numeric_limits<float>::max();
};

class BinnedBVH
{
private:
    //! BVHノード
    class Node
    {

    public:
        Node(const AABB& bounds,
             const std::array<int32_t, 2>& childIndicies,
             int32_t indexBegin,
             int32_t indexEnd,
             bool isLeaf)
          : m_bounds(bounds)
          , m_childIndicies(childIndicies)
          , m_indexBegin(indexBegin)
          , m_indexEnd(indexEnd)
          , m_isLeaf(isLeaf)
        {
        }

        std::optional<BVHNodeHitInfo>
        Intersect(const RayInternal& ray) const;

        //! 子のノードインデックスを取得
        const std::array<int, 2>&
        GetChildIndices() const
        {
            return m_childIndicies;
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

        int32_t
        GetIndexBegin() const
        {
            return m_indexBegin;
        }

        int32_t
        GetIndexEnd() const
        {
            return m_indexEnd;
        }

        Vector3f
        GetCentroid() const
        {
            return m_bounds.GetCentroid();
        }

    private:
        AABB m_bounds;
        std::array<int32_t, 2> m_childIndicies{ -1, -1 };
        int32_t m_indexBegin = -1;
        int32_t m_indexEnd = -1;
        bool m_isLeaf = false;
    };

    //! 各プリミティブが保持する前処理データ
    struct PrimitiveData
    {
        int32_t binID = -1;
        AABB aabb;
        Vector3f centroid;
    };

public:
    BinnedBVH() = default;

    //! BVHの構築
    void
    Build(const float* vertices,
          size_t numVerts,
          const float* normals,
          size_t numNormals,
          const uint32_t* indices,
          size_t numFace);

    //! レイとBVHの交差判定
    std::optional<Ray>
    Intersect(Ray* rays, size_t numRay, bool hitany);

private:
    constexpr static int kNumBins = 4; //!< ビンの分割数
};
