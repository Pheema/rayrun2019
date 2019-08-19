#include "binnedBVH.h"

#include "intersection.h"
#include "scene.h"
#include <numeric>
#include <stack>

std::optional<SimpleHitInfo>
BinnedBVH::BVHNode::Intersect(const RayInternal& ray) const
{
    const Vector3f invRayDir = Vector3f::One() / ray.dir;
    const Vector3f t0 = (m_boundary.lower - ray.o) * invRayDir;
    const Vector3f t1 = (m_boundary.upper - ray.o) * invRayDir;

    constexpr int idxArray[] = { 0, 1, 2, 0 };

    for (int cnt = 0; cnt < 3; cnt++)
    {
        const int axis0 = idxArray[cnt];
        const int axis1 = idxArray[cnt + 1];

        if (ray.dir[axis0] == 0 || ray.dir[axis1] == 0)
        {
            // 軸に平行なレイが入ってきた場合
            continue;
        }

        const float tMinMax = std::max(std::min(t0[axis0], t1[axis0]),
                                       std::min(t0[axis1], t1[axis1]));
        const float tMaxMin = std::min(std::max(t0[axis0], t1[axis0]),
                                       std::max(t0[axis1], t1[axis1]));
        if (tMaxMin < tMinMax)
        {
            return std::nullopt;
        }
    }

    float distance = std::numeric_limits<float>::max();
    for (int axis = 0; axis < 3; ++axis)
    {
        if (t0[axis] > 0.0f)
        {
            distance = std::min(distance, t0[axis]);
        }

        if (t1[axis] > 0.0f)
        {
            distance = std::min(distance, t1[axis]);
        }
    }

    const SimpleHitInfo hitInfo = [&] {
        SimpleHitInfo h;
        h.distance = distance;
        return h;
    }();
    return hitInfo;
}

void
BinnedBVH::Build(const Scene& scene)
{
    const auto numFaces = scene.GetNumFaces();

    // 事前に各面のバウンディングボックスと重心を計算しておく
    {
        m_precomputedFaceData.clear();
        m_precomputedFaceData.reserve(numFaces);
        for (uint32_t idxFace = 0; idxFace < numFaces; idxFace++)
        {
            const auto vertexPositions = scene.GetFaceVertices(idxFace);

            const AABB bound = [&] {
                AABB ret;
                ret.Merge(vertexPositions[0]);
                ret.Merge(vertexPositions[1]);
                ret.Merge(vertexPositions[2]);
                return ret;
            }();

            const Vector3f centroid = bound.GetCentroid();
            /*(vertexPositions[0] + vertexPositions[1] + vertexPositions[2]) /
            3.0f;*/

            const PrecomputedPrimitiveData data = [&] {
                PrecomputedPrimitiveData ret;
                ret.binID = -1;
                ret.aabb = bound;
                ret.centroid = centroid;
                return ret;
            }();
            m_precomputedFaceData.emplace_back(data);
        }
    }

    std::stack<uint32_t> nodeIndexStack;

    // ルートとなるノードを計算
    {
        m_bvhNodes.clear();
        m_bvhNodes.reserve(2ll * numFaces);

        const AABB rootNodeBoundary = [&] {
            AABB ret;

            for (const auto& primitiveData : m_precomputedFaceData)
            {
                ret.Merge(primitiveData.aabb);
            }
            return ret;
        }();

        m_bvhNodes.emplace_back(
          rootNodeBoundary, 0, static_cast<uint32_t>(numFaces));
        nodeIndexStack.emplace(static_cast<uint32_t>(m_bvhNodes.size() - 1));
    }

    // InPlaceソート用プリミティブID配列を初期化
    {
        m_faceIndicies.clear();
        m_faceIndicies.resize(numFaces);
        std::iota(std::begin(m_faceIndicies), std::end(m_faceIndicies), 0);
    }

    while (!nodeIndexStack.empty())
    {
        const auto currentBVHNodeIndex = nodeIndexStack.top();
        nodeIndexStack.pop();
        BVHNode& currentNode = m_bvhNodes[currentBVHNodeIndex];

        const uint32_t indexBegin = currentNode.GetIndexBegin();
        const uint32_t indexEnd = currentNode.GetIndexEnd();
        assert(indexBegin <= indexEnd);
        const uint32_t numPrimitivesInCurrentNode = (indexEnd - indexBegin);

        // ノード内のプリミティブ数が十分に少ない場合は分割しない
        constexpr int kLeastNumPrimitivesInNode = 4;
        if (numPrimitivesInCurrentNode <= kLeastNumPrimitivesInNode)
        {
            currentNode.SetLeaf(true);
            continue;
        }

        // バウンディングボックスの一番長い辺に沿ってソート
        const auto iterBegin = std::begin(m_faceIndicies) + indexBegin;
        const auto iterEnd = std::begin(m_faceIndicies) + indexEnd;

        // ビンをアップデート
        AABB binBoundary;
        for (auto iter = iterBegin; iter != iterEnd; iter++)
        {
            const auto faceIndex = *iter;
            binBoundary.Merge(m_precomputedFaceData[faceIndex].centroid);
        }

        const int widestAxis = binBoundary.GetWidestAxis();

        std::sort(iterBegin, iterEnd, [this, widestAxis](int id0, int id1) {
            return m_precomputedFaceData[id0].centroid[widestAxis] <
                   m_precomputedFaceData[id1].centroid[widestAxis];
        });

        // どのビンに属しているかを番号付け
        const float widestEdgeLength =
          binBoundary.upper[widestAxis] - binBoundary.lower[widestAxis];

        for (auto iter = iterBegin; iter != iterEnd; iter++)
        {
            const int primitiveID = *iter;

            PrecomputedPrimitiveData* const primitiveData =
              &m_precomputedFaceData[primitiveID];

            const float l = primitiveData->centroid[widestAxis] -
                            binBoundary.lower[widestAxis];

            const auto binID = static_cast<int>(
              std::nextafter(kNumBins * l / widestEdgeLength, 0.0f));
            assert(0 <= binID && binID < kNumBins);
            primitiveData->binID = binID;
        }

        // 最適な分割位置を探索
        int binPartitionIndexInBestDiv = 0;
        int numPrimsInLeftInBestDiv = 0;
        {
            float minCost = std::numeric_limits<float>::max();
            for (int binPartitionIndex = 0; binPartitionIndex <= kNumBins;
                 binPartitionIndex++)
            {
                const auto [cost, numPrimsInLeft] =
                  GetSAHCost(binPartitionIndex, currentNode);

                if (cost < minCost)
                {
                    minCost = cost;
                    binPartitionIndexInBestDiv = binPartitionIndex;
                    numPrimsInLeftInBestDiv = numPrimsInLeft;
                }
            }

            if (binPartitionIndexInBestDiv == 0 ||
                binPartitionIndexInBestDiv == kNumBins)
            {
                currentNode.SetLeaf(true);
                continue;
            }
        }

        // ---- 分割する場合 ----
        {
            AABB leftBoundary, rightBoundary;

            for (auto iter = iterBegin; iter != iterEnd; iter++)
            {
                const uint32_t faceIndex = *iter;

                const PrecomputedPrimitiveData& primitiveData =
                  m_precomputedFaceData[faceIndex];

                const bool isInLeftNode =
                  (primitiveData.binID < binPartitionIndexInBestDiv);
                if (isInLeftNode)
                {
                    leftBoundary.Merge(primitiveData.aabb);
                }
                else
                {
                    rightBoundary.Merge(primitiveData.aabb);
                }
            }

            // ノードに子ノードを登録
            {
                uint32_t leftChildIndex = 0, rightChildIndex = 0;

                {
                    m_bvhNodes.emplace_back(leftBoundary,
                                            currentNode.GetIndexBegin(),
                                            currentNode.GetIndexBegin() +
                                              numPrimsInLeftInBestDiv);

                    const auto index =
                      static_cast<uint32_t>(m_bvhNodes.size() - 1);
                    nodeIndexStack.emplace(index);
                    leftChildIndex = index;
                }

                {
                    m_bvhNodes.emplace_back(rightBoundary,
                                            currentNode.GetIndexBegin() +
                                              numPrimsInLeftInBestDiv,
                                            currentNode.GetIndexEnd());
                    const auto index =
                      static_cast<uint32_t>(m_bvhNodes.size() - 1);
                    nodeIndexStack.emplace(index);
                    rightChildIndex = index;
                }

                currentNode.SetChildIndices(leftChildIndex, rightChildIndex);
            }
        }
    }

    m_bvhNodes.shrink_to_fit();
}

std::optional<HitInfo>
BinnedBVH::Intersect(const RayInternal& ray,
                     const Scene& scene,
                     float distMin,
                     float distMax) const
{
    thread_local std::vector<uint32_t> bvhNodeIndexStack;
    bvhNodeIndexStack.clear();
    bvhNodeIndexStack.emplace_back(0);

    // ---- BVHのトラバーサル ----
    std::optional<HitInfo> finalHitInfo;

    // 2つの子ノード又は子オブジェクトに対して
    while (!bvhNodeIndexStack.empty())
    {
        // 葉ノードに対して
        const uint32_t currentNodeIndex = bvhNodeIndexStack.back();
        bvhNodeIndexStack.pop_back();
        const BVHNode& currentNode = m_bvhNodes[currentNodeIndex];

        const auto hitInfoNode = currentNode.Intersect(ray);

        // そもそもBVHノードに当たる軌道ではない
        if (hitInfoNode == std::nullopt)
        {
            continue;
        }

        // 自ノードより手前で既に衝突している
        if (finalHitInfo && currentNode.Contains(ray.o) == false)
        {
            if (finalHitInfo->distance < hitInfoNode->distance)
            {
                continue;
            }
        }

        if (currentNode.IsLeaf())
        {
            for (uint32_t i = currentNode.GetIndexBegin();
                 i < currentNode.GetIndexEnd();
                 i++)
            {
                const uint32_t faceIndex = m_faceIndicies[i];

                // 同じ面との再衝突を避ける
                if (ray.lastFaceIndex == faceIndex)
                {
                    continue;
                }

                const auto vertices = scene.GetFaceVertices(faceIndex);

                const auto hitInfoGeometry = IntersectRayTriangle(
                  vertices[0], vertices[1], vertices[2], ray);
                if (hitInfoGeometry)
                {
                    if (hitInfoGeometry->distance < distMin ||
                        hitInfoGeometry->distance > distMax)
                    {
                        continue;
                    }

                    if (finalHitInfo == std::nullopt ||
                        hitInfoGeometry->distance < finalHitInfo->distance)
                    {
                        finalHitInfo = [&] {
                            HitInfo f;
                            f.distance = hitInfoGeometry->distance;
                            f.faceIndex = faceIndex;
                            return f;
                        }();
                    }
                }
            }
        }
        else
        {
            const uint32_t leftChildIndex = currentNode.GetChildIndices()[0];
            const uint32_t rightChildIndex = currentNode.GetChildIndices()[1];
            assert(0 < leftChildIndex && 0 < rightChildIndex);

            const float sqDistLeft =
              (m_bvhNodes[leftChildIndex].GetCentroid() - ray.o)
                .SquaredLength();
            const float sqDistRight =
              (m_bvhNodes[rightChildIndex].GetCentroid() - ray.o)
                .SquaredLength();

            if (sqDistLeft < sqDistRight)
            {
                bvhNodeIndexStack.emplace_back(leftChildIndex);
                bvhNodeIndexStack.emplace_back(rightChildIndex);
            }
            else
            {
                bvhNodeIndexStack.emplace_back(rightChildIndex);
                bvhNodeIndexStack.emplace_back(leftChildIndex);
            }
        }
    }
    return finalHitInfo;
}

std::pair<float, int>
BinnedBVH::GetSAHCost(int binPartitionIndex, const BVHNode& currentNode) const
{
    AABB leftBoundary{}, rightBoundary{};
    int numPrimsInLeft = 0, numPrimsInRight = 0;

    const auto iterBegin =
      std::begin(m_faceIndicies) + currentNode.GetIndexBegin();
    const auto iterEnd = std::begin(m_faceIndicies) + currentNode.GetIndexEnd();

    for (auto iter = iterBegin; iter != iterEnd; iter++)
    {
        const uint32_t faceIndex = *iter;

        const PrecomputedPrimitiveData& primitiveData =
          m_precomputedFaceData[faceIndex];

        const int primitiveBinID = primitiveData.binID;
        if (primitiveBinID < binPartitionIndex)
        {
            // left
            leftBoundary.Merge(primitiveData.aabb);
            numPrimsInLeft++;
        }
        else
        {
            // right
            rightBoundary.Merge(primitiveData.aabb);
            numPrimsInRight++;
        }
    }

    const float surfaceAreaLeft =
      numPrimsInLeft ? leftBoundary.GetSurfaceArea() : 0;

    const float surfaceAreaRight =
      numPrimsInRight ? rightBoundary.GetSurfaceArea() : 0;

    const float cost =
      numPrimsInLeft * surfaceAreaLeft + numPrimsInRight * surfaceAreaRight;
    return { cost, numPrimsInLeft };
}
