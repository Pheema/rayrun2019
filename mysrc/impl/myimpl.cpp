#include "rayrun.hpp"
#include "scene.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

BOOL WINAPI
AllMain(_In_ HINSTANCE hinstDLL, _In_ DWORD fdwReason, _In_ LPVOID lpvReserved)
{
    switch (fdwReason)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_PROCESS_DETACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    {
        break;
    }
    }

    return TRUE;
}

Scene g_scene;

void
preprocess(const float* vertices,
           size_t numVerts,
           const float* normals,
           size_t numNormals,
           const uint32_t* indices,
           size_t numFace)
{
    g_scene = Scene();
    g_scene.Build(vertices, numVerts, normals, numNormals, indices, numFace);
}

void
intersect(Ray* rays, size_t numRay, [[maybe_unused]] bool hitany)
{
    for (size_t idxRay = 0; idxRay < numRay; idxRay++)
    {
        Ray& ray = rays[idxRay];
        auto hitInfo = g_scene.Intersect(ray);

        if (!hitInfo)
        {
            ray.isisect = false;
            continue;
        }

        const uint32_t idxFace = hitInfo->idxFace;

        // #TODO: 交差した三角形の情報を元にして交差位置と法線を補間

        // #TODO: Rayに交差判定の結果を収める
        // ray = ~~~;
    }
}
