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
        g_scene.Intersect(ray);
    }
}
