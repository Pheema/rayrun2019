#include "rayrun.hpp"

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

void
preprocess(const float* vertices,
           size_t numVerts,
           const float* normals,
           size_t numNormals,
           const uint32_t* indices,
           size_t numFace)
{
    // #TODO: 実装
}

void
intersect(Ray* rays, size_t numRay, bool hitany)
{
    // #TODO: 実装
}
