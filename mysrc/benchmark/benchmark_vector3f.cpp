#pragma once

#include "benchmark/benchmark.h"
#include "vector3f.h"

static void
BM_Vector3f_add(benchmark::State& state)
{
    Vector3f v0(0.0f, 1.0f, 2.0f);
    Vector3f v1(3.0f, 4.0f, 5.0f);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(v0 + v1);
    }
}
BENCHMARK(BM_Vector3f_add);
