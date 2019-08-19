project(rayrun)
cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0076 NEW)

set(CMAKE_CONFIGURATION_TYPES
    "Debug;RelWithDebInfo;Release"
    CACHE STRING "" FORCE)

if(MSVC)
  add_compile_options(/MP /utf-8)
endif()

# ---- refimpl ----
add_library(refimpl SHARED)
target_sources(refimpl
               PRIVATE
               "src/refimpl.cpp"
               PUBLIC
               "src/rayrun.hpp")
target_compile_features(refimpl PUBLIC cxx_std_17)

# ---- myimpl ----
add_library(myimpl SHARED)
target_sources(myimpl
               PRIVATE
               "mysrc/impl/vector3f.h"
               "mysrc/impl/myimpl.cpp"
               "mysrc/impl/scene.h"
               "mysrc/impl/scene.cpp"
               "mysrc/impl/binnedBVH.h"
               "mysrc/impl/binnedBVH.cpp"
               "mysrc/impl/aabb.h"
               "mysrc/impl/intersection.h"
               "mysrc/impl/intersection.cpp"
               "mysrc/impl/hitInfo.h"
               "mysrc/impl/rayInternal.h"
               PUBLIC
               "src/rayrun.hpp")
target_include_directories(myimpl PRIVATE "src" INTERFACE "mysrc/impl")
target_compile_features(myimpl PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(myimpl
                         PRIVATE $<$<CONFIG:RelWithDebInfo>:
                                 /fp:fast
                                 /Oi
                                 /arch:AVX2>
                                 $<$<CONFIG:Release>:/MT
                                 /Oi
                                 /fp:fast
                                 /arch:AVX2>)
endif()

# ---- rayrun ----
add_executable(rayrun "src/main.cpp")
target_include_directories(rayrun
                           PRIVATE "thirdparty/stb" "thirdparty/tinyobjloader"
                                   "thirdparty/picojson")

set_target_properties(rayrun
                      PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY
                                 "${CMAKE_SOURCE_DIR}"
                                 VS_DEBUGGER_COMMAND_ARGUMENTS
                                 "myimpl asset/hairball.json")
target_compile_features(rayrun PUBLIC cxx_std_17)

find_package(OpenMP)
target_link_libraries(rayrun PRIVATE myimpl OpenMP::OpenMP_CXX)

# ---- mybenchmark ----
option(BENCHMARK_ENABLE_TESTING "" OFF)
option(BENCHMARK_ENABLE_INSTALL "" OFF)
option(BENCHMARK_ENABLE_GTEST_TESTS "" OFF)
add_subdirectory(thirdparty/benchmark)
add_executable(mybenchmark)
target_sources(mybenchmark PRIVATE "mysrc/benchmark/benchmark_vector3f.cpp")
target_link_libraries(mybenchmark PRIVATE myimpl benchmark benchmark_main)

# Set the startup project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT rayrun)
