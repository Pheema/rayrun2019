project(rayrun)
cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0076 NEW)

set(CMAKE_CONFIGURATION_TYPES "Debug;RelWithDebInfo;Release"
    CACHE STRING ""
    FORCE)

add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
add_compile_options($<$<C_COMPILER_ID:MSVC>:/utf-8>)

# ---- refimpl ----
add_library(refimpl SHARED)
target_sources(refimpl PRIVATE "src/refimpl.cpp" PUBLIC "src/rayrun.hpp")
target_compile_features(refimpl PUBLIC cxx_std_17)

# ---- rayrun ----
add_executable(rayrun "src/main.cpp")
target_include_directories(rayrun
                           PRIVATE "thirdparty/stb" "thirdparty/tinyobjloader"
                                   "thirdparty/picojson")

set_target_properties(rayrun
                      PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY
                                 "${CMAKE_SOURCE_DIR}"
                                 VS_DEBUGGER_COMMAND_ARGUMENTS
                                 "refimpl asset/hairball.json")
target_compile_features(rayrun PUBLIC cxx_std_17)

find_package(OpenMP)
target_link_libraries(rayrun PRIVATE refimpl OpenMP::OpenMP_CXX)

# ---- rayrunbenchmark ----
option(BENCHMARK_ENABLE_TESTING "" OFF)
option(BENCHMARK_ENABLE_INSTALL "" OFF)
option(BENCHMARK_ENABLE_GTEST_TESTS "" OFF)
add_subdirectory(thirdparty/benchmark)
add_executable(rayrunbenchmark "src/benchmark/main.cpp")
target_link_libraries(rayrunbenchmark PRIVATE benchmark)
target_include_directories(rayrunbenchmark PRIVATE benchmark)

# Set the startup project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT rayrun)
