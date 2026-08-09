cmake_minimum_required(VERSION 3.18)

project(lidar_3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The whole project is headless and depends only on Eigen. (The old Open3D
# real-time demo was removed in the cleanup; ray tracing, both benchmark layers,
# the calibration probe, and the tests are all Open3D-free.)
find_package(Eigen3 REQUIRED)

# This is a latency benchmark: an unoptimized build produces meaningless timings
# and would not reproduce the -O2 cost-model calibration baked into layer2.hpp.
# Optimize by default (CMake adds no -O flags otherwise).
add_compile_options(-O2)

enable_testing()

add_executable(lidar_tests tests.cpp)
add_executable(lidar_scaling scaling.cpp)     # Layer 1: latency vs obstacle count
add_executable(calibrate calibrate.cpp)       # fits the Layer 2 cost model
add_executable(layer2_benchmark layer2_benchmark.cpp) # Layer 2: autonomy vs budget

foreach(tgt lidar_tests lidar_scaling calibrate layer2_benchmark)
    target_link_libraries(${tgt} PRIVATE Eigen3::Eigen)
endforeach()

add_test(NAME lidar_tests COMMAND lidar_tests)
