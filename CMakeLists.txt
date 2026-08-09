cmake_minimum_required(VERSION 3.18)

project(lidar_3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen is required by everything. Open3D is only needed by the optional
# real-time demo (main.cpp); the whole benchmark + test suite is headless and
# Open3D-free, so Open3D is looked up quietly and the demo target is skipped if
# it is not installed.
find_package(Eigen3 REQUIRED)
find_package(Open3D QUIET)

enable_testing()

# --- Headless suite (Eigen only): tests + Layer 1/Layer 2 benchmarks ---
add_executable(lidar_tests tests.cpp)
add_executable(lidar_benchmark benchmark.cpp)
add_executable(lidar_scaling scaling.cpp)
add_executable(layer2_benchmark layer2_benchmark.cpp)
add_executable(calibrate calibrate.cpp)

foreach(tgt lidar_tests lidar_benchmark lidar_scaling layer2_benchmark calibrate)
    target_link_libraries(${tgt} PRIVATE Eigen3::Eigen)
endforeach()

add_test(NAME lidar_tests COMMAND lidar_tests)

# --- Optional real-time demo (needs Open3D) ---
if(Open3D_FOUND)
    add_executable(lidar_3d main.cpp)
    target_link_libraries(lidar_3d PRIVATE Open3D::Open3D Eigen3::Eigen)
else()
    message(STATUS "Open3D not found -- skipping the lidar_3d demo; the headless benchmarks still build.")
endif()
