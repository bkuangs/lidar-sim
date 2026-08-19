cmake_minimum_required(VERSION 3.18)

project(lidar_3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The whole project is headless and depends only on Eigen. (The old Open3D
# real-time demo was removed in the cleanup; ray tracing, benchmarks, and tests
# are all Open3D-free.)
find_package(Eigen3 REQUIRED)

# This is a latency benchmark: an unoptimized build produces meaningless timings.
# Optimize by default (CMake adds no -O flags otherwise).
add_compile_options(-O2)

enable_testing()

# Header-only core: every ray-tracing / scene / vehicle / benchmark header lives
# under include/. This INTERFACE target carries that include path plus Eigen so
# each executable just links `lidar3d`.
add_library(lidar3d INTERFACE)
target_include_directories(lidar3d INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(lidar3d INTERFACE Eigen3::Eigen)

add_executable(lidar_tests         tests/tests.cpp)
add_executable(lidar_scaling       benchmarks/scaling.cpp)          # Layer 1: latency vs obstacle count
add_executable(hazard_benchmark    benchmarks/hazard_benchmark.cpp)

foreach(tgt lidar_tests lidar_scaling hazard_benchmark)
    target_link_libraries(${tgt} PRIVATE lidar3d)
endforeach()

add_executable(nested_ray_layouts_test tests/nested_ray_layouts.cpp)
add_executable(object_id_propagation_tests tests/object_id_propagation.cpp)
add_executable(hazard_trial_tests tests/hazard_trial_tests.cpp)
add_executable(hazard_benchmark_tests tests/hazard_benchmark_tests.cpp)

foreach(tgt nested_ray_layouts_test object_id_propagation_tests hazard_trial_tests hazard_benchmark_tests)
    target_link_libraries(${tgt} PRIVATE lidar3d)
endforeach()

add_test(NAME lidar_tests COMMAND lidar_tests)
add_test(NAME nested_ray_layouts COMMAND nested_ray_layouts_test)
add_test(NAME object_id_propagation COMMAND object_id_propagation_tests)
add_test(NAME hazard_trial COMMAND hazard_trial_tests)
add_test(NAME hazard_benchmark_helpers COMMAND hazard_benchmark_tests)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(
    NAME hazard_analysis
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_hazard_analysis.py)
add_test(
    NAME object_count_analysis
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_object_count_analysis.py)

set(HAZARD_MVP_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/hazard_mvp)
add_custom_target(
    hazard_mvp
    COMMAND ${CMAKE_COMMAND} -E make_directory ${HAZARD_MVP_OUTPUT_DIR}
    COMMAND $<TARGET_FILE:hazard_benchmark>
            safety --csv ${HAZARD_MVP_OUTPUT_DIR}/hazard_trials.csv
    COMMAND $<TARGET_FILE:hazard_benchmark>
            timing --csv ${HAZARD_MVP_OUTPUT_DIR}/hazard_timing.csv
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/analyze_hazard.py
            --trials ${HAZARD_MVP_OUTPUT_DIR}/hazard_trials.csv
            --timing ${HAZARD_MVP_OUTPUT_DIR}/hazard_timing.csv
            --out-dir ${HAZARD_MVP_OUTPUT_DIR}/analysis
    DEPENDS hazard_benchmark
    USES_TERMINAL
    COMMENT "Running the end-to-end hazard MVP acceptance gates")

set(HAZARD_OBJECT_COUNT_OUTPUT_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/hazard_object_count)
add_custom_target(
    hazard_object_count
    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${HAZARD_OBJECT_COUNT_OUTPUT_DIR}
    COMMAND $<TARGET_FILE:hazard_benchmark>
            object-count-timing
            --csv ${HAZARD_OBJECT_COUNT_OUTPUT_DIR}/hazard_object_count_timing.csv
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/analyze_object_count.py
            --trials ${CMAKE_CURRENT_SOURCE_DIR}/plots/hazard_trials.csv
            --timing ${HAZARD_OBJECT_COUNT_OUTPUT_DIR}/hazard_object_count_timing.csv
            --out-dir ${HAZARD_OBJECT_COUNT_OUTPUT_DIR}/analysis
    DEPENDS hazard_benchmark
    USES_TERMINAL
    COMMENT "Running the object-count scene-BVH extension")
