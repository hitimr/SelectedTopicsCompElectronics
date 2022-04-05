cmake_minimum_required(VERSION 3.14)

project(Benchmark)

# becnhmarker library
cuda_add_library(benchmarker benchmarker/benchmarker.cpp benchmarker/benchmarker_cuda.cu)

target_link_libraries(benchmarker 
    ${OpenVDB_LIBRARIES}
    ${TBB_LIBRARIES})

target_include_directories(benchmarker PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    "/usr/local/cuda/include/" 
    $ENV{LIB_DIR}/json/include
    $ENV{LIB_DIR}/oneapi-tbb-2021.5.0/include
    $ENV{LIB_DIR}/oneTBB/include
    $ENV{LIB_DIR}/plog-1.1.6/include)

target_compile_options(benchmarker PRIVATE 
    -fopenmp
)

# main Benchmarker
cuda_add_executable(raytracing main.cpp)
target_include_directories(raytracing PUBLIC 
    $ENV{LIB_DIR}/json/include
    $ENV{LIB_DIR}/plog/include)

target_link_libraries(raytracing
    benchmarker
    ${Boost_LIBRARIES}
    OpenMP::OpenMP_CXX
    ${OpenVDB_LIBRARIES}
    TBB::tbb
    )

# Targets for running
add_custom_target(
    run
    COMMAND raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)
