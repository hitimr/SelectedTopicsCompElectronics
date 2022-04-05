cmake_minimum_required(VERSION 3.14)

project(Benchmark)

# Dependencies
list(APPEND CMAKE_PREFIX_PATH $ENV{LIB_DIR})


add_definitions(-D CMAKE_PREFIX_PATH=$ENV{LIB_DIR}/openvdb/build/lib64/cmake/OpenVDB)


find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)

# becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
cuda_add_library(   benchmarker 
                    benchmarker/benchmarker.cpp
                    benchmarker/benchmarker_cuda.cu)

target_compile_options(benchmarker PRIVATE -fopenmp)
target_link_libraries(benchmarker 
    $ENV{LIB_DIR}/openvdb/build/lib64/libopenvdb.so)


target_include_directories(benchmarker PUBLIC 
    "${LIB_BENCHMARKER_INCLUDE_DIRS}"
    "/usr/lib/gcc/x86_64-linux-gnu/9/include/"
    "/usr/local/cuda/include/" 
    $ENV{LIB_DIR}json/include
    $ENV{LIB_DIR}/oneapi-tbb-2021.5.0/include
    $ENV{LIB_DIR}/oneTBB/include
    $ENV{LIB_DIR}/plog-1.1.6/include)

# main Benchmarker
cuda_add_executable(raytracing main.cpp)
target_include_directories(raytracing PUBLIC 
    $ENV{LIB_DIR}/json/include
    $ENV{LIB_DIR}/plog/include)

target_link_libraries(raytracing
    benchmarker
    $ENV{LIB_DIR}/openvdb/build/lib64/libopenvdb.so
    TBB::tbb
    Boost::program_options 
    OpenMP::OpenMP_CXX
    )

# Targets for running
add_custom_target(
    run
    COMMAND raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)
