cmake_minimum_required(VERSION 3.14)

project(Benchmark)

# Dependencies
list(APPEND CMAKE_MODULE_PATH "/home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB")
list(APPEND CMAKE_PREFIX_PATH /home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB/)
list(APPEND CMAKE_PREFIX_PATH /home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build)

add_definitions(-D TBB_DIR=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build)
add_definitions(-D CMAKE_PREFIX_PATH=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB)

find_package(CUDA 11.6 REQUIRED)
find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)

# becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
cuda_add_library(   benchmarker 
                    benchmarker/benchmarker.cpp
                    benchmarker/benchmarker_cuda.cu)

target_compile_options(benchmarker PRIVATE -fopenmp)
target_link_libraries(benchmarker 
    /home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/libopenvdb.so
    /home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build/lib64/libtbb.so)
    
target_include_directories(benchmarker PUBLIC 
    "${LIB_BENCHMARKER_INCLUDE_DIRS}"
    "/usr/local/cuda/include/" 
    $ENV{DIR_PROJ_ROOT}/lib/json/include
    $ENV{DIR_PROJ_ROOT}/lib/openvdb/build/include
    $ENV{DIR_PROJ_ROOT}/lib/oneTBB/build/include)

# main Benchmarker
cuda_add_executable(raytracing main.cpp)
target_include_directories(raytracing PUBLIC $ENV{DIR_PROJ_ROOT}/lib/json/include)
target_link_libraries(raytracing
    benchmarker
    /home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/libopenvdb.so 
    Boost::program_options 
    OpenMP::OpenMP_CXX
    )

# Targets for running
add_custom_target(
    run
    COMMAND raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)
