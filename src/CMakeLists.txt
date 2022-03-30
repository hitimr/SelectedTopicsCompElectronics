cmake_minimum_required(VERSION 3.14)

project(Benchmark)

# Dependencies
list(APPEND CMAKE_MODULE_PATH "/home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB")

list(APPEND CMAKE_PREFIX_PATH /home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB/)
list(APPEND CMAKE_PREFIX_PATH /home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build)

# add_definitions(-D OpenVDB_INCLUDE_DIR=ENV{DIR_PROJ_ROOT}/lib/openvdb/build/include)
# add_definitions(-D OpenVDB_LIB_COMPONENTS=ENV{DIR_PROJ_ROOT}/lib/openvdb/build/lib64)

# add_definitions(-D Tbb_INCLUDE_DIR=ENV{DIR_PROJ_ROOT}/lib/oneTBB/build/include)
# add_definitions(-D Tbb_LIB_COMPONENTS=ENV{DIR_PROJ_ROOT}/lib/oneTBB/lib64) 
add_definitions(-D TBB_DIR=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build)
add_definitions(-D CMAKE_PREFIX_PATH=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB)
# add_definitions(-D Tbb_INCLUDE_DIR=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build/include)
# add_definitions(-D Tbb_INCLUDE_DIR=/home/hiti/workspace/SelectedTopicsCompElectronics/lib/oneTBB/build/include)
# find_package(OpenVDB REQUIRED)
# find_package(TBB Required)

# add_definitions(-D OpenVDB_LIB_COMPONENTS=ENV{DIR_PROJ_ROOT}/lib/openvdb/build/lib64)
# -D TBB_INCLUDEDIR=$TBB_BUILD_DIR/include \
# -D TBB_LIBRARYDIR=$TBB_BUILD_DIR/lib64 \
# -D BLOSC_INCLUDEDIR=$BLOSC_BUILD_DIR/include \
# -D BLOSC_LIBRARYDIR=$BLOSC_BUILD_DIR/lib64 \
# find_package(Tbb REQUIRED PATHS /home/hiti/workspace/SelectedTopicsCompElectronics/lib/openvdb/build/lib64/cmake/OpenVDB/FindTBB.cmake)
# find_package(OpenVDB)
# find_package(Threads REQUIRED)
find_package(CUDA 11.6 REQUIRED)
find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)

# # becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
cuda_add_library(   benchmarker 
                    benchmarker/benchmarker.cpp
                    benchmarker/benchmarker_cuda.cu)

target_compile_options(benchmarker PRIVATE -fopenmp)
target_link_libraries(benchmarker openvdb)
target_include_directories(benchmarker PUBLIC "${LIB_BENCHMARKER_INCLUDE_DIRS}"
                                              "/usr/local/cuda/include/" ENV{DIR_PROJ_ROOT}/lib/json/include)

# main Benchmarker
cuda_add_executable(raytracing main.cpp)

target_link_libraries(raytracing openvdb Boost::program_options OpenMP::OpenMP_CXX
                      benchmarker OpenMP::OpenMP_CXX)


