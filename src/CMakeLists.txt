cmake_minimum_required(VERSION 3.14)

project(Benchmark)

# Dependencies
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")

find_package(OpenVDB REQUIRED)
find_package(Threads REQUIRED)
find_package(CUDA 11.6 REQUIRED)
find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)

# becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
cuda_add_library(benchmarker benchmarker/benchmarker.cpp
                 benchmarker/nanoVDB_GPU.cu)
target_include_directories(benchmarker PUBLIC "${LIB_BENCHMARKER_INCLUDE_DIRS}"
                                              "/usr/local/cuda/include/")

# main Benchmarker
cuda_add_executable(raytracing main.cpp)

target_link_libraries(raytracing OpenVDB::openvdb Boost::program_options
                      benchmarker)

target_link_libraries(nanoVDB OpenVDB::openvdb Boost::program_options)

# Targets
add_custom_target(
  run
  COMMAND raytracing
  # COMMAND vdb_view mygrids.vdb
  DEPENDS raytracing
  WORKING_DIRECTORY ${CMAKE_PROJECT_DIR})
