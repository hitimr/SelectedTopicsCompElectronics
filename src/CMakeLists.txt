cmake_minimum_required(VERSION 3.14)

project(Benchmark)


# set(Tbb_INCLUDE_DIR ${TBB_DIR}/include)
# set(TBB_LIBRARYDIR ${TBB_DIR}/lib/intel64/gcc4.8/)
# set(Tbb_LIB_COMPONENTS ${TBB_DIR}/lib/intel64/gcc4.8/)


# print_all_variables()

# Dependencies

list(APPEND CMAKE_MODULE_PATH $ENV{LIB_DIR}/oneapi-tbb-2021.5.0/lib/cmake/tbb)
list(APPEND CMAKE_MODULE_PATH $ENV{LIB_DIR}/openvdb-9.0.0/lib/cmake/OpenVDB)

find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)
find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenVDB REQUIRED)




# becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
cuda_add_library(   benchmarker 
                    benchmarker/benchmarker.cpp
                    benchmarker/benchmarker_cuda.cu)

target_compile_options(benchmarker PRIVATE -fopenmp)
target_link_libraries(benchmarker 
    OpenVDB::openvdb)


target_include_directories(benchmarker PUBLIC 
    "${LIB_BENCHMARKER_INCLUDE_DIRS}"
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
    TBB::tbb
    Boost::program_options 
    OpenMP::OpenMP_CXX
    OpenVDB::openvdb
    )

# Targets for running
add_custom_target(
    run
    COMMAND raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)
