cmake_minimum_required(VERSION 3.14) 

project(Benchmark)

# Dependencies
list(APPEND CMAKE_MODULE_PATH 
    "/usr/local/lib/cmake/OpenVDB/")

find_package(OpenVDB REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)



# becnhmarker library
set(LIB_BENCHMARKER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}") 
add_library(benchmarker benchmarker/benchmarker.cpp)
target_include_directories(benchmarker PUBLIC 
    "${LIB_BENCHMARKER_INCLUDE_DIRS}"
    "/usr/local/cuda/include/")

# nanoVDB Test
add_executable(nanoVDB nanovdb.cpp)
target_include_directories(nanoVDB PUBLIC 
    $ENV{DIR_NANO_VDB} 
    $ENV{DIR_NANO_VDB}/util
    "/usr/local/cuda/include/")


# main Benchmarker
add_executable(raytracing main.cpp)





target_link_libraries(raytracing 
    OpenVDB::openvdb
    Boost::program_options
    benchmarker)


target_link_libraries(nanoVDB 
    OpenVDB::openvdb
    Boost::program_options)

# Targets
add_custom_target(run
    COMMAND raytracing
    #COMMAND vdb_view mygrids.vdb
    DEPENDS raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR} 
)