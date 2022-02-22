cmake_minimum_required(VERSION 3.14)
project("OpenVDB_Test")


set(ENV{DIR_PROJ_ROOT} "${CMAKE_CURRENT_SOURCE_DIR}")
set(ENV{DIR_BUILD} $ENV{DIR_PROJ_ROOT}/build)
set(ENV{DIR_SRC} $ENV{DIR_PROJ_ROOT}/src)
set(ENV{DIR_UTIL} $ENV{DIR_SRC}/util)
set(ENV{DIR_NANO_VDB} $ENV{DIR_PROJ_ROOT}/openvdb/nanovdb/nanovdb/)

set(OpenVDB_DIR $ENV{DIR_PROJ_ROOT}/openvdb/cmake/)

set(MINIMUM_BOOST_VERSION 1.63.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)



set(CMAKE_CXX_FLAGS "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall")

include_directories(lib)
add_subdirectory(src)




