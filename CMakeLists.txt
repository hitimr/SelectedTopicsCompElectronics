cmake_minimum_required(VERSION 3.14)
project("OpenVDB-NanoVDB-Benchmark")

# Paths
set(ENV{DIR_PROJ_ROOT} "${CMAKE_CURRENT_SOURCE_DIR}")
set(ENV{DIR_BUILD} $ENV{DIR_PROJ_ROOT}/build)
set(ENV{DIR_SRC} $ENV{DIR_PROJ_ROOT}/src)
set(ENV{DIR_UTIL} $ENV{DIR_SRC}/util)
set(ENV{LIB_DIR} $ENV{DIR_PROJ_ROOT}/lib)
set(ENV{TBB_DIR} $ENV{LIB_DIR}/oneapi-tbb-2021.5.0)
set(ENV{BLOSC_DIR} $ENV{LIB_DIR}/c-blosc-1.21.1/build)


set(TBB_INCLUDEDIR $ENV{TBB_DIR}/include)
set(TBB_LIBRARYDIR $ENV{TBB_DIR}/lib/intel64/gcc4.8)
set(BLOSC_INCLUDEDIR $ENV{BLOSC_DIR}/include)
set(BLOSC_LIBRARYDIR $ENV{BLOSC_DIR}/lib)


set(ENV{CUDA_TOOLKIT_ROOT_DIR} "/usr/local/cuda/bin/")


list(APPEND CMAKE_MODULE_PATH $ENV{LIB_DIR}/openvdb-9.0.0_modified/lib/cmake/OpenVDB)
find_package(OpenVDB REQUIRED)
include_directories(${OpenVDB_INCLUDE_DIR} SYSTEM)

list(APPEND CMAKE_MODULE_PATH $ENV{LIB_DIR}/oneapi-tbb-2021.5.0/cmake)
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS} SYSTEM)

find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)


# Versions, Standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(MINIMUM_BOOST_VERSION 1.63.0)
find_package(CUDA 10.0 REQUIRED)

set(CMAKE_CXX_FLAGS "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall")

set(CMAKE_CUDA_FLAGS "-O3 -g -Wall")
set(CMAKE_CUDA_FLAGS_DEBUG "-O0 -g -Wall")


# Macro for displaying all variables
# useful for debugging
macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()


# include_directories(lib)
add_subdirectory(src)
