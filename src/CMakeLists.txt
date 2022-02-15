cmake_minimum_required(VERSION 3.12) 

project(Benchmark)

# Dependencies
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")
find_package(OpenVDB REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost ${MINIMUM_BOOST_VERSION} REQUIRED COMPONENTS program_options)

add_executable(raytracing raytracing.cpp)




target_link_libraries(raytracing 
    OpenVDB::openvdb
    Boost::program_options)

# Targets
add_custom_target(run
    COMMAND raytracing
    #COMMAND vdb_view mygrids.vdb
    DEPENDS raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR} 
)