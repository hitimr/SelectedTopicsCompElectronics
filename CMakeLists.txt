cmake_minimum_required(VERSION 3.12)

project("OpenVDB_Test")
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")
find_package(OpenVDB REQUIRED)
find_package(Threads REQUIRED)

add_executable(raytracing raytracing.cpp)
target_link_libraries(raytracing OpenVDB::openvdb)


add_custom_target(run
    COMMAND raytracing
    #COMMAND vdb_view mygrids.vdb
    DEPENDS raytracing
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR} 
)