cmake_minimum_required(VERSION 3.12)

project("HelloWorld")
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB/")
find_package(OpenVDB REQUIRED)
find_package(Threads REQUIRED)

add_executable(HelloWorld hello_world.cpp)
target_link_libraries(HelloWorld OpenVDB::openvdb)