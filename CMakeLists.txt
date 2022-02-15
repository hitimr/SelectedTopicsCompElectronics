cmake_minimum_required(VERSION 3.12)
project("OpenVDB_Test")


set(ENV{DIR_PROJ_ROOT}, $(pwd))
set(ENV{DIR_BUILD}, $ENV{DIR_PROJ_ROOT}/build)
set(ENV{DIR_SRC}, $ENV{DIR_PROJ_ROOT}/src)
set(ENV{DIR_UTIL}, $ENV{DIR_SRC}/util)


add_subdirectory(src)




