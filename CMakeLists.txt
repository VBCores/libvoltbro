project(libvoltbro)
cmake_minimum_required(VERSION 3.15)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(.)
file(GLOB_RECURSE SOURCES "voltbro/*.cpp")

include_directories(voltbro)

add_library(libvoltbro ${SOURCES})
