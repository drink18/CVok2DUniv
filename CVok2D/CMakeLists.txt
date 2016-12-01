cmake_minimum_required(VERSION 2.8.6)
project(CVok2DRoot)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_subdirectory(cvok2d)
add_subdirectory(unittest)
