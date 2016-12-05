cmake_minimum_required(VERSION 3.0.0) 
project(CVok2DRoot)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
if(MSVC)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_subdirectory(cvok2d)
add_subdirectory(unittest)
add_subdirectory(cvok2dTestBed)
