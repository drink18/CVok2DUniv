cmake_minimum_required(VERSION 3.0.0) 
project(CVok2DRoot)

#set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
if(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")
endif()

include_directories("${gtest_SOURCE_DIR}/include")
include_directories("${gtest_SOURCE_DIR}/../googlemock/include")

add_subdirectory(extern/glew)
add_subdirectory(extern/glfw)
add_subdirectory(extern/googletest)
add_subdirectory(cvok2d)
add_subdirectory(unittest)
add_subdirectory(cvok2dTestBed)
add_subdirectory(convexDecomp)
