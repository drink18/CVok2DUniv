cmake_minimum_required(VERSION 3.6.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(CVok2D)

if(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG /MTd)
    set(CMAKE_CXX_FLAGS_RELEASE /MT)
endif()

file(GLOB_RECURSE FILES *.cpp *.h)
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
add_library(${PROJECT_NAME} STATIC ${FILES}) 
