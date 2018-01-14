cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(CVok2D)

if(MSVC)
endif()

file(GLOB_RECURSE FILES *.cpp *.h *.inl)
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
add_library(${PROJECT_NAME} STATIC ${FILES}) 
