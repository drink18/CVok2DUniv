cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(unittest)

file(GLOB_RECURSE FILES *.cpp *.h *.c)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)

add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} debug gtestd debug gtest_maind)
target_link_libraries(${PROJECT_NAME} optimized gtest optimized gtest_main)

