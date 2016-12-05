cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(unittest)

file(GLOB_RECURSE FILES *.cpp *.h)
set(GTEST_DIR ${CMAKE_SOURCE_DIR}/extern/gtest/)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
include_directories(${GTEST_DIR}/include)

if(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG /MTd)
    set(CMAKE_CXX_FLAGS_RELEASE /MT)
endif()

if(MSVC)
   link_directories(${GTEST_DIR}/libs/win64)
else()
   link_directories(${GTEST_DIR}/libs/macos)
endif()

set(GTEST_DBG_LIB gtestd gtest_maind)
set(GTEST_REL_LIB gtest gtest_main)


add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} debug gtestd debug gtest_maind)
target_link_libraries(${PROJECT_NAME} optimized gtest optimized gtest_main)

