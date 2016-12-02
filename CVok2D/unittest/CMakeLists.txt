cmake_minimum_required(VERSION 3.6.0)
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

link_directories(${GTEST_DIR}/libs)

set(GTEST_DBG_LIB gtestd.lib gtest_maind.lib)
set(GTEST_REL_LIB gtest.lib gtest_main.lib)
set(LINK_LIBRARY optimized ${GTEST_REL_LIB} debug ${GTEST_DBG_LIB})


add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} debug gtestd.lib debug gtest_maind.lib)
target_link_libraries(${PROJECT_NAME} optimized gtest.lib optimized gtest_main.lib)

