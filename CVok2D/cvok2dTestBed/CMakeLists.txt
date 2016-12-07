cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(cvok2DTestBed)

file(GLOB_RECURSE FILES *.cpp *.h *.c)

set(GLFW_DIR ${CMAKE_SOURCE_DIR}/extern/glfw)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed/imgui)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed/gl3w)

include_directories(${GLFW_DIR}/include)


if(MSVC)
    #link_directories(${GLFW_DIR}/lib-vc2010-64)
else()
   link_directories(${GTEST_DIR}/libs/macos)
endif()

add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} glew)
target_link_libraries(${PROJECT_NAME} glfw)
target_link_libraries(${PROJECT_NAME} opengl32)
target_link_libraries(${PROJECT_NAME} legacy_stdio_definitions)
