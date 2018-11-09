cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(convexDecompTest)

file(GLOB_RECURSE FILES *.cpp *.h *.c)

find_package(OpenGL REQUIRED)
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/extern/glfw)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
include_directories(${CMAKE_SOURCE_DIR}/convexDecomp)
include_directories(${CMAKE_SOURCE_DIR}/convexDecomp/imgui)
include_directories(${CMAKE_SOURCE_DIR}/convexDecomp/gl3w)

include_directories(${GLFW_DIR}/include)

set (ADDITIONAL_LIBRARIES winmm)
add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} ${OPENGL_gl_LIBRARY})
target_link_libraries(${PROJECT_NAME} glew)
target_link_libraries(${PROJECT_NAME} glfw)
