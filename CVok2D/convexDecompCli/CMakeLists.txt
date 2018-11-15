cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(convexDecompCli)

file(GLOB_RECURSE FILES *.cpp *.h *.c)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
include_directories(${CMAKE_SOURCE_DIR}/convexDecomp)


add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} convexDecomp)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_SOURCE_DIR}/../bin/${PROJECT_NAME}.exe)
