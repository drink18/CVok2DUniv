cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE INTERNAL "limited configs")
project(cvok2DTestBed)

file(GLOB_RECURSE FILES *.cpp *.h *.c)

find_package(OpenGL REQUIRED)
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/extern/glfw)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/cvok2d)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed/imgui)
include_directories(${CMAKE_SOURCE_DIR}/cvok2dTestBed/gl3w)


if(WIN32)
    ADD_LIBRARY(brofiler STATIC IMPORTED)
    SET_TARGET_PROPERTIES(brofiler PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/extern/Brofiler-1.1.1/ProfilerCore64.lib)
    include_directories(${CMAKE_SOURCE_DIR}/extern/Brofiler-1.1.1)
endif(WIN32)


include_directories(${GLFW_DIR}/include)

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib )
	set (OPENGL_LIBRARIES GL GLU X11)
elseif(WIN32)
	set (ADDITIONAL_LIBRARIES winmm)
endif(APPLE)

add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} CVok2D)
target_link_libraries(${PROJECT_NAME} ${OPENGL_gl_LIBRARY})
target_link_libraries(${PROJECT_NAME} glew)
target_link_libraries(${PROJECT_NAME} glfw)
target_link_libraries(${PROJECT_NAME} brofiler)

if(WIN32)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/extern/Brofiler-1.1.1/ProfilerCore64.dll"
    $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif(WIN32)
