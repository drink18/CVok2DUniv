cmake_minimum_required(VERSION 3.0.0) 

add_definitions( -DGLEW_STATIC -D_CRT_SECURE_NO_WARNINGS )

add_library(glew STATIC 
	glew.c
    GL/glew.h
    GL/glxew.h
    GL/wglew.h
)
