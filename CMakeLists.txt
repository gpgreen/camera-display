cmake_minimum_required (VERSION 3.1.0)
project (camera_display)
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 1)

set(CAMERA_DISPLAY_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set(CAMERA_DISPLAY_VERSION_DATE "2020-11-26")

set(CMAKE_VERBOSE_MAKEFILE ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CAMERA_DISPLAY_HDRS
	c328.h
	c328_cmd.h
)

set(CAMERA_DISPLAY_SRCS
	c328.cpp
	c328_cmd.cpp
	test_camera.cpp
)


add_executable(${CMAKE_PROJECT_NAME} ${CAMERA_DISPLAY_SRCS})
