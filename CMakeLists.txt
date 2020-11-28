cmake_minimum_required (VERSION 3.1.0)
project (camera_display)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
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

install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION bin)
