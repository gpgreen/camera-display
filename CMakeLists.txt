cmake_minimum_required (VERSION 3.1.0)
project (camera_display)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CAMERA_DEV_HDRS
	c328.h
	c328_cmd.h
)

set(CAMERA_DEV_SRCS
	c328.cpp
	c328_cmd.cpp
)

set(CAMERA_GUI_HDRS
#	canvas.h
)

set(CAMERA_GUI_SRCS
#	canvas.cpp
	main.cpp
)

find_package(wxWidgets REQUIRED COMPONENTS net core base)
include(${wxWidgets_USE_FILE})

add_executable(${CMAKE_PROJECT_NAME} ${CAMERA_DEV_SRCS} ${CAMERA_GUI_SRCS})
target_link_libraries(${CMAKE_PROJECT_NAME} ${wxWidgets_LIBRARIES})

add_executable(test_c328camera ${CAMERA_DEV_SRCS} test_camera.cpp)

install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION bin)
install(TARGETS test_c328camera DESTINATION bin)
