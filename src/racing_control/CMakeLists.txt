cmake_minimum_required(VERSION 2.8.3)
project(racing_control)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  track_detection		
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
)

add_executable(racing_control src/RacingControl.cpp)
target_link_libraries(racing_control ${catkin_LIBRARIES})
add_dependencies(racing_control ${catkin_EXPORTED_TARGETS})

install(TARGETS racing_control RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

