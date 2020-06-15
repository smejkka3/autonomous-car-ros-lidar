cmake_minimum_required(VERSION 2.8.3)
project(track_detection)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  TrackMsg.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

add_executable(track_detection src/TrackDetector.cpp src/Map.cpp src/ConeKernel.cpp)
target_link_libraries(track_detection ${catkin_LIBRARIES})
add_dependencies(track_detection track_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS track_detection RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

