# Install script for directory: /home/karel/Documents/University/WS2019/AAIP/software_integration/src/sixwd_msgs/src/sixwd_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/karel/Documents/University/WS2019/AAIP/software_integration/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sixwd_msgs/msg" TYPE FILE FILES
    "/home/karel/Documents/University/WS2019/AAIP/software_integration/src/sixwd_msgs/src/sixwd_msgs/msg/SixWheelCommand.msg"
    "/home/karel/Documents/University/WS2019/AAIP/software_integration/src/sixwd_msgs/src/sixwd_msgs/msg/SixWheelInfo.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sixwd_msgs/cmake" TYPE FILE FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/build/sixwd_msgs/src/sixwd_msgs/catkin_generated/installspace/sixwd_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/include/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/share/roseus/ros/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/share/common-lisp/ros/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/share/gennodejs/ros/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/lib/python3/dist-packages/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/devel/lib/python3/dist-packages/sixwd_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/build/sixwd_msgs/src/sixwd_msgs/catkin_generated/installspace/sixwd_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sixwd_msgs/cmake" TYPE FILE FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/build/sixwd_msgs/src/sixwd_msgs/catkin_generated/installspace/sixwd_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sixwd_msgs/cmake" TYPE FILE FILES
    "/home/karel/Documents/University/WS2019/AAIP/software_integration/build/sixwd_msgs/src/sixwd_msgs/catkin_generated/installspace/sixwd_msgsConfig.cmake"
    "/home/karel/Documents/University/WS2019/AAIP/software_integration/build/sixwd_msgs/src/sixwd_msgs/catkin_generated/installspace/sixwd_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sixwd_msgs" TYPE FILE FILES "/home/karel/Documents/University/WS2019/AAIP/software_integration/src/sixwd_msgs/src/sixwd_msgs/package.xml")
endif()

