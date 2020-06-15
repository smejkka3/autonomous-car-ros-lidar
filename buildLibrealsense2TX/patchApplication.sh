#!/bin/bash
# Applies application patches for librealsense2 on a Jetson TX2 Developer Kit

echo "${green}Applying Model-Views Patch${reset}"
# The render loop of the post processing does not yield; add a sleep
patch -p1 -i /home/jetson3/buildLibrealsense2TX2/patches/model-views.patch

echo "${green}Applying Incomplete Frames Patch${reset}"
# The Jetson tends to return incomplete frames at high frame rates; suppress error logging
patch -p1 -i /home/jetson3/buildLibrealsense2TX2/patches/incomplete-frame.patch


