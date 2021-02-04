#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/ros_ws/devel/setup.bash"

roslaunch scout joystick.launch
