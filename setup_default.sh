#!/bin/bash
echo "Setup unitree ros2 environment with default interface"
source /opt/ros/humble/setup.bash
source $HOME/go2_sim/src/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
