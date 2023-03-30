#!/bin/bash
## startup script for cigritous

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/foxy/setup.bash
source ~/cigritous_ws/install/setup.bash

ros2 launch cigritous vision.py