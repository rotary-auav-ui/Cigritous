#!/bin/bash
## startup script for cigritous

source /opt/ros/foxy/setup.bash
source ~/cigritous_ws/install/setup.bash

ros2 launch cigritous vision.py