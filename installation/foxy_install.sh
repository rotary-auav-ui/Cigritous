#!/bin/bash
echo "Installing ROS2 Foxy"
sudo -S apt -y update && sudo -S apt -y install locales
sudo -S locale-gen en_US en_US.UTF-8
sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo -S apt -y install software-properties-common
sudo -S add-apt-repository universe

sudo -S apt update && sudo -S apt install curl -y
sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo -S apt -y update && sudo -S apt -y upgrade

sudo -S apt -y install ros-foxy-ros-base python3-argcomplete

sudo -S apt -y install ros-dev-tools

sudo -S apt -y install ros-foxy-rmw-fastrtps-cpp

sudo -S apt -y install python3-colcon-common-extensions

sudo -S apt -y install ros-foxy-eigen3-cmake-module

sudo -S pip3 install -y -U empy pyros-genmsg setuptools

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc