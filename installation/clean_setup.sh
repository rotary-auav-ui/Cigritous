#!/bin/bash

echo 'Starting clean setup installer'

echo 'Installing Python dependencies'
pip3 install tflite-runtime opencv-python paho-mqtt pupil-apriltags

cd ~/

mkdir cigritous_ws 
cd cigritous_ws

git clone --recurse-submodules -b main https://github.com/rotary-auav-ui/cigritous.git

mv cigritous src

cd src/installation

if [[ "${ROS_DISTRO}" == "galactic" ]]
then
  echo "Detecting ROS2 Galactic. Uninstalling..."
  sh ./galactic_to_foxy.sh
elif [[ "${ROS_DISTRO}" != "foxy" ]]
then
  echo "Invalid ROS version. Installing ROS2 Foxy"
  sh ./foxy_install.sh
fi

echo "ROS2 ${ROS_DISTRO} installed"

sh px4/px4_install.sh

cd ~/cigritous_ws

colcon build --packages-select px4_msgs
colcon build --packages-select px4_ros_com
colcon build --packages-select cigritous

echo "source ~/cigritous_ws/install/setup.bash" >> ~/.bashrc

#sh ./install_autostart.sh

echo "Installation complete! Launch using 'ros2 launch cigritous vision.py' "