cd ~/cigritous_ws

colcon build --packages-select px4_msgs
colcon build --packages-select px4_ros_com
colcon build --packages-select cigritous

echo "source ~/cigritous_ws/install/setup.bash" >> ~/.bashrc

echo "Cigritous build done!"