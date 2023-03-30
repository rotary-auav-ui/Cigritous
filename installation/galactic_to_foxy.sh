#!/bin/bash
echo "Resetting .bashrc"
sudo -S cp /etc/skel/.bashrc ~/.bashrc

source ~/.bashrc

echo "Detecting ROS2 Galactic. Uninstalling..."
sudo -S apt -y remove ~nros-galactic-* && sudo apt -y autoremove

sudo -S apt -y update
sudo -S apt -y autoremove
# Consider upgrading for packages previously shadowed.
sudo -S apt -y upgrade

sh ./foxy_install.sh