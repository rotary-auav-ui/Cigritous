# Installation Tutorial

## Prerequisites
1. Ubuntu for NXP NavQ+

20.04 Focal Fossa: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v20.04.1

22.04 Jammy Jellyfish: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v22.04.2

[OS installation tutorial](https://nxp.gitbook.io/8mpnavq/dev-guide/software/setup-guide-emmc)

2. ROS2 Install

Tested in 20.04 + ROS2 Foxy. Compiled but not tested in 22.04 + ROS2 Humble,
**do it at your own risk!**

**Install according to your Ubuntu version!**

Ubuntu 20.04 - Foxy Fitzroy: https://docs.ros.org/en/foxy/Installation.html

Ubuntu 22.04 - Humble Hawksbill: https://docs.ros.org/en/humble/Installation.html

3. PX4-ROS2 hub

[PX4-ROS2 installation tutorial](https://docs.px4.io/main/en/ros/ros2_comm.html)

# Installation

### Easy Setup!
We've created the install script for easy setup, just download the `clean_setup.sh` then run it:

```bash
wget https://github.com/rotary-auav-ui/cigritous/blob/main/cigritous/installation/clean_setup.sh

chmod +x clean_setup.sh

./clean_setup.sh
```

And just wait until complete

### Setup Commands

Run these commands:

```bash

pip3 install tflite-runtime opencv-python paho-mqtt pupil-apriltags

cd ~/

mkdir cigritous_ws && cd cigritous_ws

git clone --recurse-submodules -b main https://github.com/rotary-auav-ui/cigritous.git

mv cigritous src

colcon build

```

For autostart this program every boot:

```bash
/bin/bash ./install_autostart.sh
```

**If submodule clone fails,**

```bash
cd src
git submodule update --init
cd ..
```

Then continue build the package again

# Usage

