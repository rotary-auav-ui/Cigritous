# Installation Tutorial

## Prerequisites
1. Ubuntu for NXP NavQ+

20.04 Focal Fossa: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v20.04.1

22.04 Jammy Jellyfish: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v22.04.2

[OS installation tutorial](https://nxp.gitbook.io/8mpnavq/dev-guide/software/setup-guide-emmc)

2. ROS2 Install

**Note: ROS2 Foxy & Galactic has deprecated, but still working**

**Install according to your Ubuntu version!**

Ubuntu 20.04 - Foxy Fitzroy: https://docs.ros.org/en/foxy/Installation.html

Ubuntu 22.04 - Humble Hawksbill: https://docs.ros.org/en/humble/Installation.html

3. PX4-ROS2 hub

[PX4-ROS2 installation tutorial](https://docs.px4.io/main/en/ros/ros2_comm.html)

4. Python Dependencies

```bash
pip3 install tflite-runtime opencv-python paho-mqtt
```

# Installation

Run these commands:

```bash

cd ~/

mkdir cigritous_ws && cd cigritous_ws

git clone --recurse-submodules -b main https://github.com/rotary-auav-ui/cigritous.git

mv cigritous src

colcon build
```

**If submodule clone fails,**

```bash
cd src
git submodule update --init
cd ..
```

Then continue build the package again

# Usage

