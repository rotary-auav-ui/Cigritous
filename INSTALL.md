# Installation Guide

### Easy Install
We've created the install script `install_env.sh` for easy setup, and follow these command:

```bash
wget https://github.com/rotary-auav-ui/cigritous/blob/main/cigritous/installation/install_env.sh

/bin/bash install_env.sh
```

### Application Setup

Cigritous has some parameters you need to assign accordingly to perform without problems.
There's 3 files you must edit:

#### `cigritous/config` directory:
- `vio_bridge.yaml`

Contains settings for VIO program setup, such as topic name, rate, and rotation settings

#### `cigritous/scripts` directory:
- `program_vision.py`

On the `# ==== PARAMETERS SECTION ====` section, you must change MQTT settings according to your
system and the camera settings that if setup poorly will render whole program unusable

#### `cigritous/launch` directory:
- `vio_bridge.py`
- `vision.py`

The FCU USB address may needs to be configured in both launch file in order for NavQ+ to communicate
with FMUK66 FCU

### Building Program

Build using script:

```bash
/bin/bash ~/cigritous_ws/src/installation/build.sh
```

#### Autostart install

---
##### OPTIONAL: Add VIO bridge to autostart 

```bash
gedit ~/cigritous_ws/src/installation/autoinstall/cigritous_autostart.sh
```

Add this command at the last line

`ros2 launch cigritous vio_bridge.cpp`

Then save
---

To make the program autostart, run these:

```bash
/bin/bash ~/cigritous_ws/src/installation/install_autostart.sh
```

### Full Install Procedure

#### Prerequisites
1. Ubuntu for NXP NavQ+

[NXP NavQ+ flash OS tutorial](https://nxp.gitbook.io/8mpnavq/dev-guide/software/setup-guide-emmc)

Images:

20.04 Focal Fossa: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v20.04.1

22.04 Jammy Jellyfish: https://github.com/rudislabs/navqplus-create3-images/releases/tag/v22.04.2

2. ROS2 Install

Tested in 20.04 + ROS2 Foxy. Compiled but not tested in 22.04 + ROS2 Humble,
**do it at your own risk!**

**Install according to your Ubuntu version!**

Ubuntu 20.04 - Foxy Fitzroy: https://docs.ros.org/en/foxy/Installation.html

Ubuntu 22.04 - Humble Hawksbill: https://docs.ros.org/en/humble/Installation.html

3. PX4-ROS2 hub

[PX4-ROS2 v1.13 installation tutorial](https://docs.px4.io/v1.13/en/ros/ros2_comm.html#ros-2-user-guide-px4-ros-2-bridge)

4. Python dependencies

Run below commands in linux terminal:

```bash

pip3 install tflite-runtime opencv-python paho-mqtt 

pip3 install pep517

pip3 install pupil-apriltags

```

# Troubleshooting

## Submodule clone fails (px4_msgs and px4_ros_com error)

```bash
cd src
git submodule update --init
cd ..
```

Then build the package again

```bash
cd ~/cigritous_ws

colcon build
```

