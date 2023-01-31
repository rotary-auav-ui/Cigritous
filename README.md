# Welcome to Cigritous Repository!

!['cigritous logo'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/project_logo.png | height=500)  

Cigritous is a NXP Hovergames project by Vishwakarma Research Group Universitas Indonesia for reducing yield losses from pests and weather. For our initial design, please read: https://www.hackster.io/contests/nxp-hovergames-challenge-3/hardware_applications/15385

## Authors (Thanks to!)
- Thariq Hadyan (EE 23)
- Benedicto Matthew W. (CE 24)
- Raihan Syahran (EE 23)
- Abdul Fikih K. (CE 25)
- M. Rizky Millennianno (Phys 23)
- Dylan Vieri (EE 25)
- Ricky Iskandar Z. (Phys 24)
- Lauren Christy T. (CE 25)
- Juan Jonathan (CE 25)
- Muhammad Fikri (ME 24)
- Andre Christoga P. (Phys 24)
- Ahmad Rifqi (CE 25)
- Adrian Leo P. (EE 25)
- M. Daffa Aryasetya (EE 25)
- M. Fikri R. Abyadhi (EE 25)

## ALERT: WIP/Work In Progress!

## Branch Information

### Onboard Module Branch

*`main`*

Branch for drone computer programs. Contains drone visual-inertial-api, pest detector, and precision lander

Hardware:

- NXP Vehicle Drone Kit (RDDRONE-FMUK66)
- NXP i.MX 8M Plus based 8M NavQ Plus Computer

Software:

- Linux Ubuntu 22.04 LTS Jammy Jellyfish / 20.04 LTS Focal Fossa
- ROS2 Humble Hawksbill / Foxy Fitzroy
- PX4-Autopilot v1.13

# Installation Tutorial
Create ROS2 workspace

`mkdir cigritous_ws && cd cigritous_ws`

Clone the package

`git clone --recurse-submodules -b main https://github.com/rotary-auav-ui/cigritous.git`

Rename to src

`mv cigritous src`

Build the package

`colcon build`

If submodule clone fails,

`cd src`

`git submodule update --init`

`cd ..`

Then continue to build the package

### Ground Module Branch

*`ground-module`*

Branch for ground module programs. Contains `node-module` and `central-module` source codes and libraries.

The central module recieves sensors data from `node-module` using mesh network, and commanding drone to to agriculture tasks.

The node module connects with each other using mesh network and relaying sensor data to `central-module`.

Hardware:

- Bosch Sensortec Environmental Gas Sensor BME688
- ESP32 DevKit
- MQ131 Ozone Sensor
- YL-38/69 Moisture Sensor
- DHT22 Humidity and Temperature Sensor

Software:

- Arduino IDE 2.x or 1.x
- Libraries (contained in `libraries` folder):
  - painlessMesh
  - TaskScheduler
  - DHTesp
  - YL3869
  - MQUnifiedsensors
  - MQTT library
  - Bosch BME68x library

# Installation Tutorial
`git clone -b ground-module https://github.com/rotary-auav-ui/cigritous.git`

Copy all the folder in `libraries` to `Documents/Arduino/libraries`

Configure the sensors, program, and connections in `setting.h`

Flash to microcontroller