# Welcome to Cigritous Repository!

!['cigritous logo'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/project_logo.png)  

Cigritous is a NXP Hovergames project by Vishwakarma AUAV UI for reducing yield losses from pests and weather. For full implementation please read: https://www.hackster.io/contests/nxp-hovergames-challenge-3/hardware_applications/15385

## ALERT: WIP/Work In Progress!

## Branch Information

### Onboard /  Branch

*`main`*

Branch for drone computer programs. Contains drone visual-inertial-api, pest detector, and precision lander

Hardware:

- NXP Vehicle Drone Kit (RDDRONE-FMUK66)
- NXP i.MX 8M Plus based 8M NavQ Plus Computer

Software:

- Linux Ubuntu 22.04 LTS Jammy Jellyfish
- ROS2 Humble Hawksbill
- PX4-Autopilot v1.13


### Ground Branch

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