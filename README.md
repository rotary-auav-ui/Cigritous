# Welcome to Cigritous Repository!

!['cigritous logo'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/project_logo.png)  

Cigritous is a NXP Hovergames project by Vishwakarma AUAV UI for reducing yield losses from pests and weather. For full implementation please read: https://www.hackster.io/contests/nxp-hovergames-challenge-3/hardware_applications/15385

## ALERT: WIP/Work In Progress!

## Branch Information

### Ground Branch

Hardware:

- Bosch Sensortec Environmental Gas Sensor BME688
- ESP32 DevKit
- MQ131 Ozone Sensor
- YL-38/69 Moisture Sensor
- SIM900A GSM Module
- DHT22 Humidity and Temperature Sensor

Software:

- Arduino IDE 2.x or 1.x
- Libraries (contained in `libraries` folder):
  - painlessMesh
  - TaskScheduler
  - DHTesp
  - YL3869
  - MQUnifiedsensors
  - Bosch BME68x library

#### `ground-module`

Branch for ground module programs. Contains `node-module` and `central-module` source codes and libraries.

The central module recieves sensors data from `node-module` using mesh network, and commanding drone to to agriculture tasks.

The node module connects with each other using mesh network and relaying sensor data to `central-module`.

### Drone Branch

Hardware:

- NXP Vehicle Drone Kit (RDDRONE-FMUK66)
- NXP i.MX 8M Plus based 8M NavQ Plus Computer

Software:

- Linux Ubuntu 20.04 Focal Fossa
- ROS2 Foxy Fitzroy
- PX4-Autopilot v1.12.3

#### `main`

Repository for drone computer. Contains drone 'summon' system, waypoint code, and machine learning algorithm to track crows as pest
