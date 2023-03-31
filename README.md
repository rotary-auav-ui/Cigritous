# Welcome to Cigritous Repository!

!['cigritous logo'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/new_project_logo.png)  

Cigritous is a NXP Hovergames 3 project by *Vishwakarma Aerial Dexterity Research Group Universitas Indonesia* **for reducing yield losses from pests and weather by utilizing automated drone responder and remote monitoring system.** 

For detailed concept, please read: [Crop Monitoring with Automated UAV Spray Response](https://www.hackster.io/contests/nxp-hovergames-challenge-3/hardware_applications/15385)

## Authors
- Thariq Hadyan (Electrical Engineering 23)
- Benedicto Matthew W. (Computer Engineering 24)
- Raihan Syahran (Electrical Engineering 23)
- Abdul Fikih K. (Computer Engineering 25)
- M. Rizky Millennianno (Physics 23)
- Dylan Vieri (Computer Engineering 25)
- Ricky Iskandar Z. (Physics 24)
- Lauren Christy T. (Computer Engineering 25)
- Juan Jonathan (Computer Engineering 25)
- Muhammad Fikri (Mechanical Engineering 24)
- Andre Christoga P. (Physics 24)
- Ahmad Rifqi (Computer Engineering 25)
- Adrian Leo P. (Electrical Engineering 25)
- M. Daffa Aryasetya (Electrical Engineering 25)
- M. Fikri R. Abyadhi (Electrical Engineering 25)

## Branch Information

### Onboard Program Branch

*`main`*

Branch for drone computer programs. Contains drone *visual-inertial*, *pest detector*, and *precision landing* program

Hardware:

- NXP Vehicle Drone Kit (RDDRONE-FMUK66)
- NXP i.MX 8M Plus based 8M NavQ Plus Computer
- Google Coral 5MP Camera

Software:

- Linux Ubuntu 20.04 LTS Focal Fossa / 22.04 LTS Jammy Jellyfish
- ROS2 Foxy Fitzroy / Humble Hawksbill
- PX4-Autopilot v1.13.3

#### Installation and Usage
[Click Here](https://github.com/rotary-auav-ui/cigritous/blob/main/INSTALL.md)  

#### Detection Performance using NXP NavQ+
!['crow detected'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/detection.png)  

Average detection time: 2800ms or 2.8s 
!['detection time'](https://github.com/rotary-auav-ui/cigritous/blob/main/docs/detection.png)  

### Sensor Network / Ground Module Branch

*`ground-module`*

Branch for sensor network or ground module programs. Contains *sensor nodes* source codes and libraries.

The sensor network has 2 main part: **central** and **node**. The nodes are connected together by mesh and connected to sensors to sample ground data. Sampled data will be sent to central, which acts as 'back end' and communicating to onboard computer and FCU via TCP & MAVLink.

Hardware:

- Bosch Sensortec Environmental Gas Sensor BME688
- ESP32 DevKit
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
  - arduino-mqtt
  - Bosch BME68x library

#### Installation and Usage
[Read Me!](https://github.com/rotary-auav-ui/cigritous/blob/ground-module/README.md)  

### Web Server Branch

*`web_server`*

Branch for webserver front end. Acts as Web-based Ground Control Station which display agriculture and drone data from sensor network. **The system is backend-less because direct sensor network communication using MQTT**. *Tested in Heroku*.

#### Installation and Usage
[Read Me!](https://github.com/rotary-auav-ui/cigritous/blob/web_server/README.md)  
