# Welcome to Cigritous GROUND MODULE Branch!

## Information

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