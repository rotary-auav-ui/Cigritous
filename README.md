# Cigritous Ground Module Branch

## Installation

1. Press `< > Code` button near `Add file`
2. Press `Download ZIP`
3. Extract
4. Copy `ground_module` and `libraries` folder to `<username>\Documents\Arduino` (Windows)
5. Open `ground_module.ino` inside ground_module folder using Arduino IDE
6. Setup your sensors in `settings.h`
7. Select `ground_module.ino`

![Arduino button](https://arduinotogo.com/wp-content/uploads/2016/07/ch3-buttons-labelled.png)

(Credits to arduinotogo.com)

9. Press `upload` button to flash the code into microcontroller
10. Wait until complete. Enjoy!

## Information

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
  - Arduino-MAVLink-ESP32
  - MQUnifiedsensors
  - arduino-mqtt
  - Bosch BME68x library

#### `ground-module`

Branch for ground module programs. Contains `node-module` and `central-module` source codes and libraries.

The central module recieves sensors data from `node-module` using mesh network, and commanding drone to to agriculture tasks.

The node module connects with each other using mesh network and relaying sensor data to `central-module`.
