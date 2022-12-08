# cigritous
# ALERT: WIP/Work In Progress!
## Branch Information
#### Ground Branches
Hardware:
- Bosch Sensortec Environmental Gas Sensor BME688
- ESP32 DevKit
- MQ131 Ozone Sensor
- YL-38/69 Moisture Sensor
- SIM900A GSM Module
- DHT22 Humidity and Temperature Sensor
Software:
- Arduino IDE 2.x or 1.x
##### `central-module`
Repository for central module. Recieves sensor nodes data with mesh and communicates with drone by telemetry

##### `node-module`
Repository for sensor nodes module. Connected by mesh network to central module

#### Drone Branch
Hardware:
- NXP Vehicle Drone Kit (RDDRONE-FMUK66)
- NXP i.MX 8M Plus based 8M NavQ Plus Computer
##### `main`
Repository for drone computer. Contains drone 'summon' system, waypoint code, and machine learning algorithm to track pests such as crows
