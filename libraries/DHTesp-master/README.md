DHTesp [![Build Status](https://github.com/beegee-tokyo/DHTesp/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/beegee-tokyo/DHTesp/actions)
===
<!-- [![Build Status](https://travis-ci.com/beegee-tokyo/DHTesp.svg?branch=master)](https://travis-ci.com/beegee-tokyo/DHTesp) -->

_**This library is no longer maintained**_

An Arduino library for reading the DHT family of temperature and humidity sensors.    
Forked from [arduino-DHT](https://github.com/markruys/arduino-DHT)     
Original written by Mark Ruys, <mark@paracas.nl>.    

Why did I clone this library instead of forking the original repo and push the changes?
When I searched through Github for DHT libraries, I found a lot of them, some of them offers additional functions, some of them only basic temperature and humidity values. I wanted to combine all interesting functions into one library. In addition, none of the DHT libraries I found were written to work without errors on the ESP32. For ESP32 (a multi core/ multi processing SOC) task switching must be disabled while reading data from the sensor.    
Another problem I found is that many of the available libraries use the same naming (dht.h, dht.cpp), which easily leads to conflicts if different libraries are used for different platforms.    

_**According to users, the library works as well with DHT33 and DHT44 sensors. But as I do not own these sensors, I cannot test and confirm it. However, if you want to use this sensors, you can do so by using `setup(pin, DHTesp::DHT22)` and it should work.
Please give me feedback in the issues if you successfull use these sensors.
Thank you**_

The library is tested as well on ESP8266 and should work on AVR boards as well.    

Changes to the original library:
--------
- 2017-12-12: Renamed DHT class to DHTesp and filenames from dht.* to DHTesp.* to avoid conflicts with other libraries - beegee-tokyo, <beegee@giesecke.tk>.    
- 2017-12-12: Updated to work with ESP32 - beegee-tokyo, <beegee@giesecke.tk>.   
- 2017-12-12: Added function computeHeatIndex. Reference: [Adafruit DHT library](https://github.com/adafruit/DHT-sensor-library).    
- 2017-12-14: Added function computeDewPoint. Reference: [idDHTLib](https://github.com/niesteszeck/idDHTLib).    
- 2017-12-14: Added function getComfortRatio. Reference: [libDHT](https://github.com/ADiea/libDHT). (References about Human Comfort invalid)    
- 2017-12-15: Added function computePerception. Reference: [WikiPedia Dew point==> Relationship to human comfort](https://en.wikipedia.org/wiki/Dew_point) - beegee-tokyo, <beegee@giesecke.tk>.   
- 2018-01-02: Added example for multiple sensors usage.    
- 2018-01-03: Added function getTempAndHumidity which returns temperature and humidity in one call.    
- 2018-01-03: Added retry in case the reading from the sensor fails with a timeout.    
- 2018-01-08: Added ESP8266 (and probably AVR) compatibility.    
- 2018-03-11: Updated DHT example    
- 2018-06-19: Updated DHT example to distinguish between ESP8266 examples and ESP32 examples    
- 2018-07-06: Fixed bug in ESP32 example    
- 2018-07-17: Use correct field separator in keywords.txt    
- 2019-03-07: Added computeAbsoluteHumidity which returns the absolute humidity in g/m³. Reference: [How to convert relative humidity to absolute humidity](https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/) kudos to [Wurstnase](https://github.com/Wurstnase)    
- 2019-03-22: Fixed auto detection problem    
- 2019-07-31: Make getPin() public, Updated ESP8266 example        
- 2019-10-01: Using noInterrupts() & interrupts() instead of cli and sei
- 2019-10-05: Reduce CPU usage and add decimal part for DHT11 (thanks to Swiftyhu)
- 2019-10-06: Back to working version by removing the last commit
- 2021-02-20: Fix negative temperature problem (credits @helijunky)
- 2022-12-09: Erase human based calculation (heat index, Fahrenheit-Celsius conversion, etc). Also delete comfort classification calculation

Features
--------
  - Support for DHT11 and DHT22, AM2302, RHT03
  - Determine dewpoint
  - Determine thermal comfort:
    * More info at [Determining Thermal Comfort Using a Humidity and Temperature Sensor](https://www.azosensors.com/article.aspx?ArticleID=487)
  - Determine human perception based on humidity, temperature and dew point according to Horstmeyer, Steve (2006-08-15). [Relative Humidity....Relative to What? The Dew Point Temperature...a better approach](http://www.shorstmeyer.com/wxfaqs/humidity/humidity.html)
