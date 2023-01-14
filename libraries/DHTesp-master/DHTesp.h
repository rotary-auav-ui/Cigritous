/******************************************************************
  DHT Temperature & Humidity Sensor library for Arduino & ESP32.

  Features:
  - Support for DHT11 and DHT22/AM2302/RHT03
  - Auto detect sensor model
  - Very low memory footprint
  - Very small code

  https://github.com/beegee-tokyo/arduino-DHTesp

  Written by Mark Ruys, mark@paracas.nl.
  Updated to work with ESP32 by Bernd Giesecke, bernd@giesecke.tk

  GNU General Public License, check LICENSE for more information.
  All text above must be included in any redistribution.

  Datasheets:
  - http://www.micro4you.com/files/sensor/DHT11.pdf
  - http://www.adafruit.com/datasheets/DHT22.pdf
  - http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/RHT03.pdf
  - http://meteobox.tk/files/AM2302.pdf

  Changelog:
    See README.md
 ******************************************************************/

#ifndef dhtesp_h
#define dhtesp_h

#if ARDUINO < 100
  #include <WProgram.h>
#else
  #include <Arduino.h>
#endif

enum models {
  DHT11,
  DHT22,
  AM2302,  // Packaged DHT22
  RHT03    // Equivalent to DHT22
};

class DHTesp
{
public:
  typedef enum {
    ERROR_NONE = 0,
    ERROR_TIMEOUT,
    ERROR_CHECKSUM
  } DHT_ERROR_t;

  DHTesp(uint8_t _pin, uint8_t _model=DHT22);
  ~DHTesp();

  void begin();
  void resetTimer();

  float getTemperature();
  float getHumidity();
  void getTempAndHumidity();

  const char* getStatusString();

  float computeDewPoint(float temperature, float percentHumidity);
  float computeAbsoluteHumidity(float temperature, float percentHumidity);

  float temperature;
  float humidity;
  uint8_t error;

protected:
  void readSensor();
  uint8_t pin;
  
private:
  unsigned long lastReadTime;
  uint8_t model;
};

#endif /*dhtesp_h*/
