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
    see README.md
******************************************************************/

#include "DHTesp.h"

DHTesp::DHTesp(uint8_t _pin, uint8_t _model) {
	pin = _pin;
	model = _model;
}

DHTesp::~DHTesp() {} // DESTRUCTOR

void DHTesp::begin()
{
	DHTesp::resetTimer(); // Make sure we do read the sensor in the next readSensor()
}

void DHTesp::resetTimer()
{
	DHTesp::lastReadTime = millis() - 3000;
}

float DHTesp::getHumidity()
{
	readSensor();
	if (error == ERROR_TIMEOUT)
	{ // Try a second time to read
		readSensor();
	}
	return humidity;
}

float DHTesp::getTemperature()
{
	readSensor();
	if (error == ERROR_TIMEOUT)
	{ // Try a second time to read
		readSensor();
	}
	return temperature;
}

void DHTesp::getTempAndHumidity()
{
	readSensor();
	if (error == ERROR_TIMEOUT)
	{ // Try a second time to read
		readSensor();
	}
}

#ifndef OPTIMIZE_SRAM_SIZE

const char *DHTesp::getStatusString()
{
	switch (error)
	{
	case DHTesp::ERROR_TIMEOUT:
		return "TIMEOUT";

	case DHTesp::ERROR_CHECKSUM:
		return "CHECKSUM";

	default:
		return "OK";
	}
}

#else

// At the expense of 26 bytes of extra PROGMEM, we save 11 bytes of
// SRAM by using the following method:

prog_char P_OK[] PROGMEM = "OK";
prog_char P_TIMEOUT[] PROGMEM = "TIMEOUT";
prog_char P_CHECKSUM[] PROGMEM = "CHECKSUM";

const char *DHTesp::getStatusString()
{
	prog_char *c;
	switch (error)
	{
	case DHTesp::ERROR_CHECKSUM:
		c = P_CHECKSUM;
		break;

	case DHTesp::ERROR_TIMEOUT:
		c = P_TIMEOUT;
		break;

	default:
		c = P_OK;
		break;
	}

	static char buffer[9];
	strcpy_P(buffer, c);

	return buffer;
}

#endif

void DHTesp::readSensor()
{
	// Make sure we don't poll the sensor too often
	// - Max sample rate DHT11 is 1 Hz   (duty cicle 1000 ms)
	// - Max sample rate DHT22 is 0.5 Hz (duty cicle 2000 ms)
	unsigned long startTime = millis();
	if ((unsigned long)(startTime - lastReadTime) < (model == DHT11 ? 999L : 1999L))
	{
		return;
	}
	lastReadTime = startTime;

	temperature = NAN;
	humidity = NAN;

	uint16_t rawHumidity = 0;
	uint16_t rawTemperature = 0;
	uint16_t data = 0;

	// Request sample
	digitalWrite(pin, LOW); // Send start signal
	pinMode(pin, OUTPUT);
	if (model == DHT11)
	{
		delay(18);
	}
	else
	{
		// This will fail for a DHT11 - that's how we can detect such a device
		delay(2);
	}

	pinMode(pin, INPUT);
	digitalWrite(pin, HIGH); // Switch bus to receive data

	// We're going to read 83 edges:
	// - First a FALLING, RISING, and FALLING edge for the start bit
	// - Then 40 bits: RISING and then a FALLING edge per bit
	// To keep our code simple, we accept any HIGH or LOW reading if it's max 85 usecs long

#ifdef ESP32
	// ESP32 is a multi core / multi processing chip
	// It is necessary to disable task switches during the readings
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);
#else
	//   cli();
	noInterrupts();
#endif
	for (int8_t i = -3; i < 2 * 40; i++)
	{
		byte age;
		startTime = micros();

		do
		{
			age = (unsigned long)(micros() - startTime);
			if (age > 90)
			{
				error = ERROR_TIMEOUT;
#ifdef ESP32
				portEXIT_CRITICAL(&mux);
#else
				// sei();
				interrupts();
#endif
				return;
			}
		} while (digitalRead(pin) == (i & 1) ? HIGH : LOW);

		if (i >= 0 && (i & 1))
		{
			// Now we are being fed our 40 bits
			data <<= 1;

			// A zero max 30 usecs, a one at least 68 usecs.
			if (age > 30)
			{
				data |= 1; // we got a one
			}
		}

		switch (i)
		{
		case 31:
			rawHumidity = data;
			break;
		case 63:
			rawTemperature = data;
			data = 0;
			break;
		}
	}

#ifdef ESP32
	portEXIT_CRITICAL(&mux);
#else
	//   sei();
	interrupts();
#endif

	// Verify checksum

	if ((byte)(((byte)rawHumidity) + (rawHumidity >> 8) + ((byte)rawTemperature) + (rawTemperature >> 8)) != data)
	{
		error = ERROR_CHECKSUM;
		return;
	}

	// Store readings

	if (model == DHT11)
	{
		humidity = (rawHumidity >> 8) + ((rawHumidity & 0x00FF) * 0.1);
		temperature = (rawTemperature >> 8) + ((rawTemperature & 0x007F) * 0.1);
		if (rawTemperature & 0x0080)
		{
			temperature = -temperature;
		}
	}
	else
	{
		humidity = rawHumidity * 0.1;

		if (rawTemperature & 0x8000)
		{
			rawTemperature = -(int16_t)(rawTemperature & 0x7FFF);
		}
		temperature = ((int16_t)rawTemperature) * 0.1;
	}

	error = ERROR_NONE;
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float DHTesp::computeDewPoint(float temperature, float percentHumidity)
{
	// reference: http://wahiduddin.net/calc/density_algorithms.htm
	double A0 = 373.15 / (273.15 + (double)temperature);
	double SUM = -7.90298 * (A0 - 1);
	SUM += 5.02808 * log10(A0);
	SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1);
	SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1);
	SUM += log10(1013.246);
	double VP = pow(10, SUM - 3) * (double)percentHumidity;
	double Td = log(VP / 0.61078); // temp var
	Td = (241.88 * Td) / (17.558 - Td);
	return Td;
}

float DHTesp::computeAbsoluteHumidity(float temperature, float percentHumidity)
{
	// Calculate the absolute humidity in g/m³
	// https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
	float absHumidity;
	float absTemperature;
	absTemperature = temperature + 273.15;

	absHumidity = 6.112;
	absHumidity *= exp((17.67 * temperature) / (243.5 + temperature));
	absHumidity *= percentHumidity;
	absHumidity *= 2.1674;
	absHumidity /= absTemperature;

	return absHumidity;
}
