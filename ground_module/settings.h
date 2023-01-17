#include "painlessMesh.h" // DO NOT ERASE
/* 
If you want to flash central module code, use #define with CENTRAL_MODULE 
If you want to flash node module code, use #define with NODE_MODULE and set the node number
*/
#define CENTRAL_MODULE 

//Please define node number sequentially
#define NODE_NUMBER 2

#define MESH_PREFIX     "Cigritous"
#define MESH_PASSWORD   "Cigritous"
#define MESH_PORT       5555

constexpr uint8_t DHT_SENSOR_PINS[2] = {12, 13};
constexpr uint8_t MOIST_SENSOR_PINS[2] = {22, 23};

constexpr uint8_t dht_sensor_count = sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0]);
constexpr uint8_t moist_sensor_count = sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0]);
constexpr uint8_t SENSOR_COUNT = floor( (dht_sensor_count + moist_sensor_count) / 2 );

#if defined(CENTRAL_MODULE)
  #define WIFI_SSID "WifiSSID"
  #define WIFI_PASSWORD "WifiPassword"
  #define TOTAL_NODE 1
  #define MQ131_PIN 5
  #define BME_SCK 13
  #define BME_MISO 12
  #define BME_MOSI 11
  #define BME_CS 10
  // #define SEALEVELPRESSURE_HPA 1013.25
  constexpr uint64_t UPDATE_RATE = 5 * TASK_SECOND;
#else
  constexpr uint64_t REPEAT_SEND_RATE = 2 * TASK_SECOND;
  constexpr uint64_t UPDATE_RATE = 2 * TASK_HOUR;
#endif