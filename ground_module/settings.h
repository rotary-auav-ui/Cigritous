#include "painlessMesh.h" // DO NOT ERASE
/* 
If you want to flash central module code, use #define with CENTRAL_MODULE 
If you want to flash node module code, use #define with NODE_MODULE and set the node number
*/
#define CENTRAL_MODULE 

//Please define node number sequentially
#define NODE_NUMBER 2

// #define MESH_PREFIX     "JuanJonathan"
// #define MESH_PASSWORD   "123juan123;'"
// #define MESH_PORT       5555
#define WIFI_SSID "JuanJonathan"
#define WIFI_PASSWORD "123juan123;'"
#define MQTT_PORT 18789


//Don't use pin 12
constexpr uint8_t DHT_SENSOR_PINS[1] = {25};
constexpr uint8_t MOIST_SENSOR_PINS[1] = {22};

constexpr uint8_t dht_sensor_count = sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0]);
constexpr uint8_t moist_sensor_count = sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0]);
constexpr uint8_t SENSOR_COUNT = floor( (dht_sensor_count + moist_sensor_count) / 2 );

#if defined(CENTRAL_MODULE)
  #define MQTT_SERVER "driver.cloudmqtt.com"
  #define MQTT_USERNAME "cbobzrgp"
  #define MQTT_PASSWORD "CKvOQLxrtuqc"
  #define SENSOR_THRES 3
  #define TEMP_THRES 35
  #define HUMID_THRES 40
  #define MOIST_THRES 40
  #define TOTAL_NODE 1
  #define MQ131_PIN 5
  constexpr uint64_t UPDATE_RATE = 5 * TASK_SECOND;
#else
  constexpr uint64_t REPEAT_SEND_RATE = 2 * TASK_SECOND;
  constexpr uint64_t UPDATE_RATE = 2 * TASK_HOUR;
#endif