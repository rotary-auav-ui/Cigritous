#include "painlessMesh.h" // DO NOT ERASE
/* 
If you want to flash central module code, use #define with CENTRAL_MODULE 
If you want to flash node module code, use #define with NODE_MODULE and set the node number
*/
#define CENTRAL_MODULE
// #define NODE_MODULE 

// Please define node number sequentially starting from 1 (excluding central)
#define NODE_NUMBER 1
#define TOTAL_NODE  1  // Total number of all nodes excluding central module

#define MESH_PREFIX     "Cigritous"
#define MESH_PASSWORD   "Cigritous"
#define MESH_PORT       5555
#define NETWORK_CHANNEL 13


constexpr uint8_t DHT_SENSOR_PINS[1] = {25};
constexpr uint8_t MOIST_SENSOR_PINS[1] = {26};

constexpr uint8_t dht_sensor_count = sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0]);
constexpr uint8_t moist_sensor_count = sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0]);
constexpr uint8_t SENSOR_COUNT = floor( (dht_sensor_count + moist_sensor_count) / 2 );

#if defined(CENTRAL_MODULE)
  #define MQTT_BROKER   "driver.cloudmqtt.com" 
  #define MQTT_USERNAME "cbobzrgp"             
  #define MQTT_PASSWORD "CKvOQLxrtuqc"         
  #define MQTT_CLIENTID "ESP32Central"
  #define MQTT_PORT     18789
  #define WIFI_SSID     "VTOL"
  #define WIFI_PASSWORD "87654321"
  #define TEMP_THRES    35 // In Celsius (°C). How hot the air needs to be before watering
  #define HUMID_THRES   40 // In percentage. How dry the air needs to be before watering
  #define MOIST_THRES   40 // In percentage. How dry the soil needs to be before watering
  #define SENSOR_THRES  3  // How many nodes must be below threshold for the drone to start watering
  #define FLY_ALTITUDE  3  // In meters. Height of the drone when watering relative to home position
  constexpr uint64_t UPDATE_RATE = 2 * TASK_SECOND;
#else
  constexpr uint64_t REPEAT_SEND_RATE = 2 * TASK_SECOND;
  constexpr uint64_t UPDATE_RATE = 2 * TASK_HOUR;
#endif

