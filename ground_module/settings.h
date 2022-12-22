/* 
If you want to flash central module code, use #define with CENTRAL_MODULE 
If you want to flash node module code, use #define with NODE_MODULE and set the node number
*/
#define NODE_MODULE 

#define NODE_NUMBER 5

#define MESH_PREFIX     "meshSSID"
#define MESH_PASSWORD   "meshPassword"
#define MESH_PORT       5555

#if defined(CENTRAL_MODULE)
  #define MQ131_PIN 5
#else
  const int DHT_SENSOR_PINS[2] = {12, 13};
  const int MOIST_SENSOR_PINS[2] = {22, 23};
  constexpr int SENSOR_COUNT = ( (sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0])) + (sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0])) ) / 2;
#endif