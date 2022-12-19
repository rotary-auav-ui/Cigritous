#define NODE_MODULE // Option: CENTRAL_MODULE or NODE_MODULE

#define NODE_NUMBER 5

#define MESH_PREFIX     "meshSSID"
#define MESH_PASSWORD   "meshPassword"
#define MESH_PORT       5555

#if defined(CENTRAL_MODULE)
  #define MQ131_PIN 5
#else
  int DHT_SENSOR_PINS[2] = {12, 13};
  int MOIST_SENSOR_PINS[2] = {22, 23};
#endif