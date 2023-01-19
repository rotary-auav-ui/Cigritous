#include "settings.h"

#if defined(CENTRAL_MODULE)
// === CENTRAL MODULE SECTION SOURCE CODE ===

//latitude and longitude of home and sensor. Change to static float when able to read from backend
constexpr float HOME_LOC[2] = {6.124125, -6.12412512};
constexpr float SENSOR_LOC[TOTAL_NODE * SENSOR_COUNT][2] = {{6.1523124, -6.412512}}; 
constexpr float fly_altitude = 2; //relative to home

//sensors
#include "MQUnifiedsensor.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//sending to backend
#include "EspMQTTClient.h"

//mavlink
#include <mavlink_commands.hpp>
#include <memory>

#define BOARD "ESP32"
#define V_RES 3.3
#define ADC_BIT 12

#define RAT_MQ131_CA 15

#define NUM_OF_MISSION 3

Adafruit_BME680 bme;
MQUnifiedsensor MQ131(BOARD, V_RES, ADC_BIT, MQ131_PIN, "MQ-131");

Scheduler mainscheduler; // task scheduler

painlessMesh mesh;

std::shared_ptr<Task> send_msg_task;

std::shared_ptr<Mavlink> mavlink;

EspMQTTClient client(
  WIFI_SSID,
  WIFI_PASSWORD,
  "192.168.1.100", // Ip of broker server
  "Cigritous",
  "Cigritous",
  "Central",
  1883
);

uint8_t i = 0;
// static float home_location[2], sensor_loc[SENSOR_COUNT][2]; //read from backend in init_sensor_location

// //backend informations 
// unsigned long epochTime; 
// unsigned long dataMillis = 0;

// const char* ntpServer = "id.pool.ntp.org";
// const char* serverName = "https://data.mongodb-api.com/app/data-kmljr/endpoint/ESP32";

// StaticJsonDocument<JSON_OBJECT_SIZE(5)> central;
// StaticJsonDocument<JSON_OBJECT_SIZE(5) + JSON_ARRAY_SIZE(TOTAL_NODE * SENSOR_COUNT) + (TOTAL_NODE * SENSOR_COUNT) * JSON_OBJECT_SIZE(4)> sensors;

struct {
  uint8_t sensor_id;
  uint8_t node_id;
  float humid;
  float moisture;
  float temp;
  float ozone;
  float gas;
} sensData;

void init_sensor_locations(){
  //read from backend, for now keep static
}

void parseMsg(const String& msg) {
  static size_t pos[3];
  pos[0] = msg.indexOf('/'); //1
  sensData.sensor_id = msg.substring(0, pos[0]).toInt();
  pos[1] = msg.indexOf("/", pos[0]+1); //7
  sensData.humid = msg.substring(pos[0]+1, pos[1]-1).toFloat();
  pos[2] = msg.indexOf("/", pos[1]+1);
  sensData.temp = msg.substring(pos[1]+1, pos[2]-1).toFloat();
  sensData.moisture = msg.substring(pos[2]+1, msg.length()).toFloat();
}

// // Function that gets current epoch time
// unsigned long getTime() {
//   time_t now;
//   struct tm timeinfo;
//   if (!getLocalTime(&timeinfo)) {
//     return(0);
//   }
//   time(&now);
//   return now;
// }

// void POSTData(){
//   if(WiFi.status() == WL_CONNECTED){
//     HTTPClient http;

//     http.begin(serverName);
//     http.addHeader("Content-Type", "application/json");

//     String json;
//     serializeJson(sensors, json);

//     Serial.println(json);
//     int httpResponseCode = http.POST(json);
//     Serial.println(String(httpResponseCode));

//     if (httpResponseCode == 200) {
//       Serial.println("Data uploaded.");
//       delay(200);
//     } else {
//       Serial.println("ERROR: Couldn't upload data.");
//       delay(200);
//     }

//   }
// }

void receivedCallback(uint32_t from, const String& msg ) {
  Serial.printf("RECV: %u msg=%s\n", from, msg.c_str());
  if (msg != "S") { // keep parsing when recieving
    parseMsg(msg);
    sensData.node_id = from;
  }
  else mesh.sendSingle(from, "A"); // acknowledge/ACK message
}

void sendMsgRoutine() {

  // TODO: add BME688 readings
  bme.performReading();

  MQ131.update();

  sensData.ozone = MQ131.readSensorR0Rs();

  // TODO : Set thresholds for certain plants and saves their respective sensor id.

  // TODO: send to back-end

  // if (millis() - dataMillis > 15000 || dataMillis == 0){
  //   dataMillis = millis();

  //   epochTime = getTime();

  //   // get sensor number
  //   uint8_t id = ((sensData.node_id - 1) - 1) * SENSOR_COUNT + sensData.sensor_id;

  //   sensors[id]["dht_temperature"] = sensData.temp;
  //   sensors[id]["dht_humidity"] = sensData.humid;
  //   sensors[id]["dht_moisture"] = sensData.moisture;
  //   sensors[id]["dht_timestamp"] = epochTime;

  //   sensors["bme_temperature"] = bme.temperature;
  //   sensors["bme_pressure"] = bme.pressure / 100.0;
  //   sensors["bme_humidity"] = bme.humidity;
  //   sensors["bme_gas"] = bme.gas_resistance / 1000.0;
  //   Serial.printf("%f %f %f %f %f %f %f\n", bme.temperature, bme.pressure, bme.humidity, bme.gas_resistance, sensData.temp, sensData.humid, sensData.moisture);
  //   sensors["ozone"] = sensData.ozone;

  //   Serial.println("Uploading data... "); 
  //   POSTData();
  // }
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  client.publish("central/bme/temp", String(bme.temperature));
  client.publish("central/bme/press", String(bme.pressure));
  client.publish("central/bme/humid", String(bme.humidity));

}

void setup() {
  Serial.begin(115200);

  mavlink = std::make_shared<Mavlink>(115200, 16, 17); // Using UART2
  mavlink->req_data_stream();
  mavlink->read_data();

  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.print("Connecting to Wi-Fi");
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //     Serial.print(".");
  //     delay(300);
  // }
  // Serial.println();
  // Serial.print("Connected with IP: ");
  // Serial.println(WiFi.localIP());
  // Serial.println();

  // configTime(0, 0, ntpServer);

  // Receive heartbeat
  // while(mavlink->get_px_status() != MAV_STATE_STANDBY){
  //   Serial.println("Pixhawk not on standby!");
  //   mavlink->read_data();
  //   delay(300);
  // }

  send_msg_task = std::make_shared<Task>(UPDATE_RATE, TASK_FOREVER, sendMsgRoutine);

  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  MQ131.setRegressionMethod(1);
  MQ131.setA(23.943);
  MQ131.setB(-1.11);
  MQ131.init();

  Serial.print("Calibrating gas sensor, please wait.");
  float calcR0_131 = 0;
  float calcR0_2 = 0;
  for(i = 1; i<=10; i ++) {
    MQ131.update();
    calcR0_131 += MQ131.calibrate(RAT_MQ131_CA);
    Serial.print(".");
    delay(1);
  }
  Serial.println(".");
  MQ131.setR0(calcR0_131/10);

  Serial.println("Gas sensor calibration complete");

  if(isinf(calcR0_131)) {Serial.println("Warning: Conection issue on MQ131, R0 is infinite (Open circuit detected) please check your wiring and supply");}
  if(calcR0_131 == 0){Serial.println("Warning: Conection issue found on MQ131, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}
  
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, mainscheduler, MESH_PORT, WIFI_AP, 1); // Central node number will always be 1
  mesh.onReceive(&receivedCallback);

  mainscheduler.addTask(ConvTask::getFromShared(send_msg_task));
  send_msg_task->enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();

  client.loop();
}

// === END OF CENTRAL MODULE SOURCE CODE SECTION ===
#else
// === NODE MODULE SECTION SOURCE CODE ===

#include <yl3869.h>
#include "DHTesp.h"

#include <memory>
#include <vector>

#define uS_TO_S 1E+6
#define uS_TO_HOURS 3.6E+9

Scheduler mainscheduler;  // main task scheduler

painlessMesh mesh;

std::shared_ptr<Task> send_msg_task;

std::shared_ptr<DHTesp> dht[SENSOR_COUNT];
std::shared_ptr<YL3869> yl3869[SENSOR_COUNT];

float humid[SENSOR_COUNT], temp[SENSOR_COUNT];
float moisture[SENSOR_COUNT];
uint8_t sensor_id[SENSOR_COUNT];

uint8_t i;

bool central_status;
bool queue_status = false;

String packMsg(uint8_t idx) {
  return (String(sensor_id[idx]) + "/" + String(humid[idx]) + "/" + String(temp[idx]) + "/" + String(moisture[idx]));
}

void readSensorRoutine() {
  for (i = 0; i < SENSOR_COUNT; i++) {
    humid[i] = dht[i]->getHumidity();
    temp[i] = dht[i]->getTemperature();  
    moisture[i] = 32;  //yl3869[i].read();
  }

  Serial.println("Sensor data read");

  if (mesh.isConnected(1)) Serial.println("Connected to central module. Sending");

  i = 0;
  send_msg_task->set(TASK_MILLISECOND, TASK_FOREVER, sendMsgRoutine);
}

void sendMsgRoutine() {

  if (i == SENSOR_COUNT) {
    mesh.sendSingle(1, "S");  // confirmation message
    Serial.println("SEND: S");
    send_msg_task->delay(REPEAT_SEND_RATE);
    return;
  }

  if (!mesh.isConnected(1)) {  // are we connected to central module?
    Serial.println("Not connected to central module. Queueing");
    queue_status = true;
    send_msg_task->cancel();
    return;
  }

  mesh.sendSingle(1, packMsg(i));
  Serial.println(packMsg(i).c_str());
  i++;
}

void newConnectionCallback(uint32_t nodeId) {
  
  if ((nodeId == 1) && (queue_status == true)) {
    Serial.println("Central module connection acquired, sending queued data");
    send_msg_task->restart();
    queue_status = false;
  }
}

// Needed for painless library
void receivedCallback(uint32_t from, const String& msg) {
  Serial.printf("RECV: %u msg=%s\n", from, msg.c_str());
  if (msg == "A") {
    Serial.println("Message sent. Change to Deep Sleep Mode");
    mesh.stop();
    esp_sleep_enable_timer_wakeup(UPDATE_RATE);
    esp_deep_sleep_start();
  }
}

void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
  
  for(i = 0; i < SENSOR_COUNT; i++) {
    sensor_id[i] = i+1;
    
    dht[i] = std::make_shared<DHTesp>(DHT_SENSOR_PINS[i], models::DHT11); // add define DHT models later
    dht[i]->begin();
    
    // yl3869[i] = std::make_shared<YL3869>(MOIST_SENSOR_PINS[i]);
    // yl3869[i]->init();
  }

  send_msg_task = std::make_shared<Task>(TASK_MILLISECOND, TASK_ONCE, readSensorRoutine);
  
  mesh.init(MESH_PREFIX, MESH_PASSWORD, mainscheduler, MESH_PORT, WIFI_AP_STA, NODE_NUMBER);  // node number can be changed from settings.h
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(newConnectionCallback);

  mainscheduler.addTask(ConvTask::getFromShared(send_msg_task));
  send_msg_task->enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}

// === END OF NODE MODULE SOURCE CODE SECTION ===
#endif