#include "settings.h"

#if defined(CENTRAL_MODULE)
// === CENTRAL MODULE SECTION /SOURCE CODE ===

//sensors
#include "MQUnifiedsensor.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//sending to backend
#include <WiFi.h>
#include <MQTT.h>

//mavlink
#include <mavlink_commands.hpp>
#include <memory>
#include <array>

#define BOARD "ESP32"
#define V_RES 3.3
#define ADC_BIT 12

Adafruit_BME680 bme;

Scheduler mainscheduler; // task scheduler

painlessMesh mesh;

WiFiClient wifiClient;
MQTTClient mqttClient;

std::shared_ptr<Task> send_msg_task;
std::shared_ptr<Task> mavlink_task;
std::shared_ptr<Task> waypoint_task;

std::shared_ptr<MAVLink> mavlink;

uint8_t i = 0;
uint8_t total_id, node, sens;
uint8_t queue[SENSOR_THRES] = {0};
bool full;
static float sensor_loc[TOTAL_NODE * SENSOR_COUNT][2]; //read from web

struct Node{
  uint8_t id;
  float humid;
  float moisture;
  float temp;
  float gas;
}sensData[TOTAL_NODE * SENSOR_COUNT];

struct Central{
  float humid;
  float temp;
  float presure;
  float gas;
}central;

void parseMsg(const String& msg) {
  static size_t pos[3];
  pos[0] = msg.indexOf('/');
  pos[1] = msg.indexOf("/", pos[0]+1);
  pos[2] = msg.indexOf("/", pos[1]+1);
  sens = msg.substring(0, pos[0]).toInt();
  sensData[sens - 1].id = sens;
  sensData[sens - 1].temp = msg.substring(pos[1]+1, pos[2]-1).toFloat();
  sensData[sens - 1].moisture = msg.substring(pos[2]+1, msg.length()).toFloat();
  sensData[sens - 1].humid = msg.substring(pos[0]+1, pos[1]-1).toFloat();
  Serial.println(
    String(sensData[sens - 1].id) + " " +
    String(sensData[sens - 1].temp) + " " +
    String(sensData[sens - 1].moisture) + " " +
    String(sensData[sens - 1].humid) + " "
  );
}


void receivedCallback(uint32_t from, const String& msg ) {
  Serial.printf("RECV: %u msg=%s\n", from, msg.c_str());
  if (msg != "S") { // keep parsing when recieving
    node = from;
    parseMsg(msg);
  }
  else mesh.sendSingle(from, "A"); // acknowledge/ACK message
}

void sendMsgRoutine() {

  if(!bme.performReading()){
    Serial.println("BME688 failed to perform reading!");
  }

  publish_sensor_data();

  for(i = 0; i < TOTAL_NODE * SENSOR_COUNT; i++){
    if(sensData[i].temp > TEMP_THRES && 
      sensData[i].humid < HUMID_THRES && 
      sensData[i].moisture < MOIST_THRES
    ){
      for(i = 0; i < SENSOR_THRES; i++){
        if(queue[i] == 0){ 
          queue[i] = sensData[i].id - 1;
          if(i == SENSOR_THRES){ 
            for(i = 0; i < SENSOR_THRES; i++){
              mavlink->add_waypoint(sensor_loc[queue[i]][0], sensor_loc[queue[i]][1]);
              queue[i] = 0;
            }
            mavlink->send_mission();
          }
          return;
        }
      }
      Serial.println("Queue is full!");
    }
  }
}

bool reconnect() {
  // Loop until we're reconnected
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected!");
    return false;
  }
  if (!mqttClient.connected()) { // use if, not while so process not blocking
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      mqttClient.publish("/hello", "Hello from central");
      // ... and resubscribe
      for(i = 1; i <= TOTAL_NODE * SENSOR_COUNT; i++){
        mqttClient.subscribe("/" + String(i) + "/latitude");
        mqttClient.subscribe("/" + String(i) + "/longitude");
      }
      // mqttCsubscribe(MQTT_SERIAL_RECEIVER_CH);
      return true;
    } else {
      Serial.println("failed, try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      return false;
    }
  }
  return true;
}

void subscribe_cb(String& topic, String& payload) {

  // for debugging purposes
  Serial.println("incoming: " + topic + " - " + payload);
  if(topic.equals("/drone/take_land")){
    Serial.println("Takeoff and land manually is unsupported");
    return;
  }

  static uint8_t parser, key;
  static float coor;
  parser = topic.lastIndexOf('/');
  key = topic.substring(1, parser).toInt();
  String val = String(topic.substring(parser + 1, topic.length()));
  coor = payload.toFloat();
  if(val.equals("latitude")){
    sensor_loc[key - 1][0] = coor;
    Serial.printf("Node %u latitude set to %f\n", key, coor);
  }else if(val.equals("longitude")){
    sensor_loc[key - 1][1] = coor;
    Serial.printf("Node %u longitude set to %f\n", key, coor);
  }
}

void publish_sensor_data(){
  if(!reconnect()) return; // use guard condition
  
  Serial.println("Publishing sensor data");

  static std::array<int32_t, 2> home;
  home = mavlink->get_home_pos_curr();
  mqttClient.publish("/central/lat", (String(home[0])).c_str());
  mqttClient.publish("/central/lng", (String(home[1])).c_str());

  mqttClient.publish("/central/temp", (String(bme.temperature)).c_str());
  mqttClient.publish("/central/press", (String(bme.pressure)).c_str());
  mqttClient.publish("/central/humid", (String(bme.humidity)).c_str());
  mqttClient.publish("/central/gas", (String(bme.gas_resistance)).c_str());
  static std::array<int32_t, 3> pose;
  pose = mavlink->get_global_pos_curr();
  mqttClient.publish("/drone/lat", String(pose[0]).c_str());
  mqttClient.publish("/drone/lng", String(pose[1]).c_str());
  mqttClient.publish("/drone/alt", String(pose[2]).c_str());
  
  static std::array<float, 3> vel;
  vel = mavlink->get_velocity_curr();
  mqttClient.publish("/drone/vx", String(vel[0]).c_str());
  mqttClient.publish("/drone/vy", String(vel[1]).c_str());
  mqttClient.publish("/drone/vz", String(vel[2]).c_str());

  mqttClient.publish("/drone/time", String(mavlink->get_time_boot()).c_str());
  mqttClient.publish("/drone/yaw_curr", String(mavlink->get_yaw_curr()).c_str());
  
  mqttClient.publish("/drone/progress", String(mavlink->get_mis_reached()).c_str()); 
  mqttClient.publish("/drone/battery", String(mavlink->get_battery_status()).c_str()); 
  mqttClient.publish("/drone/status", String((int)mavlink->get_armed()).c_str()); 

  for(i = 1; i <= TOTAL_NODE * SENSOR_COUNT; i++){
    mqttClient.publish(("/" + String(i) + "/temp").c_str(), String(sensData[i - 1].temp).c_str());
    mqttClient.publish(("/" + String(i) + "/humid").c_str(), String(sensData[i - 1].humid).c_str());
    mqttClient.publish(("/" + String(i) + "/moist").c_str(), String(sensData[i - 1].moisture).c_str());
  }
}

void recieveMavlink() {
  mavlink->send_heartbeat();
}

void setup() {
  Serial.begin(115200);

  mavlink = std::make_shared<MAVLink>(57600, 16, 17); // Using UART2

  mavlink->set_fly_alt(FLY_ALTITUDE);

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, mainscheduler, MESH_PORT, WIFI_AP_STA, 1, NETWORK_CHANNEL); // Central node number will always be 1
  mesh.onReceive(&receivedCallback);
  mesh.stationManual(WIFI_SSID, WIFI_PASSWORD);
  mesh.setHostname("Cigritous");

  mesh.setRoot(true);
  mesh.setContainsRoot(true);

  if(!bme.begin()){
    Serial.println("BME is not connected!");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  send_msg_task = std::make_shared<Task>(UPDATE_RATE, TASK_FOREVER, sendMsgRoutine);
  mavlink_task = std::make_shared<Task>(TASK_SECOND, TASK_FOREVER, recieveMavlink);

  mainscheduler.addTask(ConvTask::getFromShared(send_msg_task));
  mainscheduler.addTask(ConvTask::getFromShared(mavlink_task));

  send_msg_task->enable();
  mavlink_task->enable();

  mqttClient.begin(MQTT_BROKER, MQTT_PORT, wifiClient);
  mqttClient.onMessage(subscribe_cb);
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  mqttClient.loop();
  mavlink->read_data();
}

// === END OF CENTRAL MODULE SOURCE CODE SECTION ===
#else
// === NODE MODULE SECTION SOURCE CODE ===

#include "DHTesp.h"

#include <memory>
#include <vector>

#define uS_TO_S 1E+6
#define uS_TO_HOURS 3.6E+9

Scheduler mainscheduler;  // main task scheduler

painlessMesh mesh;

std::shared_ptr<Task> send_msg_task;

std::shared_ptr<DHTesp> dht[SENSOR_COUNT];

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
    // dummy data
    // humid[i] = 30 + 2 * i;
    // temp[i] = 30 + 2 * i;  
    // moisture[i] = 30 + 2 * i; 
    for(int j = 0; j < 5; j++){
      humid[i] = dht[i]->getHumidity();
      temp[i] = dht[i]->getTemperature(); 
      moisture[i] = analogRead(26) * 80.0 / 4096; 
    }
    Serial.printf("Humid : %f\nTemperature %f \nMoisture : %f %\n", humid[i], temp[i], moisture[i]);
  }
  Serial.println("Sensor data read");

  if (mesh.isConnected(1)) Serial.println("Connected to central module. Sending");

  i = 0;
  sendMsgRoutine();
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
    sensor_id[i] = ((NODE_NUMBER - 1) * SENSOR_COUNT) + (i + 1);
    
    dht[i] = std::make_shared<DHTesp>(DHT_SENSOR_PINS[i], models::DHT11); // add define DHT models later
    dht[i]->begin();

  }

  send_msg_task = std::make_shared<Task>(TASK_SECOND, TASK_FOREVER, readSensorRoutine);
  
  mesh.init(MESH_PREFIX, MESH_PASSWORD, mainscheduler, MESH_PORT, WIFI_AP_STA, NODE_NUMBER, NETWORK_CHANNEL);  // node number can be changed from settings.h
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(newConnectionCallback);
  mesh.setContainsRoot(true);

  mainscheduler.addTask(ConvTask::getFromShared(send_msg_task));
  send_msg_task->enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}

// === END OF NODE MODULE SOURCE CODE SECTION ===
#endif