#include "settings.h"

#if defined(CENTRAL_MODULE)
// === CENTRAL MODULE SECTION SOURCE CODE ===
constexpr uint8_t dht_sensor_count = sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0]);
constexpr uint8_t moist_sensor_count = sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0]);
constexpr uint8_t SENSOR_COUNT = floor( (dht_sensor_count + moist_sensor_count) / 2 );

//latitude and longitude of home and sensor. Change to static float when able to read from backend
constexpr float HOME_LOC[2] = {6.124125, -6.12412512};
constexpr float SENSOR_LOC[SENSOR_COUNT][2] = {{6.1523124, -6.412512}, {6.165124, -6.2512412}}; 
constexpr float fly_altitude = 2; //relative to home

#include "MQUnifiedsensor.h"

#include <mavlink.h>
#include <memory>

#define BOARD "ESP32"
#define V_RES 3.3
#define ADC_BIT 12

#define RAT_MQ131_CA 15

//TODO: Add BME688
MQUnifiedsensor MQ131(BOARD, V_RES, ADC_BIT, MQ131_PIN, "MQ-131");

Scheduler mainscheduler; // task scheduler

painlessMesh mesh;

std::shared_ptr<Task> send_msg_task;

uint8_t i = 0;
uint8_t px_mode;
uint8_t px_status;
uint16_t seq_prev = 1000, seq = 1000; // might not be needed for mission_item_request idk...
uint8_t mis_up_status;
// static float home_location[2], sensor_loc[SENSOR_COUNT][2]; //read from backend in init_sensor_location


struct {
  uint8_t sensor_id;
  uint8_t node_id;
  float humid;
  float moisture;
  float temp;
  float ozone;
  float gas;
} sensData;

struct{
  uint8_t sys_id = 255; // qgc id
  uint8_t comp_id = 2; // any?
  uint8_t tgt_sys = 1; // id of pxhawk = 1
  uint8_t tgt_comp = 1; // 0 broadcast, 1 work juga
}sys;

void init_sensor_locations(){
  //read from backend, for now keep static
}

void req_status(){ //DEPRECATED SINCE 2015, BUT STILL RECOMMENDED TO USE BY PX4/Ardupilot, SUPPOSEDLY REPLACED BY MAV_CMD_SET_MESSAGE_INTERVAL.
  sys.sys_id = 255;
  sys.comp_id = 2;
  sys.tgt_sys = 1;
  sys.tgt_comp = 1;
  uint8_t req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t req_msg_rate = 0x01; // 1 times per second
  uint8_t start_stop = 1; // 1 start, 0 = stop

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, req_stream_id, req_msg_rate, start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  Serial1.write(buf, len);
}

void send_mission_count(const int& num_of_mission){
  Serial.printf("Sending mission count: %d\n", (num_of_mission + 3));
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_MISSION_COUNT_LEN];
  uint16_t count = num_of_mission + 3; 
  uint8_t mission_type = MAV_MISSION_TYPE_MISSION;

  mavlink_msg_mission_count_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, count, mission_type);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);  
}

void send_mission_items(const int& num_of_mission, const int id[num_of_mission]){
  send_mission_count(num_of_mission);

  for(int i = 0; i < SENSOR_COUNT; i++){
    seq_prev = seq;
    while(seq == seq_prev){
      Serial.printf("Waiting for mission sequence %u request sent by px\n", seq);
      read_data();
    }
    Serial.printf("Setting waypoint lat : %f, lng : %f, height : %f\n", SENSOR_LOC[id[i]][0], SENSOR_LOC[id[i]][1], fly_altitude);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MSG_ID_MISSION_ITEM_LEN];

    uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; //lat, long, altitude is relative to home altitude in meters
    uint8_t command = 16; //waypoint
    uint8_t current = 0;
    uint8_t cont = 1;
    float param1 = 1;
    float param2 = 1;
    float param3 = 0;
    float param4 = NAN;
    int32_t lat = SENSOR_LOC[id[i]][0] * 1e7;
    int32_t lng = SENSOR_LOC[id[i]] * 1e7;
    float height = fly_altitude;
    uint8_t mission_type = MAV_MISSION_TYPE_MISSION;

    mavlink_msg_mission_item_int_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, seq, frame, command, current, cont, param1, param2, param3, param4, x, y, z, mission_type);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial1.write(buf, len);

    Serial.printf("Mission sequence %u sent", seq);
  }

  while(mis_up_status != MAV_MISSION_ACCEPTED){
    read_data();
  }

  start_mission();
}

void start_mission(){
  Serial.println("Starting mission");

  takeoff();
  
  /*
  If it takes off correctly but doesn't start mission, may need MAV_CMD_COMMAND_START here.
  Documentation says that drone will automatically start mission when switched to auto mode,
  with condition that mission is accepted.
  */

  /*
  Loops while not all mission item is completed. 
  Supposedly, there is MISSON_STATE enum with MISSION_STATE_COMPLETE, but how do you request
  for it???? Or is it automatic?????????
  */
  while(seq_prev != seq){ 
    read_data();
  }

  return_to_launch();

  land();

  set_mode(MAV_MODE_STABILIZE_DISARMED);
}

void arm(){
  Serial.println("Arming");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 400; //arm disarm
  uint8_t conf = 0;
  float param1 = 1; //arm = 1

  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, param1, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
}

void disarm(){
  Serial.println("Disarming");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 400; //arm disarm
  uint8_t conf = 0;
  float param1 = 0; //disarm = 0

  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, param1, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
}

void takeoff(){
  while(px_mode != MAV_MODE_FLAG_SAFETY_ARMED){
    arm();
  }

  while(px_mode != MAV_MODE_FLAG_AUTO_ENABLED){
    set_mode(MAV_MODE_AUTO_ARMED);
  }

  Serial.println("Taking off");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 22; //takeoff
  uint8_t conf = 0;
  float param7 = fly_altitude;

  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, 0, 0, 0, 0, 0, 0, param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial.write(buf, len);
}

void land(){
  Serial.println("Taking off");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 21; //land
  uint8_t conf = 0;

  //altitude determined by param7, here it is 0 relative to frame.
  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial.write(buf, len);

  disarm();
}

void set_mode(const uint16_t& mode){
  Serial.println("Setting mode to auto");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 176; //do set mode
  uint8_t conf = 0;
  float param1 = mode; //auto disarmed

  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, param1, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial1.write(buf, len);
}

void return_to_launch(){
  Serial.println("Returning to launch");

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN];

  uint16_t command = 20; //return to launch
  uint8_t conf = 0;

  mavlink_msg_command_long_pack(sys.sys_id, sys.comp_id, &msg, sys.tgt_sys, sys.tgt_comp, command, conf, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
}

void read_data(){
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial1.available())
  {
    uint8_t c = Serial1.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
          check_mode(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
          mission_request(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ACK:
          uploaded_mission_status(&msg);
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
          check_mission_progress(&msg);
          break;
      }
    }
  }
}

void check_mode(mavlink_message_t& msg){
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(msg, &hb);
  px_mode = hb.base_mode;
  px_status = hb.system_status;
}

void mission_request(mavlink_message_t& msg){
  mavlink_mission_request_int_t mis_req;
  mavlink_msg_mission_request_int_decode(msg, &mis_req);
  seq_prev = seq; //retain previous sequence number
  seq = mis_req.seq;
  sys.tgt_sys = mis_req.target_system;
  sys.tgt_comp = mis_req.target_component;
  Serial.printf("Requesting for mission type %u sequence %u\n", mis_req.mission_type, seq);
}

void check_mission_progress(mavlink_message_t& msg){
  mavlink_mission_item_reached_t it;
  mavlink_msg_mission_item_reached_decode(msg, &it);
  seq_prev = it.seq; //using same variable to save memory
  Serial.printf("Mission sequence %u reached\n", seq_prev);
}

void uploaded_mission_status(mavlink_message_t& msg){
  mavlink_mission_ack_t mis_ack;
  mavlink_msg_mission_ack_decode(msg, mis_ack);
  mis_up_status = mis_ack.type;
  if(mis_up_status == MAV_MISSION_ACCEPTED){
    Serial.println("Mission accepted");
  }else{
    Serial.printf("Mission unaccepted with enum %u\n", mis_up_status);
  }
}

void parseMsg(const String& msg) {
  static size_t pos[3];
  pos[0] = msg.indexOf('/');
  sensData.sensor_id = msg.substring(0, pos[0]).toInt();
  pos[1] = msg.indexOf("/", pos[0]+1);
  sensData.humid = msg.substring(pos[0]+1, pos[1]-pos[0]-1).toFloat();
  pos[2] = msg.indexOf("/", pos[1]+1);
  sensData.moisture = msg.substring(pos[1]+1, pos[2]-pos[1]-1).toFloat();
  sensData.temp = msg.substring(pos[2]+1, msg.length()).toFloat();
}

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
  MQ131.update();

  sensData.ozone = MQ131.readSensorR0Rs();

  // TODO: send to back-end
}

void setup() {
  Serial.begin(115200);

  read_data_stream(); //request data from pixhawk (heartbeat, response, vehicle status, etc)

  while(px_status != MAV_STATE_STANDBY){
    Serial.println("Pixhawk not on standby!");
    read_data();
    delay(2000);
  }

  send_msg_task = std::make_shared<Task>(UPDATE_RATE, TASK_FOREVER, sendMsgRoutine);

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
}

// === END OF CENTRAL MODULE SOURCE CODE SECTION ===
#else
// === NODE MODULE SECTION SOURCE CODE ===
constexpr uint8_t dht_sensor_count = sizeof(DHT_SENSOR_PINS)/sizeof(DHT_SENSOR_PINS[0]);
constexpr uint8_t moist_sensor_count = sizeof(MOIST_SENSOR_PINS)/sizeof(MOIST_SENSOR_PINS[0]);
constexpr uint8_t SENSOR_COUNT = floor( (dht_sensor_count + moist_sensor_count) / 2 );

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
    humid[i] = 83;     //dht[i-1].getHumidity();
    temp[i] = 20;      //dht[i-1].getTemperature();  
    moisture[i] = 32;  //yl3869[i-1].read();
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
    
    dht[i] = std::make_shared<DHTesp>(DHT_SENSOR_PINS[i], models::DHT22); // add define DHT models later
    dht[i]->begin();
    
    yl3869[i] = std::make_shared<YL3869>(MOIST_SENSOR_PINS[i]);
    yl3869[i]->init();
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