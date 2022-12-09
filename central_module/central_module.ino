#include "painlessMesh.h"
#include "MQUnifiedsensor.h"
#include "SerialTransfer.h"

#include <memory>
#include <cstring>

#define MESH_PREFIX     "meshSSID"
#define MESH_PASSWORD   "meshPassword"
#define MESH_PORT       5555

#define BOARD "ESP32"
#define V_RES 3.3
#define ADC_BIT 12

#define RAT_MQ131_CA 15
#define RAT_MQ2_CA 9.83

#define NODE_NUMBER 1

MQUnifiedsensor MQ2(BOARD, V_RES, ADC_BIT, 12, "MQ-2");
MQUnifiedsensor MQ131(BOARD, V_RES, ADC_BIT, 13, "MQ-131");

Scheduler mainscheduler; // task scheduler

painlessMesh mesh;

std::shared_ptr<Task> send_msg_task;

SerialTransfer transfer;

uint8_t i = 0;

void parseMsg(const String& msg) {
  char* temp = std::strtok((char*)msg.c_str(), "|");
  static String val[4];
  i = 0;
  while (temp) {
    val[i] = temp;
    temp = std::strtok(nullptr, "|");  
    i++;
  }
  sensData.sensor_id = val[0].toInt();
  sensData.humid = val[1].toFloat();
  sensData.moisture = val[2].toFloat();
  sensData.temp = val[3].toFloat();
  delete temp;
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

  MQ2.update();
  MQ131.update();

  sensData.gas = MQ2.readSensor();
  sensData.ozone = MQ131.readSensorR0Rs();

  // send to MQTT
}

void setup() {
  Serial.begin(115200);

  send_msg_task = std::make_shared<Task>(TASK_SECOND * 5, TASK_FOREVER, sendMsgRoutine);

  MQ131.setRegressionMethod(1);
  MQ131.setA(23.943);
  MQ131.setB(-1.11);
  MQ131.init();

  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25);
  MQ2.setB(-2.222);
  MQ2.init();

  Serial.print("Calibrating gas sensor, please wait.");
  float calcR0_131, calcR0_2; // declare in setup because one time only
  calcR0_131 = 0;
  calcR0_2 = 0;
  for(i = 1; i<=10; i ++) {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    MQ131.update();
    calcR0_131 += MQ131.calibrate(RAT_MQ131_CA);
    calcR0_2 += MQ2.calibrate(RAT_MQ2_CA);
    Serial.print(".");
    delay(1);
  }
  Serial.println(".");
  MQ131.setR0(calcR0_131/10);
  MQ2.setR0(calcR0_2/10);

  Serial.println("Gas sensor calibration complete");

  if(isinf(calcR0_131)) {Serial.println("Warning: Conection issue on MQ131, R0 is infinite (Open circuit detected) please check your wiring and supply");}
  if(calcR0_131 == 0){Serial.println("Warning: Conection issue found on MQ131, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}
  
  if(isinf(calcR0_2)) {Serial.println("Warning: Conection issue on MQ2, R0 is infinite (Open circuit detected) please check your wiring and supply");}
  if(calcR0_2 == 0){Serial.println("Warning: Conection issue found on MQ2, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}

  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, mainscheduler, MESH_PORT, WIFI_AP, NODE_NUMBER); // 1 is node ID
  mesh.onReceive(receivedCallback);

  mainscheduler.addTask(ConvTask::getFromShared(send_msg_task));
  send_msg_task->enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}