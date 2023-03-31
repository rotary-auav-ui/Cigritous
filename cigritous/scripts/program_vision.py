#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import detector

from px4_msgs.msg import TrajectorySetpoint, VehicleStatus, VehicleControlMode
from px4_msgs.msg import OffboardControlMode, VehicleCommand

import paho.mqtt.client as mqtt
import math
import numpy as np
import cv2

class Vision(Node):

    def __init__(self):
        super().__init__('program_vision')

        self.get_logger().info('Initiating vision program')

        # ==== PARAMETERS SECTION ====
        # please modify this according to your setup
        loop_rate = 20
        mqtt_param = {
            'port': 18789,
            'broker': 'driver.cloudmqtt.com',
            'username': 'cbobzrgp',
            'password': 'CKvOQLxrtuqc'
            }        
        
        self.kp_vh = 0.1
        self.precland_radius = 50 # in pixel

        crowdet_param = {
            'model_name': 'crow-int8.tflite',
            'confidence_threshold': 0.25,
            'iou_threshold': 0.45
        }
        
        capture_param = {
            'width_capture': 640,
            'height_capture': 480,
            'framerate': 30,
            'address': 0
        }

        # ==== END OF PARAMETERS SECTION ====
        
        self.precland_flag = False
        self.detcrow_flag = True
        
        # start OpenCV things
        framerate = f'framerate={capture_param["framerate"]}/1'
        resolution = f'width={capture_param["width_capture"]},height={capture_param["height_capture"]}'
        address = f'/dev/video{capture_param["address"]}'
        video_capture_command = 'v4l2src device=' + address + ' ! video/x-raw,' + framerate + ',' + resolution
        video_capture_command = video_capture_command + ' ! appsink'
        self.get_logger().info("Capture address: %s" % video_capture_command)

        self.loop_time = 1/loop_rate
        
        # capture from video
        self.cap = cv2.VideoCapture(video_capture_command, cv2.CAP_GSTREAMER)

        self.crow_detector = detector.CrowML(crowdet_param["model_name"], 
                                             crowdet_param["confidence_threshold"], 
                                             crowdet_param["iou_threshold"])

        self.apriltags_detector = detector.AprilTags(250)

        self.create_subscription(VehicleStatus, 'fmu/vehicle_status/out', self.cb_veh_sta, 10)
        self.create_subscription(VehicleControlMode, 'fmu/vehicle_control_mode/out', self.cb_veh_ctl_mod, 10)

        self.veh_sta = VehicleStatus()
        self.veh_ctl_mod = VehicleControlMode()

        self.pub_trj_set = self.create_publisher(
            TrajectorySetpoint, 'fmu/trajectory_setpoint/in', 10)
        
        self.pub_ofb_mod = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in', 10)
        
        self.pub_veh_cmd = self.create_publisher(
            VehicleCommand, 'fmu/vehicle_command/in', 10)

        self.trj_set_msg = TrajectorySetpoint()
        self.ofb_mod_msg = OffboardControlMode()
        self.veh_cmd_msg = VehicleCommand()

        self.ofb_mod_msg.velocity = True
        
        self.pub_ofb_count = 0
        self.idle_counter = 0

        # init MQTT
        self.MQTTClient = mqtt.Client('navq-plus')
        
        # register callback
        self.MQTTClient.on_connect = self.new_conn_callback
        self.MQTTClient.message_callback_add("drone/flag/precision_landing", self.flag_precland_callback)
        self.MQTTClient.message_callback_add("drone/flag/detect_crow", self.flag_detcrow_callback)

        self.MQTTClient.subscribe("drone/#")

        self.get_logger().info("MQTT broker: %s, port: %d" % (mqtt_param["broker"], mqtt_param["port"]))
        self.get_logger().info("MQTT username: %s, pass: %s" % (mqtt_param["username"], mqtt_param["password"]))

        self.MQTTClient.username_pw_set(mqtt_param["username"], mqtt_param["password"])

        self.MQTTClient.subscribe("drone/#")

        # connect & subs MQTT in new thread
        self.timer_mqtt = self.create_timer(0.01,
                                            self.mqtt_loop)
        
        self.MQTTClient.connect(mqtt_param["broker"], mqtt_param["port"])

        self.timer_crow = self.create_timer(self.loop_time,
                                            self.detect_crow_routine)
        self.timer_crow.cancel()
        
        self.timer_precland = self.create_timer(self.loop_time,
                                                self.precision_landing_routine)
        self.timer_precland.cancel()
        self.get_logger().info("Program vision initialized")
        
    def mqtt_loop(self):        
        self.MQTTClient.loop_start()

        if (self.detcrow_flag and self.timer_crow.is_canceled()):
            self.timer_crow.reset()
        elif (not self.detcrow_flag):
            self.timer_crow.cancel()

    def get_time_ms(self):
        return int(self.get_clock().now().nanoseconds/1e3)

    def set_vehicle_command(self, cmd, para1 = None, para2 = None):
        if (self.veh_cmd_msg.command == cmd):
            return
        
        self.veh_cmd_msg.command = cmd
        self.veh_cmd_msg.param1 = para1
        self.veh_cmd_msg.param2 = para2
        self.veh_cmd_msg.source_system = 1
        self.veh_cmd_msg.source_component = 1
        self.veh_cmd_msg.target_system = self.veh_sta.system_id
        self.veh_cmd_msg.target_component = self.veh_sta.component_id
        self.veh_cmd_msg.from_external = True

        self.pub_veh_cmd.publish(self.veh_cmd_msg)

    def detect_crow_routine(self):
        _, frame = self.cap.read()
        start = self.get_time_ms()
        count = self.crow_detector.detect(frame)
        self.get_logger().info("inference time: %.1fs" % ((self.get_time_ms() - start)/1e3))

        self.get_logger().info("crow: %d" % count) # debug

        self.MQTTClient.publish('drone/crow_count', str(count))
        
    def precision_landing_routine(self):
        _, frame = self.cap.read()
        x, y = self.apriltags_detector.detect(frame)

        if (x == np.nan):
            self.get_logger().info("no target") # debug
            return
        else:
            self.get_logger().info("x: %d, y: %d" % (x, y)) # debug
        
        #self.trj_set_msg.timestamp = self.get_time_ms()
        #self.ofb_mod_msg.timestamp = self.trj_set_msg.timestamp
        self.trj_set_msg.vy = self.kp_vh*-x
        self.trj_set_msg.vx = self.kp_vh*-y

        # set offboard after sending bunch of setpoints
        if self.pub_ofb_count < 0.2: # 0.2 seconds
            self.pub_ofb_count =+ self.loop_time
        else:
            if self.idle_counter < 1.5 and (math.pow(x, 2) + math.pow(y, 2) < math.pow(self.precland_radius)):
                self.set_vehicle_command(self.veh_cmd_msg.VEHICLE_CMD_DO_SET_MODE,
                                        1, 6)
                self.idle_counter =+ self.loop_time
            else:
                self.set_vehicle_command(self.veh_cmd_msg.VEHICLE_CMD_NAV_LAND)
        
        self.pub_trj_set.publish(self.trj_set_msg)
        self.pub_ofb_mod.publish(self.ofb_mod_msg)

    def cb_veh_sta(self, msg):
        self.veh_sta = msg

        prec_land_start = self.veh_sta.failsafe and self.precland_flag

        if (prec_land_start and self.timer_precland.is_canceled()):
            self.pub_ofb_count = 0
            self.timer_crow.cancel()
            self.timer_precland.reset()
        elif (prec_land_start and (not self.veh_ctl_mod) and (self.timer_crow.is_canceled())):
            self.timer_precland.cancel()
            self.timer_crow.reset()

    def cb_veh_ctl_mod(self, msg):
        self.veh_ctl_mod = msg

    def new_conn_callback(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT broker connect success")
        else:
            self.get_logger().warn("MQTT broker connect failed with %d" % rc)

    def flag_precland_callback(self, client, userdata, msg):
        self.precland_flag == eval(msg.payload.decode())
        self.get_logger().info("Precision landing set to ", self.precland_flag)

    def flag_detcrow_callback(self, client, userdata, msg):
        self.detcrow_flag == eval(msg.payload.decode())
        self.get_logger().info("Crow detection set to ", self.detcrow_flag)

if __name__ == '__main__':
    rclpy.init(args=None)

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()

    rclpy.shutdown()
