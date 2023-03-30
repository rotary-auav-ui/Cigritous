#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import detector

from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from px4_msgs.msg import OffboardControlMode, VehicleCommand

import paho.mqtt.client as mqtt
import math
import numpy as np
import threading
import cv2

class Vision(Node):

    def __init__(self):
        super().__init__('vision')

        self.get_logger().info('Initiating vision program')

        # get param
        loop_rate = self.declare_parameter('publish_rate', 20).value
        self.loop_time = 1/loop_rate

        mqtt_param = self.declare_parameters(
            namespace='mqtt',
            parameters=[
                ('port', 18789),
                ('broker', 'driver.cloudmqtt.com')
            ])
        
        self.kp_vh = self.declare_parameter('precision_landing/kp', 0.1).value
        self.precland_radius = self.declare_parameter('precision_landing/radius', 50).value
        self.precland_height = self.declare_parameter('precision_landing/height', 50).value

        crowdet_param = self.declare_parameters(
            namespace='object_detection',
            parameters=[
                ('model_name', 'crow-int8.tflite'),
                ('confidence_threshold', 0.25),
                ('iou_threshold', 0.45)
            ])
        
        capture_param = self.declare_parameters(
            namespace='camera',
            parameters=[
                ('width_capture', 640),
                ('height_capture', 480),
                ('framerate', 30)
            ])
        
        self.precland_flag = False
        self.detcrow_flag = False
        
        # start OpenCV things
        framerate = f'framerate={capture_param[2].value}/1'
        resolution = f'width={capture_param[0].value},height={capture_param[1].value}'
        video_capture_command = 'v4l2src device=/dev/video3 ! video/x-raw,' + framerate + ',' + resolution
        video_capture_command = video_capture_command + ' ! appsink'
        
        # capture from video
        self.cap = cv2.VideoCapture(video_capture_command, cv2.CAP_GSTREAMER)
        
        # image detecting for debug purposes
        #frame = cv2.imread('/home/rizkymille/cigritous_ws/src/docs/apriltags/apriltag36h11.png')

        self.crow_detector = detector.CrowML(crowdet_param[0].value, 
                                             crowdet_param[1].value, 
                                             crowdet_param[2].value)

        self.apriltags_detector = detector.AprilTags(400)

        self.create_subscription(VehicleStatus, 'fmu/vehicle_status/out', self.cb_veh_sta, 10)

        self.veh_sta = VehicleStatus()

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

        # connect & subs MQTT in new thread
        threading.Thread(target=self.mqtt_loop, args=(mqtt_param, )).start()

        self.timer_main = self.create_timer(self.loop_time,
                                            self.detect_crow_routine())
        self.timer_main.cancel() # turn off before starting
        
    def mqtt_loop(self, mqtt_param):
        self.get_logger().info("%s %d" % (mqtt_param[1].value, mqtt_param[0].value))
        self.MQTTClient.connect(str(mqtt_param[1].value), mqtt_param[0].value)
        
        self.MQTTClient.subscribe("drone/#")

        self.MQTTClient.loop_forever()


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
        
        self.get_logger().info("x: %d, y: %d" % (x, y)) # debug
        
        if (x == np.nan):
            self.get_logger().info("no target") # debug
            return
        
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

        if not self.arming_state:
            self.timer_main = self.create_timer(self.loop_time,
                                            self.detect_crow_routine())

    def cb_veh_sta(self, msg):
        self.veh_sta = msg

        prec_land_mode = self.veh_sta.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND 
        prec_land_mode = prec_land_mode and self.timer_main.is_canceled() and self.precland_flag

        if (prec_land_mode):
            self.pub_ofb_count = 0
            self.timer_main = self.create_timer(self.loop_time, 
                                                self.precland_flag_routine())
        else:
            self.timer_main.cancel()

    def new_conn_callback(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT broker connect success")
        else:
            self.get_logger().warn("MQTT broker connect failed with %d" % rc)

    def flag_precland_callback(self, client, userdata, msg):
        if (self.precland_flag == eval(msg.payload.decode())):
            return
         
        self.get_logger().info("Precision landing set to ", self.precland_flag)
        self.precland_flag = eval(msg.payload.decode())

    def flag_detcrow_callback(self, client, userdata, msg):
        if (self.detcrow_flag == eval(msg.payload.decode())): 
            return 
        
        self.get_logger().info("Crow detection set to ", self.detcrow_flag)
        self.detcrow_flag = eval(msg.payload.decode())
        if(self.detcrow_flag):
            self.timer_main = self.create_timer(self.loop_time,
                                                self.precland_flag_routine())
        else:
            self.timer_main.cancel()

if __name__ == '__main__':
    rclpy.init(args=None)

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()

    rclpy.shutdown()
