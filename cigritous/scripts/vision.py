#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from module import CrowDetector
from module import AruCoDetector

from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleOdometry

import paho.mqtt.client as mqtt

import math

import cv2

class Vision(Node, CrowDetector, AruCoDetector):

    def __init__(self):
        super().__init__('vision')

        self.get_logger().info('Initiating precision landing program')

        # get param
        self.loop_rate = self.declare_parameter('publish_rate', 20)

        mqtt_param = self.declare_parameters(
            namespace='mqtt',
            parameters=[
                ('port', 5555),
                ('broker', '0'),
                ('username', 'a'),
                ('password', 'a')
            ])
        
        self.kp_vh = self.declare_parameter('precision_landing/kp', 0.1)
        self.precland_radius = self.declare_parameter('precision_landing/radius', 50)
        self.precland_height = self.declare_parameter('precision_landing/height', 50)

        self.precision_landing = False

        # MQTT
        self.mqttClient = mqtt.Client('navq-plus-1')
        self.mqttClient.username_pw_set(mqtt_param[2], mqtt_param[3])
        self.mqttClient.on_connect = self.connect_callback
        self.mqttClient.connect(mqtt_param[1], mqtt_param[0])

        self.mqttClient.subscribe([("/drone/flag/precision_landing", 0),
                                   ("/drone/flag/detect_crow", 0)])
        
        self.mqttClient.message_callback_add("/drone/flag/precision_landing", self.flag_precland_callback)
        self.mqttClient.message_callback_add("/drone/flag/detect_crow", self.flag_detcrow_callback)
        
        # start OpenCV things
        self.cap = cv2.VideoCapture('v4l2src device=/dev/video3 ! video/x-raw,framerate=30/1,width=640,height=480 ! appsink', 
                               cv2.CAP_GSTREAMER)

        _, frame = self.cap.read()

        self.crow_detector = CrowDetector(frame)

        self.aruco_detector = AruCoDetector(400)

        self.create_subscription(VehicleStatus, 'fmu/vehicle_status/out', self.cb_veh_sta, 10)

        self.veh_sta = VehicleStatus()

        self.pub_trj_set = self.create_publisher(
            TrajectorySetpoint, 'fmu/trajectory_setpoint/in')
        
        self.pub_ofb_mod = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in')
        
        self.pub_veh_cmd = self.create_publisher(
            VehicleCommand, 'fmu/vehicle_command/in')
        
        self.timer_main = rclpy.Timer()
        self.timer_main.cancel() # turn off before starting
        
        self.trj_set_msg = TrajectorySetpoint()
        self.ofb_mod_msg = OffboardControlMode()
        self.veh_cmd_msg = VehicleCommand()

        self.ofb_mod_msg.velocity = True
        
        self.pub_ofb_count = 0

    '''
    def get_time_ms(self):
        return int(self.get_clock().now().nanoseconds/1e3)
    '''

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

    def detect_crow_routine(self):
        _, frame = self.cap.read()
        count, _, _ = self.crow_detector.detect(frame)
        self.mqttClient.publish('/drone/crow_count', str(count))
        
    def precision_landing_routine(self):
        _, frame = self.cap.read()
        x, y = self.aruco_detector.detect(frame)
        
        #self.trj_set_msg.timestamp = self.get_time_ms()
        #self.ofb_mod_msg.timestamp = self.trj_set_msg.timestamp
        self.trj_set_msg.vy = self.kp_vh*-x
        self.trj_set_msg.vx = self.kp_vh*-y

        if self.pub_ofb_count < 5:
            self.pub_ofb_count =+ 1
        else:
            self.set_vehicle_command(self.veh_cmd_msg.VEHICLE_CMD_DO_SET_MODE,
                                     1, 6)
            
        if (math.pow(x, 2) + math.pow(y, 2) < math.pow(self.precland_radius)):
            self.set_vehicle_command(self.veh_cmd_msg.VEHICLE_CMD_NAV_LAND)
        
        self.pub_veh_cmd.publish(self.veh_cmd_msg)
        self.pub_trj_set.publish(self.trj_set_msg)
        self.pub_ofb_mod.publish(self.ofb_mod_msg)

    def cb_veh_sta(self, msg):
        self.veh_sta = msg

        prec_land_mode = self.veh_sta.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND 
        prec_land_mode = prec_land_mode and self.timer_main.is_canceled() and self.precision_landing

        if (prec_land_mode):
            self.pub_ofb_count = 0
            self.timer_main = self.create_timer(int(1/self.loop_rate), 
                                                self.precision_landing_routine)
        else:
            self.timer_main.cancel()

    def subscribe_callback(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT broker connect success")
        else:
            self.get_logger().warn("MQTT broker connect failed with %d", rc)

    def flag_precland_callback(self, client, userdata, msg):
        if (self.precision_landing != eval(msg.payload.decode())): 
            self.get_logger().info(f"Precision landing set to {self.precision_landing}")

        self.precision_landing = eval(msg.payload.decode())

if __name__ == '__main__':
    rclpy.init(args=None)

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()

    rclpy.shutdown()