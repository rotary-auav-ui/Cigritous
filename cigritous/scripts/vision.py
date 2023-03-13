#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from module import CrowDetector
from module import AruCoDetector

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

import cv2

class VisionProcess(Node, CrowDetector, AruCoDetector):

    def __init__(self):
        super().__init__('precision_landing')

        # TODO: connect ke TCP ESP-MESH dan kirim

        TCP_PORT = 5555
        #STATION_IP = 192.168.1.1

        self.precision_landing = True # TODO: ganti sama rosparam

        self.get_logger().info('Initiating precision landing program')

        self.loop_rate = 30  # in hertz

        self.cap = cv2.VideoCapture('v4l2src device=/dev/video3 ! video/x-raw,framerate=30/1,width=640,height=480 ! appsink', 
                               cv2.CAP_GSTREAMER)

        _, frame = self.cap.read()

        self.crow_detector = CrowDetector(frame)

        if (self.precision_landing):
            self.aruco_detector = AruCoDetector(400)

        self.create_subscription(
            VehicleStatus, 'fmu/vehicle_status/out', self.cb_veh_sta, 10)

        self.pub_trj_set = self.create_publisher(
            TrajectorySetpoint, 'fmu/trajectory_setpoint/in')
        
        self.timer_main = rclpy.Timer()
        self.timer_main.cancel() # turn off before starting
        
        self.trj_set_msg = TrajectorySetpoint()

    def detect_crow_routine(self):
        _, frame = self.cap.read()
        count, x, y = self.crow_detector.detect(frame)
        
    def precision_landing_routine(self):
        _, frame = self.cap.read()
        count, x, y = self.aruco_detector.detect(frame)
        # TODO lanjutin program precision, pake apriltags (jika sempat)
        self.trj_set_msg.vy = x
        self.trj_set_msg.vx = y
        self.trj_set_msg.z = 1
        self.pub_trj_set.publish(self.trj_set_msg)

    def cb_veh_sta(self, msg):
        self.veh_sta = msg

        prec_land_mode = self.veh_sta.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND 
        prec_land_mode = prec_land_mode and self.timer_main.is_canceled() and self.precision_landing

        if (prec_land_mode):
            self.timer_main = self.create_timer(int(1/self.loop_rate), 
                                                self.precision_landing_routine)
        else:
            self.timer_main.cancel()


if __name__ == '__main__':
    rclpy.init(args=None)

    vision_process = VisionProcess()

    rclcpy.spin(vision_process)

    vision_process.destroy_node()
    rclpy.shutdown()
