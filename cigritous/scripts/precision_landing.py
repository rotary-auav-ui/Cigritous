#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

class PrecisionLanding(Node):

  def __init__(self):
    super().__init__('precision_landing')

    self.get_logger().info('Initiating precision landing program')

    self.loop_rate = 30 # in hertz

    self.create_subscription(VehicleStatus, 'fmu/vehicle_status/out', self.cb_veh_sta, 10)

    self.pub_trj_set = self.create_publisher(TrajectorySetpoint, 'fmu/trajectory_setpoint/in')

  def main(self):
    # TODO lanjutin program precision, pake apriltags (jika niat)
    trj_set_msg = TrajectorySetpoint()
    trj_set_msg.x = 1
    self.pub_trj_set.publish(trj_set_msg)


  def cb_veh_sta(self, msg):
    self.veh_sta = msg

    if(self.veh_sta.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
      self.timer_main = self.create_timer(1/self.loop_rate, self.publish_landing)


if __name__ == '__main__':
  rclpy.init(args=None)

  precision_landing = PrecisionLanding()

  rclcpy.spin(precision_landing)

  precision_landing.destroy_node()
  rclpy.shutdown()