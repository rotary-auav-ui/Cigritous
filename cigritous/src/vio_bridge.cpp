#include <memory>
#include <chrono>
#include <functional>
#include <array>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"

#include "px4_msgs/msg/vehicle_visual_odometry.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class VioBridge : public rclcpp::Node {
  public:
    VioBridge() : Node("vio_bridge") {
			// Initiate in constructor because these data is useless after q_rot is calculated (Destroy when constructor is done)
			RCLCPP_INFO(this->get_logger(), "Initializing VIO-PX4 bridge program");

			std::string ODOM_SUB_TOPIC = this->declare_parameter<std::string>("topic.sub_odom", "odometry_sub");
			std::string VISION_PUB_TOPIC = this->declare_parameter<std::string>("topic.pub_vision", "vision_pub");

			auto body_tf = this->declare_parameters<float>(
				"frame_rotation",
				{
					{"roll", 0},
					{"pitch", 0},
					{"yaw", 0}
				});

			auto camera_tf = this->declare_parameters<float>(
				"camera2body_rotation",
				{
					{"roll", 0},
					{"pitch", 0},
					{"yaw", 0}
				});

			int rate = this->declare_parameter<int>("publish_rate", 30);

			WITH_COV = this->declare_parameter<bool>("use_covariance", false);
			WITH_VEL = this->declare_parameter<bool>("use_velocity", false);

			// Initiate in constructor because these data is useless after q_rot is calculated (Destroy when constructor is done)
			tf2::Quaternion q_cam_enu, q_body_enu;
			q_cam_enu.setRPY(radians(camera_tf[0]), radians(camera_tf[1]), radians(camera_tf[2]));
			q_body_enu.setRPY(radians(body_tf[0]), radians(body_tf[1]), radians(body_tf[2]));
			q_rot = q_cam_enu * q_body_enu;
			m_rot = tf2::Matrix3x3(q_rot);
			RCLCPP_INFO(this->get_logger(), "Transform cam; Roll: %f | Pitch: %f | Yaw: %f", camera_tf[0], camera_tf[1], camera_tf[2]);
			RCLCPP_INFO(this->get_logger(), "Transform body; Roll: %f | Pitch: %f | Yaw: %f", body_tf[0], body_tf[1], body_tf[2]);

			odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(ODOM_SUB_TOPIC, 10, std::bind(&VioBridge::odometry_sub_cb, this, _1));
			RCLCPP_INFO(this->get_logger(), "Odometry subscriber topic: %s", ODOM_SUB_TOPIC.c_str());

			odometry_pub = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>(VISION_PUB_TOPIC, 10);
			RCLCPP_INFO(this->get_logger(), "Odometry publisher topic: %s", VISION_PUB_TOPIC.c_str());
			RCLCPP_INFO(this->get_logger(), "Covariance Data: %s", WITH_COV ? "Used" : "Not Used");
			RCLCPP_INFO(this->get_logger(), "Velocity Data: %s", WITH_VEL ? "Used" : "Not Used");

			timer = this->create_wall_timer(std::chrono::milliseconds(int(1e3/rate)), std::bind(&VioBridge::publish, this));

			last_odom_time = this->now();
		}

    void publish() {

			if (odometry_sub->get_publisher_count() == 0) {
		    RCLCPP_WARN(this->get_logger(), "No VIO data. Waiting");
        rclcpp::sleep_for(1s);
				return;
			}

			odom_proc.timestamp = int(this->now().seconds() / 1e3); // in microseconds
			odom_proc.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
			
			// CALCULATE ORIENTATION
			static tf2::Quaternion q_cam;

			q_cam = q_rot * q_orig;
			q_cam.normalize();
			odom_proc.q[0] = q_cam.w();
			odom_proc.q[1] = q_cam.x();
			odom_proc.q[2] = q_cam.y();
			odom_proc.q[3] = q_cam.z();

			// CALCULATE POSITION
			static tf2::Vector3 pose_cam;
			
			pose_cam = m_rot * pose_orig;
			odom_proc.x = pose_cam[0];
			odom_proc.y = pose_cam[1];
			odom_proc.z = pose_cam[2];

			if (WITH_COV) {
				// CALCULATE COVARIANCE POSITION
				static tf2::Matrix3x3 pose_cov_proc, rot_pose_cov_proc;

				pose_cov_proc = m_rot*pose_cov_orig*m_rot.transpose();
				rot_pose_cov_proc = m_rot*rot_pose_cov_orig*m_rot.transpose();

				odom_proc.pose_covariance = toMsgPX4(pose_cov_proc, rot_pose_cov_proc);
			}

			if (WITH_VEL) {
				odom_proc.velocity_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;

				// CALCULATE LINEAR VELOCITY
				static tf2::Vector3 spd_cam;

				spd_cam = m_rot*spd_orig;
				odom_proc.vx = spd_cam[0];
				odom_proc.vy = spd_cam[1];
				odom_proc.vz = spd_cam[2];

				// CALCULATE ANGULAR VELOCITY
				static tf2::Vector3 rotspd_cam;
				
				rotspd_cam = m_rot*rotspd_orig;

				odom_proc.rollspeed = rotspd_cam[0];
				odom_proc.pitchspeed = rotspd_cam[1];
				odom_proc.yawspeed = rotspd_cam[2];

				if (WITH_COV) {
					// CALCULATE COVARIANCE VELOCITY
					static tf2::Matrix3x3 vel_cov_proc, rot_vel_cov_proc;

					vel_cov_proc = m_rot*vel_cov_orig*m_rot.transpose();
					rot_vel_cov_proc = m_rot*rot_vel_cov_orig*m_rot.transpose();

					odom_proc.velocity_covariance = toMsgPX4(vel_cov_proc, rot_vel_cov_proc);
				}
			}

			odometry_pub->publish(odom_proc);
		}

    rclcpp::TimerBase::SharedPtr timer;
  
  private:
    rclcpp::Node::SharedPtr node;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odometry_pub;

    rclcpp::Time last_odom_time;

    // visual odometry send data
    tf2::Vector3 pose_orig;
    tf2::Quaternion q_orig;
    tf2::Vector3 rotspd_orig;
    tf2::Vector3 spd_orig;
    tf2::Matrix3x3 vel_cov_orig, rot_vel_cov_orig;
    tf2::Matrix3x3 pose_cov_orig, rot_pose_cov_orig;

    px4_msgs::msg::VehicleVisualOdometry odom_proc;

    tf2::Quaternion q_rot;
    tf2::Matrix3x3 m_rot;

    bool WITH_COV;
    bool WITH_VEL;

		void fromMsgLT(const std::array<double, 36>& data, tf2::Matrix3x3& mat3x3) {
			mat3x3.setValue(data[0], 0, 0,
											0, data[7], 0,
											0, 0, data[14]);
		}

    void fromMsgRB(const std::array<double, 36>& data, tf2::Matrix3x3& mat3x3) {
			mat3x3.setValue(data[21], 0, 0,
											0, data[28], 0,
											0, 0, data[35]);
		}

		float radians(const float& deg) {
			return deg*M_PI/180;
		}

		float degrees(const float& rad) {
			return rad*180/M_PI;
		}

    std::array<float, 21> toMsgPX4(const tf2::Matrix3x3& matLT, const tf2::Matrix3x3& matRB) {
			return {float(matLT[0][0]), 0, 0, 0, 0, 0, float(matLT[1][1]), 
				0, 0, 0, 0, float(matLT[2][2]), 0, 0, 
				0, float(matRB[0][0]), 0, 0, float(matRB[1][1]), 0, float(matRB[2][2])};
		}

    // slow subscribing speed to match publishing data. EXPERIMENTAL!
    void odometry_sub_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
			pose_orig[0] = msg->pose.pose.position.x;
			pose_orig[1] = msg->pose.pose.position.y;
			pose_orig[2] = msg->pose.pose.position.z;
			
			q_orig[0] = msg->pose.pose.orientation.x;
			q_orig[1] = msg->pose.pose.orientation.y;
			q_orig[2] = msg->pose.pose.orientation.z;
			q_orig[3] = msg->pose.pose.orientation.w;

			spd_orig[0] = msg->twist.twist.linear.x;
			spd_orig[1] = msg->twist.twist.linear.y;
			spd_orig[2] = msg->twist.twist.linear.z;

			rotspd_orig[0] = msg->twist.twist.angular.x;
			rotspd_orig[1] = msg->twist.twist.angular.y;
			rotspd_orig[2] = msg->twist.twist.angular.z;
			
      if(WITH_COV) {		
				fromMsgLT(msg->pose.covariance, pose_cov_orig);
				fromMsgRB(msg->pose.covariance, rot_pose_cov_orig);

				fromMsgLT(msg->twist.covariance, vel_cov_orig);
				fromMsgRB(msg->twist.covariance, rot_vel_cov_orig);
			}

			if(last_odom_time >= msg->header.stamp) {
				RCLCPP_ERROR(this->get_logger(), "Odometry data lagging");
				RCLCPP_ERROR(this->get_logger(), "Shutting odometry node");
				rclcpp::shutdown();
			}

			last_odom_time = msg->header.stamp;
		}
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

	auto node = std::make_shared<VioBridge>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}