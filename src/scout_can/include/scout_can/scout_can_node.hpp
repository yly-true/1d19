#ifndef SCOUT_CAN__SCOUT_CAN_NODE_HPP_
#define SCOUT_CAN__SCOUT_CAN_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "scout_can/planar_odometry.hpp"

namespace tf2_ros {class TransformBroadcaster;}
struct can_frame;

class ScoutCanNode final : public rclcpp::Node
{
public:
  ScoutCanNode();
  ~ScoutCanNode() override;

private:
  int openCanSocket();
  void onTimer();
  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr message);
  void sendControlFrame();
  void receiveFrames();
  void handleFrame(const struct can_frame & frame, int64_t stamp_ns);
  void handleSystemStateFrame(const struct can_frame & frame);
  void handleMotionFeedbackFrame(const struct can_frame & frame, int64_t stamp_ns);
  void handleWheelOdometerFrame(const struct can_frame & frame);
  void publishOdometry(double vx, double vy, double wz, int64_t stamp_ns);

  // CAN 接口
  int socket_fd_ = -1;
  std::string interface_name_;

  // CAN 收发开关和周期
  bool send_control_ = true;
  bool use_cmd_vel_ = false;
  bool receive_system_state_ = true;
  bool receive_motion_feedback_ = true;
  bool receive_wheel_odometry_ = true;
  bool publish_odom_ = false;
  bool log_frames_ = true;
  int send_period_ms_ = 20;
  int cmd_vel_timeout_ms_ = 500;

  // 0x111 控制帧中的目标速度
  int control_vx_mm_s_ = 0;
  int control_wz_mrad_s_ = 0;
  int control_vy_mm_s_ = 0;

  // 里程计坐标系名称
  std::string odom_frame_;
  std::string base_frame_;

  // 当前里程计状态
  PlanarOdometry odometry_;
  std::array<double, 3> odom_scale_{};
  std::array<double, 3> velocity_variance_{};
  double odom_timeout_sec_ = 0.2;

  // 键盘 /cmd_vel 的最新接收时间；超时后自动发送零速度。
  bool has_cmd_vel_ = false;
  int64_t last_cmd_vel_ns_ = 0;

  // ROS 2 通信对象
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // SCOUT_CAN__SCOUT_CAN_NODE_HPP_
