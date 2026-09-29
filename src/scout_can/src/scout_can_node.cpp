#include "scout_can/scout_can_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
constexpr canid_t kControlId = 0x111;
constexpr canid_t kSystemStateId = 0x211;
constexpr canid_t kMotionFeedbackId = 0x221;
constexpr canid_t kWheelOdometerId = 0x311;

int16_t decodeSigned16(const __u8 * data)
{
  const uint16_t value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return static_cast<int16_t>(value);
}

uint16_t decodeUnsigned16(const __u8 * data)
{
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

int32_t decodeSigned32(const __u8 * data)
{
  const uint32_t value =
    (static_cast<uint32_t>(data[0]) << 24) |
    (static_cast<uint32_t>(data[1]) << 16) |
    (static_cast<uint32_t>(data[2]) << 8) |
    data[3];
  return static_cast<int32_t>(value);
}

void encodeSigned16(__u8 * data, int value)
{
  value = std::max(-32768, std::min(32767, value));
  const auto encoded = static_cast<uint16_t>(static_cast<int16_t>(value));
  data[0] = static_cast<__u8>((encoded >> 8) & 0xff);
  data[1] = static_cast<__u8>(encoded & 0xff);
}
}  // namespace

ScoutCanNode::ScoutCanNode()
: rclcpp::Node("scout_can")
{
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// 普通类直接创建指针：

//   class A
//   {
//   };

//   A* p = new A();

//   或者使用智能指针：

//   std::shared_ptr<A> p =
//     std::make_shared<A>();





  interface_name_ = declare_parameter<std::string>("interface_name", "can0");
  send_control_ = declare_parameter<bool>("send_control", true);
  use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
  receive_system_state_ = declare_parameter<bool>("receive_system_state", true);
  receive_motion_feedback_ = declare_parameter<bool>("receive_motion_feedback", true);
  receive_wheel_odometry_ = declare_parameter<bool>("receive_wheel_odometry", true);
  publish_odom_ = declare_parameter<bool>("publish_odom", false);
  log_frames_ = declare_parameter<bool>("log_frames", true);
  send_period_ms_ = declare_parameter<int>("send_period_ms", 20);
  cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 500);
  control_vx_mm_s_ = declare_parameter<int>("control_vx_mm_s", 0);
  control_wz_mrad_s_ = declare_parameter<int>("control_wz_mrad_s", 0);
  control_vy_mm_s_ = declare_parameter<int>("control_vy_mm_s", 0);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  socket_fd_ = openCanSocket();

  if (publish_odom_) {
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  if (use_cmd_vel_) {
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ScoutCanNode::handleCmdVel, this, std::placeholders::_1));
  }

  const auto period = std::chrono::milliseconds(std::max(1, send_period_ms_));
  timer_ = create_wall_timer(period, std::bind(&ScoutCanNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "SocketCAN %s ready: send_control=%s, publish_odom=%s, odom_source=%s",
    interface_name_.c_str(),
    send_control_ ? "true" : "false",
    publish_odom_ ? "true" : "false",
    publish_odom_ ? "CAN 0x221" : "disabled");
}

ScoutCanNode::~ScoutCanNode()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

int ScoutCanNode::openCanSocket()
{
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw std::runtime_error(
      "cannot create CAN socket: " + std::string(std::strerror(errno)));
  }

  struct ifreq interface_request {};
  std::strncpy(
    interface_request.ifr_name,
    interface_name_.c_str(),
    IFNAMSIZ - 1);

  if (ioctl(fd, SIOCGIFINDEX, &interface_request) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot find CAN interface " + interface_name_ + ": " + error);
  }

  struct sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot bind CAN interface " + interface_name_ + ": " + error);
  }

  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot set CAN socket non-blocking: " + error);
  }

  return fd;
}

void ScoutCanNode::onTimer()
{
  if (send_control_) {
    if (use_cmd_vel_) {
      const int64_t now_ns = now().nanoseconds();
      const int64_t timeout_ns =
        static_cast<int64_t>(std::max(1, cmd_vel_timeout_ms_)) * 1000000;
      if (!has_cmd_vel_ || now_ns - last_cmd_vel_ns_ > timeout_ns) {
        control_vx_mm_s_ = 0;
        control_wz_mrad_s_ = 0;
        control_vy_mm_s_ = 0;
      }
    }
    sendControlFrame();
  }
  receiveFrames();
}

void ScoutCanNode::handleCmdVel(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  control_vx_mm_s_ = static_cast<int>(std::lround(message->linear.x * 1000.0));
  control_wz_mrad_s_ = static_cast<int>(std::lround(message->angular.z * 1000.0));
  control_vy_mm_s_ = static_cast<int>(std::lround(message->linear.y * 1000.0));
  last_cmd_vel_ns_ = now().nanoseconds();
  has_cmd_vel_ = true;
}

// 功能一：发送 0x111 目标速度控制帧。
void ScoutCanNode::sendControlFrame()
{
  struct can_frame frame{};
  frame.can_id = kControlId;
  frame.can_dlc = 8;
  encodeSigned16(frame.data, control_vx_mm_s_);
  encodeSigned16(frame.data + 2, control_wz_mrad_s_);
  encodeSigned16(frame.data + 4, control_vy_mm_s_);

  if (::write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "failed to send CAN 0x111: %s", std::strerror(errno));
  }
}

void ScoutCanNode::receiveFrames()
{
  while (true) {
    struct can_frame frame{};
    const ssize_t bytes = ::read(socket_fd_, &frame, sizeof(frame));
    if (bytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to receive CAN frame: %s", std::strerror(errno));
      return;
    }
    if (bytes == static_cast<ssize_t>(sizeof(frame))) {
      handleFrame(frame);
    }
  }
}

void ScoutCanNode::handleFrame(const struct can_frame & frame)
{
  const canid_t id = frame.can_id & CAN_SFF_MASK;
  if (receive_system_state_ && id == kSystemStateId && frame.can_dlc >= 8) {
    handleSystemStateFrame(frame);
    return;
  }

  // 0x221 reports the chassis's actual velocity, which is integrated into odometry.
  if (receive_motion_feedback_ && id == kMotionFeedbackId && frame.can_dlc >= 6) {
    handleMotionFeedbackFrame(frame);
    return;
  }

  if (receive_wheel_odometry_ && id == kWheelOdometerId && frame.can_dlc >= 8) {
    handleWheelOdometerFrame(frame);
    return;
  }

  if (log_frames_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RX CAN frame: id=0x%03X dlc=%u", id, frame.can_dlc);
  }
}

// 功能二：解析 0x211 系统状态帧。
void ScoutCanNode::handleSystemStateFrame(const struct can_frame & frame)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "RX 0x211: status=%u mode=%u battery_raw=%u fault=%u counter=%u",
    frame.data[0], frame.data[1], decodeUnsigned16(frame.data + 2),
    frame.data[5], frame.data[7]);
}

// 功能三：解析 0x221 实际速度帧，并按需生成 /odom。
void ScoutCanNode::handleMotionFeedbackFrame(const struct can_frame & frame)
{
  const double vx = static_cast<double>(decodeSigned16(frame.data)) / 1000.0;
  const double wz = static_cast<double>(decodeSigned16(frame.data + 2)) * 0.001;
  const double vy = static_cast<double>(decodeSigned16(frame.data + 4)) / 1000.0;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "RX 0x221: vx=%.3f m/s vy=%.3f m/s wz=%.3f rad/s",
    vx, vy, wz);

  if (publish_odom_) {
    publishOdometry(vx, vy, wz);
  }
}

// 功能四：解析 0x311 左右轮累计里程帧。
void ScoutCanNode::handleWheelOdometerFrame(const struct can_frame & frame)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "RX 0x311: left_wheel=%d mm right_wheel=%d mm (not used for odometry)",
    decodeSigned32(frame.data), decodeSigned32(frame.data + 4));
}

void ScoutCanNode::publishOdometry(double vx, double vy, double wz)
{
  const auto stamp = now();
  const int64_t stamp_ns = stamp.nanoseconds();
  if (has_last_stamp_) {
    const double dt = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
    if (dt > 0.0 && dt < 1.0) {
      x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
      y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
      yaw_ += wz * dt;
    }
  }
  last_stamp_ns_ = stamp_ns;
  has_last_stamp_ = true;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw_);

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose.position.x = x_;
  odometry.pose.pose.position.y = y_;
  odometry.pose.pose.orientation = tf2::toMsg(quaternion);
  odometry.twist.twist.linear.x = vx;
  odometry.twist.twist.linear.y = vy;
  odometry.twist.twist.angular.z = wz;
  odom_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ScoutCanNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "scout_can_node: %s\n", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
