#ifndef SCOUT_CAN__SCOUT_CAN_NODE_HPP_
#define SCOUT_CAN__SCOUT_CAN_NODE_HPP_  //是头文件保护，防止同一个头文件被重复包含

#include <cstdint>
#include <memory>
#include <string>
// 分别提供：

//   - int64_t 等固定宽度整数类型
//   - std::unique_ptr、智能指针
//   - std::string


#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// 入本类需要使用的 ROS 2 类型：

//   - nav_msgs::msg::Odometry：发布 /odom
//   - rclcpp::Node、定时器、Publisher 等 ROS 2 功能

namespace tf2_ros
{
class TransformBroadcaster;
}
// 这里只是提前告诉编译器：

//   > tf2_ros 命名空间里有一个叫 TransformBroadcaster 的类。   命名空间就是给类、函数和变量加一个“前缀”，防止重名

//   具体实现放到 .cpp 中再 include，这样头文件更简洁。

struct can_frame;

//  同样是提前声明 Linux CAN 的数据帧结构体。真正的定义在 .cpp 中通过：

//   #include <linux/can.h>

//   引入。

class ScoutCanNode final : public rclcpp::Node   // - ScoutCanNode：类名。  - : public rclcpp::Node：继承 ROS 2 的节点类，所以它本身就是一个 ROS 2 节点。   - final：禁止其他类继续继承它。
{
public:
  // 构造函数：创建 ScoutCanNode 对象时自动调用。
  // 具体实现位于 src/scout_can_node.cpp。
  ScoutCanNode();

  // 析构函数：ScoutCanNode 对象销毁时自动调用。
  // 具体实现中会关闭已经打开的 CAN socket，释放相关资源。
  // override 表示它重写了父类 rclcpp::Node 的虚析构函数。
  ~ScoutCanNode() override;

//  - ScoutCanNode()：构造函数。创建节点对象时自动执行。
//   - ~ScoutCanNode()：析构函数。节点销毁时自动执行，用来关闭 CAN 文件描述符等资
//     源。

//   - override：说明它重写了父类的虚函数，编译器会帮我们检查



private:
  int openCanSocket();              //打开并绑定 can0，返回 Linux CAN socket 的文件描述符
  void onTimer();                             //定时器回调函数，周期执行。
  void sendControlFrame();                       //发送 0x111 控制帧。
  void receiveFrames();                             //从 CAN socket 中读取接收到的数据
  void handleFrame(const struct can_frame & frame);                       //根据 CAN ID 判断这是什么帧，然后分发给不同处理函数
  void handleSystemStateFrame(const struct can_frame & frame);
  void handleMotionFeedbackFrame(const struct can_frame & frame);
  void handleWheelOdometerFrame(const struct can_frame & frame);

// 这里的：

//   const struct can_frame & frame

//   表示通过引用传入 CAN 帧，避免复制；const 表示函数不会修改这帧数据。


  // 分别处理：

  // - 0x211 系统状态
  // - 0x221 实际速度
  // - 0x311 左右轮累计里程


  void publishOdometry(double vx, double vy, double wz);

  //把 0x221 中解析出来的速度积分成 /odom，并发布 odom -> base_link TF。

  // CAN 接口
  int socket_fd_ = -1;
  std::string interface_name_;

  // CAN 收发开关和周期
  bool send_control_ = true;
  bool receive_system_state_ = true;
  bool receive_motion_feedback_ = true;
  bool receive_wheel_odometry_ = true;
  bool publish_odom_ = false;
  bool log_frames_ = true;
  int send_period_ms_ = 20;

  // 0x111 控制帧中的目标速度
  int control_vx_mm_s_ = 0;
  int control_wz_mrad_s_ = 0;
  int control_vy_mm_s_ = 0;

  // 里程计坐标系名称
  std::string odom_frame_;
  std::string base_frame_;

  // 当前里程计状态
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  bool has_last_stamp_ = false;
  int64_t last_stamp_ns_ = 0;

  // ROS 2 通信对象
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // SCOUT_CAN__SCOUT_CAN_NODE_HPP_
