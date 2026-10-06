/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD19 products
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */







//  ld19_scan_node.cpp 启动 ROS 2 并创建 ldlidar_published 节点，读取并准备 LD19 的参数，然
//   后通过裸指针创建 LDLidarDriver 驱动对象，打开串口并确认通信正常；接着创建发布
//   sensor_msgs::msg::LaserScan 消息的 Publisher，在循环中读取雷达原始点和扫描频
//   率，调用 ToLaserscanMessagePublish() 打包成 LaserScan 消息并发布到 /scan，最后
//   在退出时停止雷达、释放驱动对象并关闭 ROS 2

#include "ldlidar_stl_ros2/laser_scan_settings.hpp"
#include "ldlidar_driver.h"

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);                         //初始化rclcpp
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // 创建一个 ROS2 Node 节点名字叫做ldlidar_published
  //node是一个智能指针

  std::string product_name = "LDLiDAR_LD19";
	std::string topic_name = "scan";
	std::string port_name = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
  int serial_port_baudrate = 230400;
  // 定义 LD19 的普通 C++ 配置变量：

  // product_name       雷达型号
  // topic_name         topic 名字
  // port_name          串口路径
  // serial_port_baudrate 波特率

  ldlidar::LDType type_name;    //定义一个表示雷达型号的变量。现在还没有赋值，后面第 84 行才设置为： ldlidar::LDType::LD_19

  LaserScanSetting setting;     //一个结构体，LaserScanSetting在laser_scan_settings.hpp中定义


  setting.frame_id = "base_laser";    //LaserScan 的坐标系是 base_laser
  setting.laser_scan_dir = true;      //使用逆时针方向
  setting.enable_angle_crop_func = false;//关闭角度裁剪
  setting.angle_crop_min = 135.0;        //范围仍然先被设置为 135～225 度
  setting.angle_crop_max = 225.0;     //给setting这个结构体的成员赋值
  

  //到这里为止，程序还没有打开串口，也没有创建 /scan 发布器，只是在准备配置。





  //node在上面，是节点ldlidar_published，数据类型是一个智能指针
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  //这些代码把 C++ 变量注册成 ROS 2 参数

  // 例如：

  // node->declare_parameter<std::string>(
  //     "port_name", port_name);

  // 意思是：

  // 声明一个名字叫 port_name 的 ROS 参数，
  // 类型是 string，
  // 默认值使用当前 port_name 变量的值。

  // 也就是：

  // 参数名：port_name
  // 默认值：/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0








  // ros2参数传回C++
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);

  //作用是把 ROS 参数的值读回 C++ 变量
  // 流程就是：

  // 先准备默认值
  //   -> declare_parameter() 注册参数
  //   -> get_parameter() 读取最终参数

  // 当前 Launch 没有传参数，所以最终仍然使用 ld19_scan_node.cpp 里的默认值。

  // 如果 Launch 传了：

  // parameters=[{
  //     'port_name': '/dev/ttyUSB0'
  // }]

  // 那么 get_parameter() 后，C++ 变量 port_name 就会变成：

  // /dev/ttyUSB0





  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  // ldlidar::LDLidarDriver 仍然只是一个类类型，ldlidar:: 是它所属的命名空间。
  // `*` 表示 ldlidarnode 是一个指针变量；指针本身只保存地址，不是驱动对象。
  // 右侧的 `new ldlidar::LDLidarDriver()` 才会在堆内存中创建一个实例，
  // 调用它的构造函数，并返回这个实例的地址。因此 `new T()` 的类型是 `T*`。
  // 左侧的 `ldlidar::LDLidarDriver*` 表示 ldlidarnode 可以保存这个类对象的地址。
  // ldlidarnode 保存这个地址后，可以通过 `->` 访问对象的公开成员和成员函数。
  // 例如：ldlidarnode->Start(...) 等价于 (*ldlidarnode).Start(...)。
  //
  // 这个堆对象最后必须通过 delete 释放：
  // delete ldlidarnode;
  // 释放后再把指针设为 nullptr，可以避免继续使用已经失效的地址。
  //
  // 对比：
  // LDLidarDriver object;  // 直接创建对象，访问用 object.xxx，离开作用域后自动销毁
  // LDLidarDriver* ptr;     // 只声明指针，没有创建 LDLidarDriver 对象

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<topic_name>: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);

  //打印信息






  if (product_name != "LDLiDAR_LD19") {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }//检查，改成至支持ld19了



  // type_name = ldlidar::LDType::LD_19; //把字符串改成底层驱动使用的枚举了

  type_name = ldlidar::LDType::LD_19;

  // ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));

  // ldlidarnode->EnableFilterAlgorithnmProcess(true);

  // 接着注册时间回调：

  // ldlidarnode->RegisterGetTimestampFunctional(
  //     std::bind(&GetSystemTimeStamp));

  // 意思是把 GetSystemTimeStamp 函数交给驱动保存。

  // 之后驱动需要获取当前时间时，就可以调用这个函数。它相当于：

  // 驱动对象 -> 保存时间函数
  // 驱动内部 -> 需要时调用时间函数

  // 启用滤波：

  // ldlidarnode->EnableFilterAlgorithnmProcess(true);

  // 把底层驱动的滤波功能打开。








  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

//  最后真正启动雷达：

//   if (ldlidarnode->Start(
//           type_name,
//           port_name,
//           serial_port_baudrate,
//           ldlidar::COMM_SERIAL_MODE)) {

//   传入：

//   LD19 型号
//   串口路径
//   波特率
//   串口通信模式





  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // 调用 ldlidarnode 对象的 WaitLidarCommConnect()
  // 最多等待 3000 毫秒
  // 返回 true 就进入成功分支
  // 返回 false 就进入失败分支

  // Start() 成功，只说明驱动已经开始启动；这里进一步确认雷达是否真的有通信。






  // 创建雷达话题和发布器
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  // rclcpp::Publisher<sensor_msgs::msg::LaserScan>
  // 是发布器对象的类型，表示这个发布器发布 LaserScan 消息。
  // ::SharedPtr publisher 表示 publisher 是指向该发布器对象的智能指针。

  // node->create_publisher<sensor_msgs::msg::LaserScan>(
  //     topic_name, 10);
  // 1. 通过 node 指针调用 create_publisher() 方法。
  // 2. 创建一个发布 LaserScan 消息的发布器对象。
  // 3. 返回该发布器对象的 SharedPtr，并赋值给 publisher。



  rclcpp::WallRate r(10); //10hz

  ldlidar::Points2D laser_scan_points;  //保存一圈雷达扫描点



  double lidar_scan_freq;               //保存雷达当前扫描频率


  RCLCPP_INFO(node->get_logger(), "Publish topic message:ldlidar scan data."); //打印
  
  
  while (rclcpp::ok()) {    //ROS 2是否正常在工作
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)){   //读一次雷达的数据
      case ldlidar::LidarStatus::NORMAL:                                 //如果工作正常
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);

        // bool GetLidarScanFreq(double& spin_hz) 引用，也就是可以直接修改外面的变量，也就是直接给 double lidar_scan_freq赋值

        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, node, publisher); 

        //调用自定义函数，把原始雷达数据转换成 ROS 2 的 LaserScan 消息
            // ToLaserscanMessagePublish()
            // -> 创建 LaserScan 消息
            // -> 填充角度、距离、时间戳等数据
            // -> publisher->publish(output)
            // -> 发布 /scan


        // 这里的参数分别是：
        // laser_scan_points：GetLaserScanData() 读取到的原始扫描点。
        // lidar_scan_freq：雷达当前的扫描频率。
        // setting：LaserScanSetting 配置结构体，不是要发布的消息类型。
        // node：前面创建的 ROS 2 节点，用来获取时间等信息。
        // publisher：前面通过 node->create_publisher() 创建的发布器。
        // 函数内部会创建 sensor_msgs::msg::LaserScan output，
        // 然后调用 publisher->publish(output)，最终发布到 /scan。
        // /scan 的消息类型是 sensor_msgs::msg::LaserScan，
        // 这个消息类型来自 ROS 2 的 sensor_msgs 功能包。










        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:         //不正常，滚吧
        RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    r.sleep();
  }
  //关掉，关掉
  ldlidarnode->Stop();

  delete ldlidarnode;
  ldlidarnode = nullptr;

  RCLCPP_INFO(node->get_logger(), "ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = node->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }
  // Adjust the parameters according to the demand
  angle_min = 0;
  range_min = 0.02;
  range_max = 12.0f; // LD19 specified maximum range at 70% reflectivity
  // Keep LaserScan geometry identical across revolutions. The number of raw
  // LD19 points varies slightly, and SLAM Toolbox drops scans whose size changes.
  constexpr int beam_size = 720;
  angle_increment = static_cast<float>(2.0 * M_PI / beam_size);
  angle_max = angle_min + (beam_size - 1) * angle_increment;
  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::msg::LaserScan output;
    // GetLaserScanData() returns a completed revolution.  LaserScan's stamp
    // represents the first ray, while time_increment advances through the
    // revolution, so move the stamp back by one scan period.
    output.header.stamp = start_scan_time - rclcpp::Duration::from_seconds(scan_time);
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    output.time_increment = static_cast<float>(scan_time / (beam_size - 1));
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(std::floor((angle - angle_min) / angle_increment));
      if (index >= 0 && index < beam_size) {
        const int scan_index = setting.laser_scan_dir ? (beam_size - index) % beam_size : index;
        // Multiple raw points may land in one beam; keep the nearest range.
        if (std::isnan(output.ranges[scan_index]) || range < output.ranges[scan_index]) {
          output.ranges[scan_index] = range;
        }
        output.intensities[scan_index] = intensity;
      }
    }
    lidarpub->publish(output);
    end_scan_time = start_scan_time;
  } 
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
