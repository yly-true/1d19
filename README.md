# LD19 车辆建图、Nav2 定位与 Scout Mini CAN

这是一个 ROS 2 Jazzy 工作空间，目录为：

```text
/home/yly/ld19_hand_on
```

当前工作空间包含三个功能包：

- `ldlidar_stl_ros2`：LD19 雷达驱动，发布 `/scan`。
- `scout_bringup`：统一管理建图、Nav2、CAN 测试和 RViz 的 launch 与配置。
- `scout_can`：通过 SocketCAN 收发 Scout Mini CAN 帧，并可根据 `0x221` 生成里程计。

当前使用 LD19 做二维建图和二维导航。建图使用 Cartographer，定位使用 Nav2 AMCL；`slam_toolbox` 不在当前启动流程中。

## 一、工作空间结构

```text
/home/yly/ld19_hand_on/
├── src/
│   ├── ldlidar_stl_ros2/
│   │   ├── src/ld19_scan_node.cpp
│   │   └── ldlidar_driver/
│   ├── scout_bringup/
│   │   ├── config/
│   │   │   ├── ld19_cartographer_2d.lua
│   │   │   ├── nav2_navigation_params.yaml
│   │   │   └── scout_bringup_config.yaml
│   │   ├── launch/
│   │   │   ├── ld19_mapping.launch.py
│   │   │   ├── ld19_nav2_amcl.launch.py
│   │   │   ├── scout_can_communication_test.launch.py
│   │   │   └── scout_can_odometry_rviz.launch.py
│   │   ├── maps/
│   │   ├── rviz/
│   │   └── scripts/save_ld19_map
│   └── scout_can/
│       ├── include/scout_can/scout_can_node.hpp
│       └── src/scout_can_node.cpp
├── build/
├── install/
└── README.md
```

`build/` 和 `install/` 是编译生成目录，不需要手动修改。

## 二、环境和编译

需要已经安装并可用的 ROS 2 Jazzy，以及以下 ROS 2 功能包：

```text
ldlidar_stl_ros2
cartographer_ros
mola_lidar_odometry
nav2_bringup
nav2_amcl
nav2_map_server
rviz2
tf2_ros
```

检查主要依赖：

```bash
source /opt/ros/jazzy/setup.bash

ros2 pkg prefix ldlidar_stl_ros2
ros2 pkg prefix cartographer_ros
ros2 pkg prefix mola_lidar_odometry
ros2 pkg prefix nav2_bringup
```

编译整个工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yly/ld19_hand_on
colcon build --symlink-install
source install/setup.bash
```

编译完成后，推荐重新加载 shell 配置：

```bash
source ~/.bashrc
```

当前 `~/.bashrc` 已经提供了以下快捷命令：

```text
scout_mapping
scout_save_map
scout_navigation
scout_can_send_control
scout_can_system_status
scout_can_motion_feedback
scout_can_wheel_odometry
scout_can_odometry_rviz
scout_teleop
```

## 三、LD19 串口权限

LD19 通过串口连接。当前驱动默认使用：

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

如果出现 `Permission denied`，把当前用户加入 `dialout`：

```bash
sudo usermod -aG dialout "$USER"
newgrp dialout
```

也可以注销后重新登录。检查串口：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

LD19 的默认参数在：

```text
/home/yly/ld19_hand_on/src/ldlidar_stl_ros2/src/ld19_scan_node.cpp
```

当前默认值：

```text
product_name       LDLiDAR_LD19
topic_name         scan
frame_id           base_laser
port_baudrate      230400
laser_scan_dir     Counterclockwise
angle_crop         disabled
```

当前 launch 不额外传入雷达参数，因此使用上述 C++ 默认值。修改雷达串口或其他默认参数后，需要重新编译工作空间。

## 四、启动 LD19 车辆建图

启动：

```bash
source ~/.bashrc
scout_mapping
```

等价的 ROS 2 原命令：

```bash
source /opt/ros/jazzy/setup.bash
source /home/yly/ld19_hand_on/install/setup.bash
ros2 launch scout_bringup ld19_mapping.launch.py
```

这个 launch 会启动：

1. `ldlidar_stl_ros2_node`：读取 LD19，并发布 `/scan`。
2. `static_transform_publisher`：发布 `base_link -> base_laser`，当前平移为 `z=0.18 m`。
3. `scout_can_node`：接收车辆 `0x221`，发布 `/odom` 和 `odom -> base_link`，并把 `/cmd_vel` 转成 `0x111`。
4. `cartographer_node`：使用 LD19 二维激光和车辆 `/odom` 建图。
5. `cartographer_occupancy_grid_node`：把 Cartographer 地图发布成二维栅格地图。
6. `rviz2`：显示激光、轨迹和地图。

没有遥控器时，先在一个终端使能 CAN 控制模式：

```bash
cansend can0 421#01
```

然后启动建图：

```bash
source ~/.bashrc
scout_mapping
```

再开一个 SSH 终端运行键盘控制：

```bash
source ~/.bashrc
scout_teleop --ros-args -p speed:=0.15 -p turn:=0.4
```

键盘控制中，`i` 前进，`,` 后退，`j` 左转，`l` 右转，`k` 停止，`Ctrl+C` 退出。CAN 节点超过 500 ms 没收到 `/cmd_vel` 也会自动发送零速度。

建图流程：

```text
键盘 -> /cmd_vel -> scout_can_node -> 0x111 -> Scout Mini
Scout Mini -> 0x221 -> scout_can_node -> /odom -> odom -> base_link
LD19 -> /scan -> Cartographer -> map -> odom
base_link -> base_laser
```

现在的建图配置是二维激光加车辆里程计：

```text
use_odometry = true
use_imu_data = false
use_trajectory_builder_2d = true
provide_odom_frame = false
```

建图参数文件：

```text
/home/yly/ld19_hand_on/src/scout_bringup/config/ld19_cartographer_2d.lua
```

建图 launch 的可选参数：

```bash
scout_mapping rviz:=false
scout_mapping resolution:=0.05
scout_mapping publish_period_sec:=1.0
```

## 五、保存二维地图

保持建图程序运行，另开终端：

```bash
source ~/.bashrc
scout_save_map
```

默认保存到：

```text
/home/yly/ld19_hand_on/src/scout_bringup/maps/ld19_handheld_map.yaml
/home/yly/ld19_hand_on/src/scout_bringup/maps/ld19_handheld_map.pgm
```

自定义保存目录和地图名称：

```bash
scout_save_map /home/yly/ld19_hand_on/src/scout_bringup/maps my_map
```

会生成：

```text
/home/yly/ld19_hand_on/src/scout_bringup/maps/my_map.yaml
/home/yly/ld19_hand_on/src/scout_bringup/maps/my_map.pgm
```

保存完成后，回到建图终端按 `Ctrl+C` 退出。

## 六、Nav2 + AMCL 定位

启动默认定位：

```bash
source ~/.bashrc
scout_navigation
```

默认使用配置文件中的：

```yaml
navigation:
  map: ld19_handheld_map.yaml
  odom_source: laser
```

也可以直接指定参数：

```bash
scout_navigation map:=/绝对路径/地图.yaml
scout_navigation odom_source:=laser
scout_navigation odom_source:=vehicle
```

### 激光里程计模式

```bash
scout_navigation odom_source:=laser
```

这个模式会启动 MOLA：

```text
LD19 /scan -> MOLA 激光里程计 -> odom -> base_link
地图 -> map_server -> /map
LD19 /scan -> AMCL -> map -> odom
Nav2 -> /cmd_vel
```

### 车辆里程计模式

```bash
scout_navigation odom_source:=vehicle
```

这个模式不启动 MOLA，要求车辆底盘提供：

```text
odom -> base_link
```

目前 `scout_can_node` 可以根据 CAN `0x221` 发布这个里程计。只测试车辆 CAN 里程计时，可以先执行：

```bash
source ~/.bashrc
scout_can_odometry_rviz rviz:=false
```

再在另一个终端启动：

```bash
source ~/.bashrc
scout_navigation odom_source:=vehicle
```

现在使用 `scout_navigation odom_source:=vehicle` 时，Nav2 launch 会自动启动 CAN 控制桥接，把 Nav2 的 `/cmd_vel` 转成 CAN `0x111`。

进入 Nav2 的 RViz 后：

1. 点击 `2D Pose Estimate`，设置机器人初始位姿。
2. 点击 `Nav2 Goal`，设置导航目标点。

Nav2 配置文件：

```text
/home/yly/ld19_hand_on/src/scout_bringup/config/nav2_navigation_params.yaml
```

Nav2 主要接口：

```text
/scan       sensor_msgs/msg/LaserScan
/map        nav_msgs/msg/OccupancyGrid
/odom       nav_msgs/msg/Odometry
/cmd_vel    geometry_msgs/msg/Twist
```

主要 TF：

```text
map -> odom
odom -> base_link
base_link -> base_laser
```

主要导航 action：

```text
/navigate_to_pose
/navigate_through_poses
```

## 七、配置文件

所有本项目启动文件共用：

```text
/home/yly/ld19_hand_on/src/scout_bringup/config/scout_bringup_config.yaml
```

配置内容：

```yaml
common:
  use_sim_time: false
  rviz: true

mapping:
  resolution: 0.05
  publish_period_sec: 1.0

navigation:
  map: ld19_handheld_map.yaml
  odom_source: laser

scout_can:
  interface_name: can0
  send_control: true
  use_cmd_vel: false
  cmd_vel_timeout_ms: 500
  publish_odom: false
  send_period_ms: 20
  control_vx_mm_s: 0
  control_wz_mrad_s: 0
  control_vy_mm_s: 0
  odom_frame: odom
  base_frame: base_link
```

修改 YAML 配置后，重新启动对应命令即可生效；只有修改了 `~/.bashrc` 中的快捷命令时，才需要执行：

```bash
source ~/.bashrc
```

## 八、配置和测试 CAN

Scout Mini 使用 SocketCAN 接口 `can0`，当前设置为 500 kbit/s：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

CAN 节点的可执行文件是：

```text
scout_can/scout_can_node
```

### 四个独立测试命令

四个命令分别调用同一个 launch 文件的不同 `mode`。每次测试建议只运行一个：

```bash
source ~/.bashrc
scout_can_send_control
```

只发送 `0x111` 目标速度控制帧。控制速度默认全为零。

```bash
source ~/.bashrc
scout_can_system_status
```

只接收并打印 `0x211` 系统状态帧。

```bash
source ~/.bashrc
scout_can_motion_feedback
```

只接收并打印 `0x221` 实际速度反馈帧。

```bash
source ~/.bashrc
scout_can_wheel_odometry
```

只接收并打印 `0x311` 左右轮累计里程帧。

也可以直接调用通用 launch：

```bash
ros2 launch scout_bringup scout_can_communication_test.launch.py mode:=send_control
ros2 launch scout_bringup scout_can_communication_test.launch.py mode:=system_status
ros2 launch scout_bringup scout_can_communication_test.launch.py mode:=motion_feedback
ros2 launch scout_bringup scout_can_communication_test.launch.py mode:=wheel_odometry
```

四种模式对应的 CAN 帧：

| 模式 | CAN 帧 | 方向 | 功能 |
|---|---:|---|---|
| `send_control` | `0x111` | 主机 -> 底盘 | 发送目标速度 |
| `system_status` | `0x211` | 底盘 -> 主机 | 系统状态、电池、故障 |
| `motion_feedback` | `0x221` | 底盘 -> 主机 | 实际 `vx`、`vy`、`wz` |
| `wheel_odometry` | `0x311` | 底盘 -> 主机 | 左右轮累计里程 |

## 九、CAN 里程计 RViz 测试

启动：

```bash
source ~/.bashrc
scout_can_odometry_rviz
```

这个 launch 会启动：

1. `scout_can_node`：发送 `0x111`，接收 `0x221`。
2. `rviz2`：显示网格、TF 和里程计箭头。

CAN `0x221` 的速度反馈会被积分成：

```text
/odom                 nav_msgs/msg/Odometry
odom -> base_link     TF
```

RViz 配置文件：

```text
/home/yly/ld19_hand_on/src/scout_bringup/rviz/scout_can_odometry.rviz
```

默认控制速度为零，所以车辆不会主动运动。当前 `/odom` 只使用 `0x221`，不使用 `0x311`。

检查里程计：

```bash
ros2 topic echo /odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

如果只需要 CAN 里程计、不打开 RViz：

```bash
scout_can_odometry_rviz rviz:=false
```

## 十、主要数据流

### LD19 车辆建图

```text
键盘 -> /cmd_vel -> scout_can_node -> 0x111 -> Scout Mini
Scout Mini -> 0x221 -> scout_can_node -> /odom -> odom -> base_link
LD19 -> /scan -> Cartographer -> map -> odom
base_link -> base_laser
```

### Nav2 + MOLA 激光里程计

```text
LD19 -> /scan -> MOLA -> odom -> base_link
地图 -> map_server -> /map
/scan + /map + odom -> AMCL -> map -> odom
Nav2 -> /cmd_vel
```

### Nav2 + 车辆里程计

```text
车辆底盘 -> odom -> base_link
LD19 -> /scan
地图 -> map_server -> /map
/scan + /map + odom -> AMCL -> map -> odom
Nav2 -> /cmd_vel
```

### CAN 里程计

```text
Scout Mini CAN 0x221
  -> scout_can_node
  -> /odom
  -> odom -> base_link
```

## 十一、常用检查命令

查看当前节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link base_laser
ros2 run tf2_ros tf2_echo odom base_link
```

检查 LD19 是否发布数据：

```bash
ros2 topic hz /scan
```

检查 CAN 接口：

```bash
ip link show can0
ip -details link show can0
```

如果系统安装了 `can-utils`，也可以使用：

```bash
candump can0
```

## 十二、常见问题

### 1. LD19 报 Permission denied

把用户加入 `dialout`，然后重新登录或执行：

```bash
sudo usermod -aG dialout "$USER"
newgrp dialout
```

### 2. 找不到 `/scan`

依次检查：

```bash
ls -l /dev/serial/by-id/
ros2 node list
ros2 topic list
ros2 topic hz /scan
```

确认 LD19 串口路径与 `ld19_scan_node.cpp` 中的默认值一致。

### 3. CAN 节点提示找不到 `can0`

先确认接口存在并启动：

```bash
ip link show can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 4. RViz 打开但看不到里程计

确认车辆正在返回 `0x221`：

```bash
scout_can_motion_feedback
```

然后检查：

```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

### 5. Nav2 启动后定位异常

确认以下 TF 链完整：

```text
map -> odom -> base_link -> base_laser
```

并确认：

- 地图文件存在。
- `/scan` 正常发布。
- AMCL 能收到 `/scan`。
- 选择 `odom_source:=laser` 时 MOLA 正常运行。
- 选择 `odom_source:=vehicle` 时车辆确实发布 `odom -> base_link`。

## 十三、重新编译

修改 C++、launch 或配置文件后，执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yly/ld19_hand_on
colcon build --symlink-install
source install/setup.bash
source ~/.bashrc
```
