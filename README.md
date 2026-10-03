# LD19 + Scout Mini 二维建图与导航

ROS 2 Jazzy 工作空间。建图使用 SLAM Toolbox，Nav2 使用 AMCL 定位；里程计统一来自 Scout Mini CAN `0x221`，由 `scout_can_node` 发布 `/odom` 和 `odom -> base_link`。不启动独立激光里程计节点。

## 使用

安装需要的 ROS 包（已有则跳过）：

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server
```

编译：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yly/1d19
colcon build --symlink-install
source install/setup.bash
```

CAN 接口需先配置为 500 kbit/s；LD19 串口用户需有 `dialout` 权限：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

启动 SLAM Toolbox 建图：

```bash
ros2 launch scout_slam_toolbox ld19_slam_toolbox_mapping.launch.py
```

实车默认 `use_sim_time:=false`。该 launch 启动 LD19、雷达静态 TF、CAN 里程计和 SLAM Toolbox。需要键盘控制时另开终端运行 `scout_teleop`；若底盘尚未使能，可发送 `cansend can0 421#01`。

建图时保存地图，文件名不带扩展名：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/yly/1d19/src/scout_bringup/maps/my_map
```

启动 Nav2 定位与导航：

```bash
ros2 launch scout_bringup ld19_nav2_amcl.launch.py map:=/home/yly/1d19/src/scout_bringup/maps/my_map.yaml
```

在 RViz 点击 `2D Pose Estimate` 设置初始位姿，再用 `Nav2 Goal` 设置目标。Nav2 默认地图在 `src/scout_bringup/maps/ld19_handheld_map.yaml`。

## 算法与数据流

```text
LD19 -> /scan ------------------------------┐
                                            v
CAN 0x221 -> scout_can_node -> /odom -> odom -> base_link
                                            |
base_link -> base_laser --------------------┘
                  SLAM Toolbox -> /map + map -> odom
```

`scout_can_node` 将 CAN `0x221` 的实际速度 `vx/vy/wz` 按时间积分，发布车辆里程计。静态 TF 描述雷达安装位姿。SLAM Toolbox 以车辆里程计预测扫描位姿，再用激光扫描匹配建立位姿图；车辆回到已走过的区域时，回环检测添加约束并优化位姿图。它发布 `/map` 和 `map -> odom`，车辆 CAN 仍是 `odom -> base_link` 的唯一来源。激光扫描匹配是 SLAM 建图的一部分，不是独立激光里程计。

## 参数

### 当前建图参数

修改 `src/scout_slam_toolbox/config/mapper_params_online_async.yaml`：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `odom_frame` | `odom` | 车辆里程计 TF 名，须与 CAN 节点一致 |
| `map_frame` | `map` | 地图 TF 名 |
| `base_frame` | `base_link` | 车体 TF 名，须与 CAN 节点一致 |
| `scan_topic` | `/scan` | 雷达扫描话题 |
| `mode` | `mapping` | 在线建图模式 |
| `resolution` | `0.05` m | 栅格大小；越小地图越细、占用越多 |
| `use_scan_matching` | `true` | SLAM 内用扫描匹配优化位姿 |
| `do_loop_closing` | `true` | 启用回环检测和位姿图修正 |

### SLAM Toolbox 可调参数全集

以下为 SLAM Toolbox 上游异步建图示例配置及支持的附加选项；本项目当前 YAML 只显式覆盖上表八项。需固定其他参数时，将其放入同一 YAML 的 `slam_toolbox: ros__parameters:`。上游说明：[参数文档](https://github.com/SteveMacenski/slam_toolbox#configuration)、[异步配置示例](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)。

| 类别 | 参数（上游示例值） |
|---|---|
| 图优化器 | `solver_plugin=solver_plugins::CeresSolver`；`ceres_linear_solver=SPARSE_NORMAL_CHOLESKY`；`ceres_preconditioner=SCHUR_JACOBI`；`ceres_trust_strategy=LEVENBERG_MARQUARDT`；`ceres_dogleg_type=TRADITIONAL_DOGLEG`；`ceres_loss_function=None` |
| 地图与 TF | `use_map_saver=true`；`debug_logging=false`；`throttle_scans=1`；`transform_publish_period=0.02` 秒；`map_update_interval=5.0` 秒；`restamp_tf=false`；`min_laser_range=0.0` m；`max_laser_range=20.0` m；`minimum_time_interval=0.5` 秒；`transform_timeout=0.2` 秒；`tf_buffer_duration=30.0` 秒；`stack_size_to_use=40000000` 字节；`enable_interactive_mode=true` |
| 扫描筛选与匹配 | `minimum_travel_distance=0.5` m；`minimum_travel_heading=0.5` rad；`check_min_dist_and_heading_precisely=false`；`use_scan_barycenter=true`；`scan_buffer_size=10`；`scan_buffer_maximum_scan_distance=10.0` m；`link_match_minimum_response_fine=0.1`；`link_scan_maximum_distance=1.5` m |
| 回环检测 | `loop_search_maximum_distance=3.0` m；`loop_match_minimum_chain_size=10`；`loop_match_maximum_variance_coarse=3.0`；`loop_match_minimum_response_coarse=0.35`；`loop_match_minimum_response_fine=0.45` |
| 搜索范围 | `correlation_search_space_dimension=0.5` m；`correlation_search_space_resolution=0.01` m；`correlation_search_space_smear_deviation=0.1`；`loop_search_space_dimension=8.0` m；`loop_search_space_resolution=0.05` m；`loop_search_space_smear_deviation=0.03` |
| 匹配惩罚与栅格 | `distance_variance_penalty=0.5`；`angle_variance_penalty=1.0`；`fine_search_angle_offset=0.00349` rad；`coarse_search_angle_offset=0.349` rad；`coarse_angle_resolution=0.0349` rad；`minimum_angle_penalty=0.9`；`minimum_distance_penalty=0.5`；`use_response_expansion=true`；`min_pass_through=2`；`occupancy_threshold=0.1` |
| 其他支持选项 | `scan_queue_size`（异步模式建议 `1`）；`map_file_name`、`map_start_pose`、`map_start_at_dock`（接续已有位姿图时用）；`position_covariance_scale=1.0`、`yaw_covariance_scale=1.0`（输出协方差缩放）；`localization_on_configure`（仅组合建图/定位节点适用） |

常见调参先后：先确认 TF、`scan_topic`、雷达距离范围；再按地图细节调整 `resolution`；扫描匹配不稳时再调整移动阈值、匹配搜索范围和回环阈值。坐标系名必须与 TF 树一致。

### Launch、CAN、LD19 与 TF 参数

新建图 launch 在 `src/scout_slam_toolbox/launch/ld19_slam_toolbox_mapping.launch.py`，目前命令行 launch 参数只有 `use_sim_time`（默认 `false`）。同一文件还设置 CAN 和静态 TF：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| CAN `interface_name` | `can0` | SocketCAN 接口 |
| `send_control` / `use_cmd_vel` | `true` / `true` | 发送控制帧并订阅 `/cmd_vel` |
| `receive_motion_feedback` / `publish_odom` | `true` / `true` | 用 `0x221` 发布车辆里程计 |
| `receive_wheel_odometry` | `false` | 不使用 `0x311` 轮里程帧 |
| `receive_system_state` / `log_frames` | `false` / `false` | 关闭状态帧处理和逐帧日志 |
| CAN `odom_frame` / `base_frame` | `odom` / `base_link` | 必须和 SLAM TF 配置一致 |
| `send_period_ms` / `cmd_vel_timeout_ms` | `20` / `500` | CAN 节点默认控制周期与指令超时 |
| 静态 TF `x/y/z` | `0/0/0.18` m | 雷达相对车体平移 |
| 静态 TF `qx/qy/qz/qw` | `0/0/0/1` | 雷达相对车体旋转四元数 |
| 静态 TF `frame-id/child-frame-id` | `base_link/base_laser` | TF 父/子坐标系 |

`scout_can_node` 还声明 `control_vx_mm_s=0`、`control_vy_mm_s=0`、`control_wz_mrad_s=0`；当前由 `/cmd_vel` 控制时不用设置。其他 CAN 参数默认值及含义见 `src/scout_can/src/scout_can_node.cpp`。

LD19 launch 当前不传参数，驱动使用 `src/ldlidar_stl_ros2/src/ld19_scan_node.cpp` 中的默认值：`product_name=LDLiDAR_LD19`、`topic_name=scan`（ROS 话题 `/scan`）、`frame_id=base_laser`、`port_name=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`、`port_baudrate=230400`、`laser_scan_dir=true`、`enable_angle_crop_func=false`、`angle_crop_min=135.0`、`angle_crop_max=225.0`。要覆盖这些值，在 launch 的 LD19 `Node` 添加 `parameters=[{...}]`；改串口时通常只需改 `port_name`。

## 检查

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_laser
```

完整 TF 链应为 `map -> odom -> base_link -> base_laser`。SLAM Toolbox 保存普通栅格地图使用 `nav2_map_server map_saver_cli`；继续建图用的 SLAM 位姿图可通过 SLAM Toolbox 的序列化服务保存。
