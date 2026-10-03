# LD19 + Scout Mini 二维建图与导航

ROS 2 Jazzy 工作空间。建图使用 SLAM Toolbox，Nav2 使用 AMCL 定位；里程计统一来自 Scout Mini CAN `0x221`，由 `scout_can_node` 发布 `/odom` 和 `odom -> base_link`。不启动独立激光里程计节点。

明天实车验证与 Codex 交接见 [待做.md](待做.md)，后续路线见 [改进方向.md](改进方向.md)。

## 使用

安装需要的 ROS 包（已有则跳过）：

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server can-utils
```

编译：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yly/1d19
# 原 build/install 含旧工作空间路径，使用独立目录。
colcon --log-base /home/yly/1d19_field_log build --symlink-install \
  --build-base /home/yly/1d19_field_build \
  --install-base /home/yly/1d19_field_install \
  --packages-up-to scout_slam_toolbox scout_bringup
source /home/yly/1d19_field_install/setup.bash
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

实车默认 `use_sim_time:=false`。该 launch 启动 LD19、雷达静态 TF、CAN 里程计和 SLAM Toolbox。需要键盘控制时另开终端运行 `ros2 run teleop_twist_keyboard teleop_twist_keyboard`；若底盘尚未使能，可发送 `cansend can0 421#01`。

建图时保存地图，文件名不带扩展名：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/yly/1d19/src/scout_bringup/maps/my_map
```

保存后先退出建图，再启动 Nav2 定位与导航（两套 launch 都会启动硬件节点）：

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

`scout_can_node` 将 CAN `0x221` 的反馈速度 `vx/vy/wz` 校准后，按内核接收时间积分（相邻速度平均、区间中间朝向），发布车辆里程计。静态 TF 描述雷达安装位姿。SLAM Toolbox 以车辆里程计预测扫描位姿，再用激光扫描匹配建立位姿图；车辆回到已走过的区域时，回环检测添加约束并优化位姿图。它发布 `/map` 和 `map -> odom`，车辆 CAN 仍是 `odom -> base_link` 的唯一来源。激光扫描匹配是 SLAM 建图的一部分，不是独立激光里程计。

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

新建图 launch 在 `src/scout_slam_toolbox/launch/ld19_slam_toolbox_mapping.launch.py`，目前命令行 launch 参数只有 `use_sim_time`（默认 `false`）。它复用 `src/scout_bringup/launch/ld19_vehicle.launch.py` 启动雷达、CAN 和静态 TF；CAN 基础参数统一在 `src/scout_bringup/config/scout_bringup_config.yaml` 修改：

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

车辆里程计还支持以下参数，均在同一配置文件的 `scout_can` 下修改，重启生效：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `odom_vx_scale` / `odom_vy_scale` / `odom_wz_scale` | `1.0` / `1.0` / `1.0` | 前进、横移、旋转反馈的正数标定系数；不改变控制指令 |
| `odom_vx_stddev` / `odom_vy_stddev` / `odom_wz_stddev` | `0.03` / `0.06` / `0.03` | 校准后速度标准差，单位 m/s、m/s、rad/s；初始估计，需实测 |
| `odom_timeout_sec` | `0.2` 秒 | 相邻反馈最大间隔；超时或时钟回退时不积分该段，增加不确定性并告警 |

时间戳使用 SocketCAN `SO_TIMESTAMPNS` 的系统接收时间，不是车辆采样时间。积压帧各自保留接收时间；缺少时间戳的帧丢弃。实车 CAN 里程计要求 `use_sim_time=false`；回放时直接使用已录制的 `/odom` 和 TF，不启动实车 CAN。停止反馈后不会继续预测或发布里程计。

速度协方差对角线为上述标准差的平方；位姿协方差按积分模型传播，未测量的 Z、roll、pitch 设置大方差。这个近似模型不能准确描述长期偏差、相关噪声或打滑，反馈中断时的方差增量也只是经验估计，不代表恢复了缺失位移。

标定时保持系数为 `1.0`，分别前进、横移约 2 米、原地旋转一圈，记录实际位移与 `/odom` 的位移或累计转角，多次重复后计算 `系数 = 实际值 / 里程计值`。旋转用逐帧 yaw 差归一化后累加，不能用首尾四元数直接求整圈转角。已有系数时，新系数为 `旧系数 × 实际值 / 里程计值`。例如横移实际 2 米、里程计 2.2 米，设置 `odom_vy_scale: 0.9091`。方向错误应核对坐标系及协议。默认误差参数不代表已完成实车标定；没有独立 IMU，暂不增加 EKF。

LD19 launch 当前不传参数，驱动使用 `src/ldlidar_stl_ros2/src/ld19_scan_node.cpp` 中的默认值：`product_name=LDLiDAR_LD19`、`topic_name=scan`（ROS 话题 `/scan`）、`frame_id=base_laser`、`port_name=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`、`port_baudrate=230400`、`laser_scan_dir=true`、`enable_angle_crop_func=false`、`angle_crop_min=135.0`、`angle_crop_max=225.0`。要覆盖这些值，在 `ld19_vehicle.launch.py` 的 LD19 `Node` 添加 `parameters=[{...}]`；改串口时通常只需改 `port_name`。

## 检查

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_laser
```

完整 TF 链应为 `map -> odom -> base_link -> base_laser`。SLAM Toolbox 保存普通栅格地图使用 `nav2_map_server map_saver_cli`；继续建图用的 SLAM 位姿图可通过 SLAM Toolbox 的序列化服务保存。

## 麦轮建图与导航改进

本仓库专用于 Scout Mini Omni 麦轮款：CAN 控制与里程计固定使用 `vx/vy/wz`，AMCL 使用 `nav2_amcl::OmniMotionModel`。RPP 和速度平滑器仍保持前进与转向导航；局部代价地图使用二维 `ObstacleLayer`。

建议按以下顺序改进：

1. **横移导航**：改用 DWB 或 MPPI 的 Omni 模型，并同步设置控制器与 `velocity_smoother` 的 Y 方向速度、加速度及减速度。限值须根据实车测试确定。
2. **车辆轮廓**：实测长宽与雷达安装位置，局部代价地图使用矩形 `footprint`。当前 NavFn 全局规划按圆形近似检查碰撞，全局半径应包住实际车体；狭窄通道再考虑支持完整轮廓的规划器。
3. **里程计标定**：分别测试前进、横移、旋转，核对 CAN `0x221` 的反馈方向、单位和实际位移；注意麦轮横移打滑。后续可增加 IMU 与车辆里程计融合，但保留唯一的 `odom -> base_link` 发布者。
4. **扫描时序与建图**：低速建图，检查雷达时间戳、里程计延迟和墙面是否重影；录制 `/scan`、`/odom`、`/tf`、`/tf_static` 后回放调参。

参考：[Nav2 控制器选择](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/)。

## 车辆 CAN 协议核对

当前驱动只实现 AgileX **协议 V2**（`0x111` 控制、`0x221` 反馈），不做 V1/V2 自动检测。以下字节编号从 0 开始，16 位数为有符号、大端编码：

| 字节 | `0x111` 控制 / `0x221` 反馈 |
|---|---|
| 0–1 | 前进速度，mm/s（ROS `linear.x × 1000`） |
| 2–3 | 转向角速度，mrad/s（ROS `angular.z × 1000`） |
| 4–5 | 麦轮版横向速度，mm/s（ROS `linear.y × 1000`） |
| 6–7 | 当前 Scout 控制不使用，发送零 |

官方 ROS 2 驱动的 Omni 分支调用 `SetMotionCommand(linear.x, angular.z, linear.y)`；V2 SDK 将横向速度写入控制帧第 4–5 字节，并从反馈帧同位置读取。用户已确认本车为麦轮款，因此仓库固定使用该协议，不设横移开关。语雀手册页面尚未能读取，以上字段依据厂商官方 SDK。横移自主导航还需要调整控制器及速度平滑器。

依据：[厂商 SDK 协议定义](https://github.com/agilexrobotics/ugv_sdk/blob/main/src/protocol_v2/agilex_protocol_v2.h)、[编码/解码实现](https://github.com/agilexrobotics/ugv_sdk/blob/main/src/protocol_v2/agilex_msg_parser_v2.c)、[厂商 ROS 2 车型说明](https://github.com/agilexrobotics/scout_ros2)。

在建图 launch 运行、Nav2 尚未启动时，可单独检查低速横移；观察实际方向及 `/odom.twist.twist.linear.y`：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.05}, angular: {z: 0.0}}'
# Ctrl+C 后，CAN 指令超时 500 ms 自动归零。
```

对应 V2 控制数据应为 `0x111: 00 00 00 00 00 32 00 00`。检查命令为 `candump can0,111:7FF,221:7FF`。若命令正确却不横移，核对 CAN 控制模式、遥控器状态和固件对 Omni 字段的支持。
