# LD19 + Scout Mini 建图与导航

ROS 2 Jazzy；Scout Mini CAN `0x221` 提供 `/odom`，LD19 提供 `/scan`。建图用 SLAM Toolbox，导航用 Nav2 + AMCL。当前没有 IMU，也没有独立激光里程计。

## 准备

新环境安装并编译；已完成可跳过：

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-teleop-twist-keyboard can-utils
source /opt/ros/jazzy/setup.bash
cd /home/yly/ld19_hand_on
colcon --log-base /home/yly/ld19_field_log build --symlink-install \
  --build-base /home/yly/ld19_field_build \
  --install-base /home/yly/ld19_field_install \
  --packages-up-to scout_slam_toolbox scout_bringup
```

首次配置 CAN 开机启动：

```bash
sudo install -m 644 /home/yly/ld19_hand_on/systemd/scout-can0.service /etc/systemd/system/scout-can0.service
sudo systemctl daemon-reload
sudo systemctl enable --now scout-can0.service
ip -details link show can0
```

LD19 串口需当前用户有 `dialout` 权限。下面的车辆命令会检查 `can0` 并发送 `0x421` 使能帧。

## 日常使用

建图（启动雷达、车辆里程计、SLAM Toolbox、RViz 和键盘）：

```bash
source ~/.bashrc
scout_mapping
```

键盘 `i` 前进、`j`/`l` 转向、`u`/`o` 边前进边转、`k` 停车；`Ctrl+C` 结束。当前建图键盘默认上限为 1.0 m/s、1.5 rad/s，先在空旷处低速试车。

建图时在第二个终端保存地图：

```bash
source ~/.bashrc
scout_save_map
```

默认生成 `/home/yly/ld19_field_data/my_map.yaml` 和 `.pgm`；另存用 `scout_save_map /保存目录 地图名`。保存后退出建图，再启动导航；两个 launch 不能同时运行：

```bash
source ~/.bashrc
scout_navigation
# 其他地图：scout_navigation map:=/绝对路径/地图.yaml
```

先等定位节点启动，在 RViz 用 **2D Pose Estimate** 设置车的实际位置和朝向；确认扫描与地图对齐、终端出现导航管理器的 `Managed nodes are active` 后，再用 **Nav2 Goal** 发送近距离目标。若提示 `Action server is inactive`，先不要重复发目标，检查 AMCL 初始位姿和启动日志。`scout_navigation` 当前将 ROS 自动发现限制在本机；SSH 到这台车上运行命令可以使用，另一台电脑上的 ROS 2 节点不会自动发现它。

## 里程计标定

当前前进系数 `odom_vx_scale` 为 `0.9`，横移和旋转系数仍为 `1.0`；这些都还需要用实测距离复核。退出建图和导航，在平整空地标记起点，分别运行：

```bash
# 终端一：里程计 + RViz
source ~/.bashrc
scout_can_odometry_rviz

# 终端二：低速键盘控制
source ~/.bashrc
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.3

# 终端三：读取里程计位置和偏航角
source ~/.bashrc
ros2 run tf2_ros tf2_echo odom base_link
```

用 `i` 前进约 2 米、大写 `J`/`L` 横移约 2 米、小写 `j`/`l` 原地转约 90°；每次按 `k` 停车。分别比较实际运动量与 TF 变化量，计算 `新系数 = 旧系数 × 实际值 / 里程计变化量`，修改[车辆配置](src/scout_bringup/config/scout_bringup_config.yaml)并重启测试。正反方向都测；方向相反时先查坐标系和 CAN 反馈。

## 数据流与配置

```text
CAN 0x221 -> /odom + odom→base_link ─┐
LD19      -> /scan                    ├─ 建图：SLAM Toolbox -> /map + map→odom
base_link -> base_laser               └─ 导航：AMCL 读取已存地图 -> map→odom
```

- [车辆配置](src/scout_bringup/config/scout_bringup_config.yaml)：地图路径、雷达安装位置、CAN 接口和里程计系数。
- [建图配置](src/scout_slam_toolbox/config/mapper_params_online_async.yaml)：地图分辨率、扫描匹配、回环、新扫描处理阈值。
- [导航配置](src/scout_bringup/config/nav2_navigation_params.yaml)：AMCL、规划、控制器、代价地图和速度平滑。修改后重启导航。

导航当前用 RPP 跟踪路径，设定巡航线速度 `0.65 m/s`、速度平滑器角速度上限 `0.8 rad/s`；接近障碍或目标时会减速。车体半径暂设 `0.22 m`，障碍膨胀半径 `0.45 m`，都还没按实车尺寸复核。自主导航目前没有横移指令；键盘控制可以横移。

### AMCL 重点参数

AMCL 先根据里程计预测车可能在哪里，再用 `/scan` 和已有地图给各个**粒子**（候选位姿）打分、重采样。以下是本项目当前值；完整定义见 [Nav2 Jazzy AMCL 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `robot_model_type` | `OmniMotionModel` | 麦轮车的全向运动模型 |
| `alpha1`–`alpha5` | 各 `0.2` | 里程计运动噪声；调大使预测粒子更分散，不是速度标定系数 |
| `min_particles` / `max_particles` | `500` / `2000` | 自适应粒子数上下限；更多粒子更耗 CPU，未必更准 |
| `pf_err` / `pf_z` | `0.05` / `0.99` | 决定自适应粒子数的误差和置信度要求 |
| `laser_model_type` / `max_beams` | `likelihood_field` / `60` | 用地图障碍距离为扫描打分；每帧取最多 60 束 |
| `sigma_hit` / `z_hit` / `z_rand` | `0.2` / `0.5` / `0.5` | 扫描匹配容差；地图命中与随机测量的权重 |
| `update_min_d` / `update_min_a` | `0.25 m` / `0.2 rad` | 位移或转角达到阈值才更新；过大会使定位修正滞后 |
| `recovery_alpha_fast` / `recovery_alpha_slow` | `0` / `0` | 当前关闭随机位姿恢复；明显定位丢失时可在 RViz 重设初始位姿 |
| `laser_max_range` / `do_beamskip` | `-1.0` / `false` | 采用 `/scan` 的最大距离；当前不跳过不匹配的光束 |

`alpha1` 是转动造成的转角噪声，`alpha2` 是平移造成的转角噪声，`alpha3` 是平移造成的位置噪声，`alpha4` 是转动造成的位置噪声，`alpha5` 是全向模型额外的平移噪声。这些值描述**预测的不确定性**，不直接改写 CAN 里程计，也不等于 `/odom` 的协方差。

若车在地图里左右跳动，先核对里程计、`base_link→base_laser` 安装 TF、雷达时间戳和地图墙面是否重影。一次只调一组参数；增加粒子数不能修正有偏差的里程计。

## 快速检查

```bash
ip -br link show can0
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_laser
```

导航时 TF 链应为 `map → odom → base_link → base_laser`。后续实车验证见 [待做.md](待做.md)，技术改进见 [改进方向.md](改进方向.md)。
