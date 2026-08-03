# navigation_robot_ros2

面向香橙派 5 Plus（Ubuntu Server 22.04 ARM64、ROS 2 Humble）的差速导航机器人
上位机工作区。默认配置只运行传感器，机械参数未实测前不会向底盘发送运动命令。

ROS 包维护者使用 `air <air@localhost.local>`；ROS 元数据校验不接受不带点号的
`air@localhost`，因此采用语义相同的本地域名占位写法。

## 当前状态

- 已实现 ESP32-S3 USB/TCP 上位机协议、轮速里程计、BMI088、诊断、急停和重连。
- 已重构 150000 波特率二维雷达驱动，默认设备 `/dev/navigation_lidar`。
- 已纳入 Navigation2 1.1.20 和 SLAM Toolbox 2.6.10 完整源码。
- 已配置 EKF、SLAM Toolbox、DWB、速度平滑、twist_mux 和 Collision Monitor。
- 已提供 `navigation_robot_remote_control`，可从 Ubuntu 电脑经 ROS 2 DDS 无线
  发布 `/cmd_vel_teleop`，并保留底盘诊断门控、Collision Monitor 与多级超时停车。
- 相机型号和所有机械尺寸待测，因此相机默认关闭、`motion_enabled` 默认关闭。

## 构建

```bash
cd /home/air/ros2_ws_projects/navigation_robot_ros2
./scripts/install_dependencies.sh
./scripts/build_workspace.sh
source install/setup.bash
```

香橙派内存有限时：

```bash
COLCON_WORKERS=2 ./scripts/build_workspace.sh
```

## 配置与启动

日常只修改：

```text
src/navigation_robot_bringup/config/robot_config.yaml
```

完成 `docs/HARDWARE_MEASUREMENT.md` 前保持安全锁。总启动命令：

```bash
ros2 launch navigation_robot_bringup robot.launch.py
```

模式也可临时覆盖：

```bash
ros2 launch navigation_robot_bringup robot.launch.py operation_mode:=mapping
ros2 launch navigation_robot_bringup robot.launch.py operation_mode:=navigation
```

远程键盘建图：

```bash
ros2 run navigation_robot_remote_control keyboard_teleop --ros-args --params-file \
  "$(ros2 pkg prefix --share navigation_robot_remote_control)/config/remote_control.yaml"
```

显式保存地图，不覆盖同名文件：

```bash
ros2 launch navigation_robot_navigation save_map.launch.py map_name:=office
```

远程 RViz：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch navigation_robot_navigation rviz.launch.py
```

## 协议模拟器

无需硬件即可生成伪串口：

```bash
python3 src/navigation_robot_base_driver/scripts/lower_controller_emulator.py \
  --symlink /tmp/navigation_base
```

然后将总配置中的串口改成 `/tmp/navigation_base`，启动传感器模式检查
`/wheel/odometry`、`/imu/data_raw` 和 `/diagnostics`。

真实底盘接入后可在香橙派终端执行无运动安全检测：

```bash
./scripts/check_real_base.sh
```

脚本固定使用 `motion_enabled=false` 和零几何参数，只发送协议握手、查询、速度模式
准备和最终停车命令，同时读取左右轮反馈、IMU 与诊断；它不发送运动 RPM。

## 部署注意事项

- `scripts/install_udev_rules.sh` 会在底盘规则仍为 `0000:0000` 时拒绝安装；必须先
  根据真实 FTDI/ESP32 设备填写 VID/PID，最好同时写入序列号。
- 相机按 `docs/CAMERA_SELECTION.md` 确认后只复制一套驱动。
- systemd 用户服务可用 `scripts/install_user_service.sh` 安装，但不会自动 enable。
- Wi-Fi 组播发现失败时，在两端 Cyclone DDS 配置中加入静态 peers。
