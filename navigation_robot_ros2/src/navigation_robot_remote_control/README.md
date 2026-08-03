# navigation_robot_remote_control

Ubuntu 开发电脑上运行的安全键盘遥控节点。它不直接连接 ESP32 的 Wi-Fi TCP，
而是通过 ROS 2 DDS 无线发布 `/cmd_vel_teleop`，由香橙派上的 `twist_mux`、
Collision Monitor 和底盘驱动转换成 USB 串口双轮命令。

## 建图直通行为

- 直接向香橙派 `/cmd_vel` 发布，不经过 twist_mux 或 Collision Monitor。
- 方向命令持续发布，直到新方向、空格或 Q；空格立即停车，Q 停车后退出。
- 五档速度与原始上位机一致：25/50/75/100/125 RPM；按当前模型尺寸换算，
  最大线速度为 0.6545 m/s、原地角速度为 5.2360 rad/s。
- 节点退出时连续发布三次零速度；电脑失联时仍有底盘 0.2 秒命令超时和
  下位机 0.3 秒看门狗。

## 编译

在 Ubuntu 22.04 + ROS 2 Humble 电脑中，将本包放到任意工作区的 `src/`：

```bash
cd ~/navigation_robot_pc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select navigation_robot_remote_control
source install/setup.bash
```

## 启动

两端使用相同 DDS 配置：

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop
ros2 run navigation_robot_remote_control keyboard_teleop \
  --ros-args --params-file \
  "$(ros2 pkg prefix --share navigation_robot_remote_control)/config/remote_control.yaml"
```

键位：W/S 前后，A/D 原地左右转，也支持方向键；1～5 选择速度档位；空格停车；
Q 停车并退出。

SLAM 时香橙派必须启动 `operation_mode:=mapping` 并设置 `motion_enabled=true`。
传感器模式下底盘驱动会忽略所有运动命令。

## 电脑本地显示导航地图

部分 Wi-Fi/路由器会丢弃 DDS 的大尺寸 OccupancyGrid 数据，而 `/scan` 等小消息
仍可正常工作。将地图 YAML 和 PGM 复制到电脑后，可在电脑本地发布仅供 RViz
显示的 `/map_pc`；AMCL、规划和底盘控制仍全部运行在香橙派。

```bash
ros2 run navigation_robot_remote_control local_map_publisher \
  --ros-args -p map_yaml:=/home/air/navigation_robot_maps/home_01.yaml
```

RViz 使用 `Fixed Frame: map`，Map 的 Topic 选择 `/map_pc`。节点同时发布一个
无冲突的 `map -> map_visualization_anchor` 静态 TF，使 AMCL 接收初始位姿前也能
选择 `map`。然后使用 `2D Pose Estimate` 和 `2D Goal Pose`；小尺寸的初始位姿、
目标和 TF 仍经 ROS 2 无线发送。

## 从 GitHub 稀疏拉取

本包提交到主仓库后，可以下载并运行仓库中的一键拉取/更新/编译脚本：

```bash
curl -fsSL \
  https://raw.githubusercontent.com/Il1yasviel/navigation_robot/main/navigation_robot_ros2/scripts/bootstrap_remote_control_pc.sh \
  -o /tmp/bootstrap_remote_control_pc.sh
bash /tmp/bootstrap_remote_control_pc.sh
```

也可以在电脑工作区的 `src/` 中手动只拉取本目录：

```bash
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/Il1yasviel/navigation_robot.git navigation_robot_source
git -C navigation_robot_source sparse-checkout set \
  navigation_robot_ros2/src/navigation_robot_remote_control
```

随后从工作区根目录执行上面的 `rosdep` 和 `colcon build` 命令。当前香橙派目录
是 GitHub 源码快照而不是 Git 工作区；在本包实际提交到 GitHub 前，该拉取命令
不会获得本地新增内容。
