# 奥比中光相机选择

当前实物型号未知，因此工作区不会擅自复制错误驱动。连接相机后先记录 `lsusb`、
产品标签和序列号：

- 若设备能被资料中的 `OrbbecSDK_ROS2` 正常枚举，复制该目录为
  `src/drivers/orbbec_camera_ros2`，保留内部 `orbbec_camera*` 包名。
- 否则对 Astra、Dabai、旧 Gemini 使用 `ros2_astra_camera`，外层目录命名为
  `src/drivers/astra_camera_ros2`。
- 在 `robot_config.yaml` 中填写 `camera.driver` 与该型号对应的 `launch_file`，
  再开启 `use_camera`。

两套资料均含 ARM64 动态库；仍必须在香橙派上使用真实设备完成编译、启动、
热插拔和 30 分钟连续运行测试。
