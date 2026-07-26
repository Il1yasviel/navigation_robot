# 实车测量与解锁清单

在 `robot_config.yaml` 保持 `motion_enabled: false` 的条件下完成以下工作。

1. 有效轮半径：让驱动轮带载滚动至少 10 圈，用总距离除以 `20π`；左右轮分别测量并取初值，后续用直线误差校正。
2. 有效轮距：先测两个轮胎接地点中心距；再架设外部角度基准，完成至少 5 圈原地旋转，用实际角度与轮速里程计角度的比例校正。
3. `base_footprint`：取驱动轮轴线中点在地面的投影，测量车体相对该点的前、后、左、右最大边界。
4. 传感器位姿：从 `base_link` 测量雷达、相机和 IMU 的 `[x,y,z,roll,pitch,yaw]`，确认 BMI088 为 X 前、Y 左、Z 上。
5. 将 footprint 同步写入 `nav2_params.yaml`，将其外扩 0.10 m、0.25 m 后分别写入 Collision Monitor 停车区和减速区。
6. 架空轮完成 0、±25 RPM、看门狗和急停测试后，才把 `motion_enabled` 改为 `true`。
