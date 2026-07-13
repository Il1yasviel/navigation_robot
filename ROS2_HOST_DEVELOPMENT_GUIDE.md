# ROS2上位机开发指导

本文档用于后续编写ROS2上位机。当前仓库不包含ROS2节点；下位机已经提供
双轮速度、电流、位置、故障以及BMI088六轴原始数据。

## 1. 连接方式

- USB：UART0，115200、8N1，无流控。
- WiFi：ESP32-S3以STA方式连接2.4GHz局域网，TCP端口3333。
- 默认mDNS：`navigation-robot.local`，也可以使用DHCP分配的IPv4地址。
- USB和TCP使用完全相同的帧格式。TCP是字节流，不能假设一次 `recv()` 返回
  完整一帧。
- 真实WiFi密码不写入本文件或源码。通过ESP-IDF `menuconfig` 配置。

连接后发送带ACK请求的空载荷 `0x01 HELLO`，收到成功ACK后再发其他命令。
断线重连必须重新HELLO、重新确认双轮状态，并清除旧序号和旧运动目标。

## 2. 帧解析

```text
AA 55 | version:u8 | type:u8 | sequence:u8 | flags:u8 |
payload_length:u16_le | payload | crc16_le
```

- 版本固定1，载荷最大128字节。
- flags bit0表示成功时需要ACK；错误始终返回ACK。
- CRC16/CCITT-FALSE：poly 0x1021、init 0xFFFF、非反射、xorout 0。
- CRC范围从version开始，到payload结束，不包括 `AA 55` 和CRC本身。
- 解析器应扫描 `AA 55` 重同步，拒绝超长载荷和错误CRC，并保留可能被拆开的
  单个 `AA` 字节。

Python伪代码：

```python
buffer += sock.recv(4096)
while True:
    start = buffer.find(b"\xAA\x55")
    if start < 0:
        buffer = buffer[-1:] if buffer.endswith(b"\xAA") else b""
        break
    buffer = buffer[start:]
    if len(buffer) < 8:
        break
    payload_len = int.from_bytes(buffer[6:8], "little")
    total = 10 + payload_len
    if len(buffer) < total:
        break
    # validate version, length and CRC before consuming total bytes
```

完整消息表和逐字节遥测偏移见 `docs/host_protocol_v1.md`。

## 3. 双轮控制

固定配置：左轮ID1、右轮ID2、速度范围±125RPM。发送 `0x1A`：

```text
left_id=1, right_id=2,
left_logical_rpm:i16, right_logical_rpm:i16,
accel:u8, brake:u8
```

逻辑正RPM表示车轮驱动车体前进。右电机物理方向相反，但反向只在下位机
执行；ROS2节点不得再次取反右轮。

```text
前进: (+v, +v)     后退: (-v, -v)
左转: (-v, +v)     右转: (+v, -v)
```

运动命令成功后每100ms发送一次无ACK请求的 `0x1B`，载荷为 `01 02`。
超过300ms没有有效保活，下位机会刹停。停止或急停发送 `0x1C`，载荷为
`01 02 FF`。停止命令可以由非当前控制来源执行。

USB和TCP之间有单一控制权：第一个成功运动命令取得控制权，另一连接的运动
命令返回状态4；查询不占用控制权。断线、停止或看门狗会释放控制权。

## 4. ROS2 `/cmd_vel` 换算

创建节点前必须提供：

- `wheel_radius_m`：轮子有效半径R。
- `wheel_separation_m`：左右轮接地点中心距W。

对于 `geometry_msgs/Twist` 的线速度 `v` 和角速度 `w`：

```python
left_rad_s = (v - w * W / 2.0) / R
right_rad_s = (v + w * W / 2.0) / R
left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)

scale = max(1.0, abs(left_rpm) / 125.0, abs(right_rpm) / 125.0)
left_rpm /= scale
right_rpm /= scale
```

推荐订阅 `/cmd_vel`，以10～20Hz接收目标；仅在目标变化时发送 `0x1A`，保活
独立以10Hz发送。不要用不断重复运动帧代替保活。

## 5. 轮子遥测

订阅下位机 `0x92`，频率约10Hz。左右记录中的目标和实际RPM已经是车体逻辑
方向。建议发布：

- `sensor_msgs/JointState`：左右轮速度，名称 `left_wheel_joint`、
  `right_wheel_joint`，RPM转换为rad/s。
- 自定义诊断或 `diagnostic_msgs/DiagnosticArray`：电流、模式、故障、反馈年龄、
  CRC错误和超时。
- 配置R和W后，可由左右轮速度积分发布 `nav_msgs/Odometry` 和 `odom→base_link`
  TF。

M0601位置字段尚未验证为可靠的多圈累计编码器。必须完成回绕处理和实车标定
后才能用于长期里程计；初期建议按逻辑轮速积分，并融合雷达与IMU修正漂移。

## 6. IMU数据

`0x93` 以100Hz发送，字段已经换算为SI单位：

- X向前、Y向左、Z向上。
- 加速度单位m/s²。
- 角速度单位rad/s。
- 时间戳是ESP32启动后的单调微秒，不是Unix时间。
- 状态bit0在线、bit1陀螺仪零偏校准完成、bit2样本有效。

建议发布 `sensor_msgs/Imu` 到 `/imu/data_raw`：

```text
header.frame_id = "imu_link"
linear_acceleration = accel_xyz
angular_velocity = gyro_xyz
orientation_covariance[0] = -1   # 下位机没有输出姿态四元数
```

启动后的约2.5秒必须保持车辆静止以完成500样本三轴零偏校准。bit1和bit2未置位
时不要把数据送入导航滤波器。ROS主机可在首次有效样本到达时记录
`host_time - mcu_timestamp`，把单调时间映射到ROS时钟，并持续检查回退或复位。

## 7. 推荐ROS2节点结构

```text
transport thread (serial or TCP)
  -> byte stream parser
  -> ACK dispatcher / telemetry queues
  -> cmd_vel controller (0x1A + 0x1B + 0x1C)
  -> wheel publisher (0x92)
  -> imu publisher (0x93)
```

- 接收线程不得在ROS回调中阻塞。
- ACK按序号和请求类型匹配；断线时清空所有在途请求。
- 每次最多保留一个等待ACK的运动命令，连续目标采用最新值覆盖。
- 急停绕过普通运动队列立即发送。
- 反馈年龄超过200ms告警，超过500ms视为对应电机离线。
- 100Hz IMU应使用最新值队列或有界队列，禁止无限积压。

## 8. 联调顺序

1. 架空两轮，仅HELLO并接收 `0x92/0x93`。
2. 查询ID1、ID2并分别切换到速度模式。
3. 发送0RPM，再测试±25RPM，确认车体逻辑方向一致。
4. 连续保活30秒，确认每轮反馈8～12Hz、IMU约100Hz。
5. 中断TCP或USB，确认300ms内双轮停止。
6. 提供实测R和W后再启用 `/cmd_vel`、里程计和导航。
