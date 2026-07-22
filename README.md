# ESP32-S3 Navigation Robot Lower Controller

本项目是双M0601轮毂电机和BMI088的ESP32-S3下位机。USB UART与WiFi TCP
使用同一套二进制协议，配套 `motor_test_gui.py` 可控制底盘和查看遥测。

## 硬件默认值

| 功能 | ESP32-S3引脚 |
|---|---:|
| USB/FTDI UART0 TX/RX | GPIO43 / GPIO44 |
| RS485 UART1 TX/RX | GPIO17 / GPIO18 |
| BMI088 SCK/MOSI/MISO | GPIO12 / GPIO11 / GPIO13 |
| BMI088加速度计/陀螺仪CS | GPIO47 / GPIO21 |

左电机ID固定为1，右电机ID固定为2。右轮安装方向相反，下位机自动反转
右轮命令和速度反馈；上位机只使用车体逻辑RPM。应用速度上限为125RPM。

自动换向TTL-RS485模块不需要RTS、DE或RE：GPIO17连接模块RX，GPIO18连接
模块TX，A+接A+，B+接B+，所有控制设备共地。

## 构建与WiFi

项目面向ESP-IDF 5.5.4，并已在5.4.3完成兼容构建验证：

```powershell
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p COM_PORT flash
```

在 `Navigation robot configuration` 中填写WiFi STA SSID和密码。凭据只保存
在已被Git忽略的本地 `sdkconfig`，不要写入源码。ESP32-S3仅支持2.4GHz网络。
TCP端口默认3333，mDNS主机名默认 `navigation-robot.local`。

## GUI

```powershell
python -m pip install -r requirements-host.txt
python motor_test_gui.py
```

GUI可以选择USB串口或WiFi TCP。点击“准备双轮控制”后才能使用摇杆或方向键。
数字键1～5对应25/50/75/100/125RPM，方向键控制前后和原地左右旋转。
测试时先架空车轮。

## 上位机目录结构

上位机代码位于 `host/` 包，按“纯逻辑 → 领域逻辑 → 传输层 → 界面层”分层：

| 模块 | 内容 |
|---|---|
| `host/config.py` | 限额、电机ID、超时/周期常量与界面文案映射 |
| `host/protocol.py` | 线协议常量、CRC16、`encode_frame`、`FrameParser` |
| `host/mapping.py` | 电流/位置换算、摇杆与键盘映射、控制使能标志 |
| `host/motion.py` | `MotionCommand`、`MotionCommandGate`（ACK门控与保活） |
| `host/telemetry.py` | `TelemetryRateMeter` 与底盘/IMU遥测载荷纯解析 |
| `host/link.py` | `HandshakeController`、`ResetDetector` |
| `host/transport.py` | `open_ftdi_serial`、`SerialWorker`、`TcpWorker` |
| `host/joystick.py` | `VirtualJoystick` 虚拟摇杆控件 |
| `host/app.py` | `MotorTestApp` 主界面与 `main()` |

两种启动方式等价：根目录 `python motor_test_gui.py`（兼容 shim）或
`python -m host`。

## 测试

```powershell
python -m unittest tests.host.test_protocol -v
cmake -S tests/native -B build-native
cmake --build build-native
ctest --test-dir build-native --output-on-failure
```

完整线协议见 `docs/host_protocol_v1.md`，后续ROS2上位机开发见根目录
`ROS2_HOST_DEVELOPMENT_GUIDE.md`。
