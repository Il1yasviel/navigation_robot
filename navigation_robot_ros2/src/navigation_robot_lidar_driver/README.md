# navigation_robot_lidar_driver

该包由内部使用的 `lidar_pkg`（来源：`zhangwanjie/lidar_pkg_ros2`）整理而来，
保留其 150000 波特率数据协议和角度修正算法，并移除了服务器环境不需要的
OpenCV 界面。上游 `package.xml` 中许可证为 `TODO`，因此本包不声明对上游代码
的再分发权利，仅供本项目内部使用。

默认设备为 `/dev/navigation_lidar`，USB VID/PID 为 `34bf:ff0a`。实物型号尚待
确认，不能仅凭资料图片认定为 D1 EDU。
