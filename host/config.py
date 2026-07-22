"""电机限额、超时/周期常量与界面文案映射。"""

M0601_BRAKE_OFF = 0x00
M0601_BRAKE_ON = 0xFF
M0601_RESERVED_QUERY_ID = 0xC8
M0601_MODE_CURRENT = 0x01
M0601_MODE_SPEED = 0x02
M0601_MODE_POSITION = 0x03
MAX_CURRENT_MA = 1000.0
MAX_RPM = 125
SPEED_GEARS = (25, 50, 75, 100, 125)
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 2
SET_ID_CONFIRM = 0x4D36

HANDSHAKE_INITIAL_DELAY_S = 1.0
HANDSHAKE_RETRY_S = 0.3
HANDSHAKE_TIMEOUT_S = 2.5
RESET_WINDOW_S = 5.0
RESET_MARKER = b"ESP-ROM:"
OPEN_TIMEOUT_S = 6.0
STARTUP_PURGE_MAX_S = 1.5
STARTUP_PURGE_QUIET_READS = 3
POLL_EVENT_LIMIT = 48
POLL_TIME_BUDGET_S = 0.008
MOTION_ACK_TIMEOUT_S = 0.250
CONTROL_KEEPALIVE_PERIOD_S = 0.100
TELEMETRY_RATE_WINDOW_S = 2.0

STATUS_TEXT = {
    0: "成功",
    1: "主机CRC错误",
    2: "长度错误",
    3: "参数越界",
    4: "控制权被占用",
    5: "电机响应超时",
    6: "电机CRC错误",
    7: "前置条件不满足",
    8: "不支持的命令",
    9: "I/O错误",
}

MOTOR_STATE_TEXT = {0: "离线", 1: "空闲", 2: "运行", 3: "故障", 4: "急停"}
MODE_TEXT = {
    M0601_MODE_CURRENT: "电流模式",
    M0601_MODE_SPEED: "速度模式",
    M0601_MODE_POSITION: "位置模式",
}
MODE_BY_TEXT = {text: mode for mode, text in MODE_TEXT.items()}
FAULT_NAMES = ("传感器", "过流", "相线过流", "堵转",
               "故障位4", "保留5", "保留6", "保留7")
