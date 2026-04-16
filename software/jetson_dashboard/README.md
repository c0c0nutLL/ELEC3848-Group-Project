# Jetson Dashboard

这个目录提供一条完整链路：

- `Arduino -> USB Serial -> Jetson Nano -> Browser`
- `Browser -> Jetson HTTP API -> USB Serial -> Arduino`
- `Phone QR Scan -> Jetson Unlock Page -> Password Verify -> Serial OPEN_DOOR -> Arduino`

当前版本除了显示 telemetry，也支持从网页直接手动控制 Arduino 小车前后左右移动，并通过滑条调整电机功率。

## 主要文件

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - Jetson 主服务
  - 读取 Arduino 串口 JSON telemetry
  - 提供 dashboard 页面、unlock 页面和手动控制 API
- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - 实时状态页
  - 包含方向按钮、功率滑条、键盘控制和事件日志
- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - 手机扫码后访问的开仓页面
- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - 固定密码与开仓命令配置
- [requirements.txt](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/requirements.txt)
  - 当前只需要 `pyserial`

## 配套 Arduino 草图

当前配套的 Arduino 草图是：

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

这个草图现在提供：

- 电机方向命令：`DRIVE FORWARD/BACKWARD/LEFT/RIGHT/STOP <PWM>`
- `OPEN_DOOR` 命令
- 周期性 JSON telemetry
- 前方左右超声波距离上报
- 控制状态回传，例如 `control_direction`、`motor_pwm`、`control_status`

主要 JSON 字段包括：

- `state`
- `target_station`
- `right_cm`
- `left_cm`
- `avg_cm`
- `diff_cm`
- `heading_deg`
- `cargo_door`
- `delivery_result`
- `control_direction`
- `motor_pwm`
- `control_status`
- `fault`

## Jetson Nano 运行方式

进入目录：

```bash
cd software/jetson_dashboard
```

安装依赖：

```bash
pip3 install -r requirements.txt
```

启动服务：

```bash
python3 server.py --serial /dev/ttyUSB0 --baud 115200 --http-port 8080
```

如果 Arduino 枚举成别的设备名，也可能是：

- `/dev/ttyUSB0`
- `/dev/ttyACM1`

启动后可访问：

- Dashboard: `http://<jetson-ip>:8080/`
- Unlock 页面: `http://<jetson-ip>:8080/unlock`

## Dashboard 操作方式

手动控制页面在 `/`：

- 长按 `Forward / Backward / Left / Right` 按钮可持续运动
- 松开按钮会立即发送 `STOP`
- 中间 `Stop` 和 `Emergency Stop` 都会立即发送 `STOP`
- 可用滑条调整 PWM 功率
- 支持键盘 `W/A/S/D` 或方向键

Jetson 端控制接口：

- `POST /api/control`

请求体示例：

```json
{
  "direction": "FORWARD",
  "power": 120,
  "source": "dashboard"
}
```

## 无 Arduino 时的网页联调

如果只是先调网页和 API，可以使用 mock 模式：

```bash
python3 server.py --mock --http-port 8080
```

在 mock 模式下：

- dashboard 仍然可以收实时状态
- 手动控制按钮仍然会更新方向、功率和模拟距离
- `OPEN_DOOR` 仍然会触发模拟开门状态

## 当前 unlock 配置

默认配置文件是：

- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)

默认内容：

```json
{
  "robot_id": "amr01",
  "unlock_mode": "fixed",
  "fixed_password": "3848",
  "door_command": "OPEN_DOOR",
  "unlock_cooldown_ms": 5000
}
```

当前行为：

- 手机打开 `/unlock`
- 输入固定密码 `3848`
- Jetson 校验成功后，把 `OPEN_DOOR` 通过串口发给 Arduino

## 现阶段定位

这版代码已经适合做 Jetson 与 Arduino 的手动控制联调，但还不是最终 proposal 自动流程。

后续仍需要：

1. 把手动控制/演示状态替换成完整的 proposal 状态机
2. 接入真实 RFID、servo 和完整 docking 逻辑
3. 进一步细化失败状态、任务日志和演示脚本
