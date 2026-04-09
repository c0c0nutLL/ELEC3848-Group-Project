# Jetson Dashboard And Unlock Draft

这个目录现在包含两条功能链路：

- `Arduino -> USB Serial -> Jetson Nano -> Browser`
- `Phone QR Scan -> Jetson Unlock Page -> Password Verify -> Serial OPEN_DOOR -> Arduino`

## 文件说明

- [server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
  - Jetson 主服务
  - 读取 Arduino 串口 JSON 状态
  - 提供 dashboard 页面
  - 提供 unlock 页面
  - 提供固定密码校验接口
  - 向 Arduino 下发 `OPEN_DOOR`

- [dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
  - 实时遥测页面

- [unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)
  - 手机扫码后打开的固定密码开仓页面

- [unlock_config.json](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock_config.json)
  - 当前开仓配置
  - 现在先使用固定密码模式

- [requirements.txt](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/requirements.txt)
  - 目前只需要 `pyserial`

## Arduino 端配套草稿

配套 Arduino 草稿在：

- [telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)

它现在做两件事：

- 每 `200 ms` 发送一条 JSON 遥测状态
- 监听来自 Jetson 的 `OPEN_DOOR` 命令

收到 `OPEN_DOOR` 后，这个 demo 会临时把状态切成：

- `state = REMOTE_UNLOCK`
- `cargo_door = OPEN`
- `delivery_result = UNLOCKED`

这只是联调用的占位逻辑。后面接真实舵机时，把这里替换成真正的 servo 开门代码即可。

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
python3 server.py --serial /dev/ttyACM0 --baud 115200 --http-port 8080
```

如果 Arduino 枚举成别的设备名，也可能是：

- `/dev/ttyUSB0`
- `/dev/ttyACM1`

启动后可访问：

- Dashboard
  - `http://<jetson-ip>:8080/`
- Unlock 页面
  - `http://<jetson-ip>:8080/unlock`

## 没接 Arduino 时的测试方式

如果只是先测网页链路：

```bash
python3 server.py --mock --http-port 8080
```

这样 dashboard 会自己显示演示状态。  
注意：`--mock` 只适合测页面，不适合真串口开仓联调。

## 当前固定密码配置

当前配置文件是：

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

现在的行为是：

- 手机打开 `/unlock`
- 输入固定密码 `3848`
- Jetson 校验成功后，把 `OPEN_DOOR` 通过串口发给 Arduino
- 冷却时间默认 `5 秒`

## 这版的定位

这是第一版“先把开仓链路打通”的代码，不是最终安全版本。

当前已完成：

- 固定密码页面
- Jetson 端密码验证
- Jetson 到 Arduino 的串口开门命令
- Dashboard 与 unlock 页面共存

下一步可以继续做：

1. 把 `OPEN_DOOR` 从 demo 状态改成真实舵机动作
2. 记录每次开仓日志
3. 限制只有特定状态下才允许开仓
4. 升级成随机密码 / 一次性密码
