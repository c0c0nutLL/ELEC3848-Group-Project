# ELEC3848 Group Project

这个仓库保存 ELEC3848 Integrated Design Project 的代码、硬件记录和 Jetson Nano 上位机开发内容。

当前项目主线已经从 `Required Function` 阶段转到 proposal 阶段。  
后续总体规划以根目录的 [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md) 为准，长期线程记忆以 [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md) 为准。

## 当前技术基线

最新硬件与控制决策如下：

- 侧边两个 ultrasonic 已移除
- 不再使用侧边墙距做 corridor guidance
- 长距离直行改为 `MPU6050 gyro + 四轮 PWM tuning / trim`
- 前方 ultrasonic 仍然保留，用于靠站、停车、近距离安全
- 机械臂双舵机保留：
  - `pin 44` gripper
  - `pin 45` lift

这意味着：

- 不要再把“移除侧边 ultrasonic”理解成“项目里完全不用 ultrasonic”
- 当前要明确区分：
  - 前方 ultrasonic：仍有效
  - 侧边 ultrasonic：已废弃

## 当前主要入口

### 规划与记忆

- [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md)
- [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md)

### Arduino 侧

- [firmware/arduino/required_function_c1/final_code/final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
  - Required Function 集成参考代码
  - 保留有效的前方 ultrasonic 靠站逻辑

- [firmware/arduino/required_function_c1/straight_line_power_test/straight_line_power_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/straight_line_power_test/straight_line_power_test.ino)
  - 当前直行调参草图
  - 使用 `gyro heading hold + 四轮 PWM`

- [firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)
  - proposal 阶段 Jetson 串口桥接草图
  - 保留前方 ultrasonic 遥测
  - 手动控制改成 `gyro + per-wheel trim`

- [firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino)
  - 电机自动 / 半自动校准草图
  - 前进 / 后退可做 gyro 自动初值搜索
  - 横移只做半自动初值

### Jetson Nano 侧

- [software/jetson_dashboard/server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
- [software/jetson_dashboard/dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
- [software/jetson_dashboard/unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)

当前 Jetson 侧已具备：

- 串口读取 Arduino JSON telemetry
- 浏览器 dashboard
- fixed-password unlock 页面
- 向 Arduino 回发命令
- `--mock` 模式

## 历史 / 废弃文件说明

下面这些文件或目录保留为历史参考，但不应再当作当前硬件方案继续使用：

- [firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino)
- [firmware/arduino/required_function_c1/dual_ultrasonic_oled_test/dual_ultrasonic_oled_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/dual_ultrasonic_oled_test/dual_ultrasonic_oled_test.ino)
- [firmware/arduino/proposal_amr/obstacle_avoidance_test/obstacle_avoidance_test.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/obstacle_avoidance_test/obstacle_avoidance_test.ino)

这些文件共同特点是：

- 它们依赖已经移除的侧边 ultrasonic
- 现在只应作为旧实验记录保留

## 目录结构

```text
.
|-- README.md
|-- plan.md
|-- agent.md
|-- firmware/
|   `-- arduino/
|       |-- proposal_amr/
|       |-- required_function_c1/
|       `-- singleServo/
|-- hardware/
|   `-- amr_pin_mapping_template.xlsx
|-- software/
|   |-- jetson_dashboard/
|   `-- jetson_yolo_tests/
|-- Test_Code_24A/
`-- reflection/
```

## 备注

- `required_function_c1/` 是已完成阶段的参考基线
- `proposal_amr/` 是当前 Arduino 主实现区
- `software/jetson_dashboard/` 是当前 Jetson Nano 主实现区
- 如果硬件假设再次变化，至少要同步更新：
  - `hardware/amr_pin_mapping_template.xlsx`
  - 对应 Arduino 草图
  - `plan.md`
  - `agent.md`

## New Proposal Demo

Current interrupt / resume integration target:

- [firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/interrupt_resume_demo.ino)
- [firmware/arduino/proposal_amr/interrupt_resume_demo/README.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/interrupt_resume_demo/README.md)

This demo is a proposal-stage auto-flow prototype used to test:

- UI manual takeover
- task freeze during manual override
- stage-level resume after manual control
- Jetson <-> Arduino interrupt / resume serial protocol

It is not the final proposal mission state machine.
