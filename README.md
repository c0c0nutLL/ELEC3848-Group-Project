# ELEC3848 Group Project

本仓库用于 ELEC3848 Integrated Design Project 的代码、硬件记录和 Jetson Nano 上位机部分开发。

当前项目主线已经从 `Required Function` 阶段转到 proposal 阶段。  
后续总体计划以根目录的 [plan.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/plan.md) 为准，线程交接记忆以 [agent.md](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/agent.md) 为准。

## Current Status

### Required Function baseline

`Required Function` 部分已经完成，当前作为后续 proposal 实现的技术基线。  
这一阶段主要完成了：

- 小车自动靠墙停车
- 使用超声波和陀螺仪进行对齐修正
- OLED 和串口状态输出
- 多个传感器与底盘测试程序

主要代码位于：

- [firmware/arduino/required_function_c1](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1)

其中集成版本在：

- [final_code.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code/final_code.ino)
- [final_code_optimized.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/required_function_c1/final_code_optimized/final_code_optimized.ino)

### Proposal-stage communication baseline

Proposal 阶段已经搭起了 Arduino 和 Jetson Nano 之间的基础通信链路：

- Arduino 通过 USB 串口发送 JSON telemetry
- Jetson Nano 读取串口数据并提供网页 dashboard
- Jetson Nano 提供固定密码 unlock 页面
- Jetson Nano 可以通过串口向 Arduino 发送控制命令

主要文件位于：

- [firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/telemetry_serial_demo/telemetry_serial_demo.ino)
- [software/jetson_dashboard/server.py](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/server.py)
- [software/jetson_dashboard/dashboard.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/dashboard.html)
- [software/jetson_dashboard/unlock.html](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/software/jetson_dashboard/unlock.html)

### Hardware records

当前硬件接线和 pin 记录主要放在：

- [hardware/amr_pin_mapping_template.xlsx](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/hardware/amr_pin_mapping_template.xlsx)

机械臂舵机的 sample code 参考：

- [firmware/arduino/singleServo/singleServo.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/singleServo/singleServo.ino)

## Repository Structure

当前主要结构如下：

```text
.
|-- README.md
|-- agent.md
|-- plan.md
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

## Notes

- `required_function_c1/` 是已完成阶段的参考基线
- `proposal_amr/` 是后续 Arduino 主实现区
- `software/jetson_dashboard/` 是当前 Jetson Nano 上位机主实现区
- 较老的测试和教学参考代码放在 `Test_Code_24A/`
