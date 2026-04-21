# Motor Calibration Assistant

## 1. 这个程序做什么

[motor_calibration_assistant.ino](/F:/李仁杰/香港大学/2025-2026/ELEC3848/Group%20project/firmware/arduino/proposal_amr/motor_calibration_assistant/motor_calibration_assistant.ino) 是当前仓库里用于“电机自动校准 / 半自动校准”的 Arduino 草图。

它面向 2026-04-21 之后的最新硬件决策：

- 侧边两个 ultrasonic 已移除
- 直行不再依赖 corridor guidance
- 校准时只使用 `MPU6050 gyroscope`
- 四个 motor 各自维护独立 PWM 参数

这个程序不会假装在“没有 encoder、没有侧向位移传感器”的条件下，能一次性自动算出所有运动模式的最优 PWM。

它实际提供的是：

- `forward / backward / strafe left / strafe right`
- `low / medium / high`
- `4 个 motor` 各自的 PWM 参数
- EEPROM 保存 / 读取 / 校验 / 恢复默认值
- OLED 非阻塞显示
- 串口命令与按钮交互
- 对 `forward / backward` 的 gyro 自动搜索
- 对 `strafe left / strafe right` 的半自动初值搜索

## 2. 自动与半自动的技术判断

### 可做“较可信自动初值”的模式

- `straight forward`
- `straight backward`

原因：

- 这两种模式的主要目标是“尽量不偏航”
- `MPU6050` 可以直接给出航向漂移
- 通过四轮 PWM 分配与小幅 heading hold，可把候选参数往“更不偏航”的方向搜索

限制：

- 没有 encoder，就没有单轮速度闭环
- 也没有真正的位移闭环
- 所以结果仍然是“当前底盘与当前场地条件下更稳的 PWM 初值”，不是严格速度标定

### 只能做“半自动初值”的模式

- `strafe left`
- `strafe right`

原因：

- 现在没有侧边 ultrasonic，也没有 encoder
- 程序只能根据 `gyro` 压低横移时的偏航
- 但无法自动判断横移时是否发生明显前后串动
- 也无法判断横移距离是否准确

因此：

- `SL / SR` 的 `AUTO` 仍会执行搜索
- 但它只是在做“heading-only initial value”
- 结果必须人工复测确认

### 是否需要 encoder

需要。

如果后续要更严谨地完成：

- 四轮速度一致性标定
- 定长前进 / 后退标定
- 定量横移标定
- 更可靠的运动模型

那么 encoder 基本是必须的。

## 3. 参数结构

程序的核心参数结构如下：

- `MotionMode`
  - `MODE_FORWARD`
  - `MODE_BACKWARD`
  - `MODE_STRAFE_LEFT`
  - `MODE_STRAFE_RIGHT`
- `SpeedTier`
  - `TIER_LOW`
  - `TIER_MEDIUM`
  - `TIER_HIGH`
- `CalibrationProfile`
  - `pwm[A]`
  - `pwm[B]`
  - `pwm[C]`
  - `pwm[D]`

也就是：

`4 motion modes x 3 speed tiers x 4 motor PWM`

默认值位于：

- `DEFAULT_PROFILE_TABLE[MODE_COUNT][TIER_COUNT]`

运行时使用的参数位于：

- `profiles[MODE_COUNT][TIER_COUNT]`

## 4. EEPROM 保存方案

### 地址布局

程序从 EEPROM 地址 `0` 写入一个 `CalibrationBlob`：

| 地址范围 | 字段 | 说明 |
|---|---|---|
| `0x0000 - 0x0003` | `magic` | 固定值 `0x43414C32`，即 `CAL2` |
| `0x0004 - 0x0005` | `version` | 当前版本 `2` |
| `0x0006 - 0x0007` | `payloadSize` | `profiles` 区域大小 |
| `0x0008 - 0x0009` | `checksum` | 16-bit 校验值 |
| `0x000A - ...` | `profiles` | `4 x 3 x 4` 组 PWM 参数 |

### 支持的能力

- 默认值
- EEPROM 读取
- EEPROM 写入
- `magic / version / payloadSize / checksum` 校验
- EEPROM 无效时回退到默认值
- 串口命令恢复默认值

## 5. OLED 显示

因为当前 OLED 是 `128x32`，程序使用双页轮播，避免阻塞主循环。

### Page 1

显示：

- 当前 `mode`
- 当前 `speed tier`
- 当前阶段状态
- 四个 motor PWM
- 当前是否 dirty
- 自动搜索时的 pass / candidate 进度

### Page 2

显示：

- 当前 `mode`
- 当前 `speed tier`
- 当前阶段状态
- `heading error / final heading`
- `peak heading drift`
- `mean correction`
- `score`
- 当前模式是 `auto` 还是 `semi`

### 故障页

若当前有错误，优先显示：

- `FAIL`
- 错误原因
- 当前 `heading`
- 当前 `gyroZ`

## 6. 串口命令与按钮交互

建议串口监视器使用 `115200` 并打开换行结尾。

### 常用命令

- `HELP`
- `MODE FWD|BWD|SL|SR`
- `TIER LOW|MED|HIGH`
- `AUTO` / `CAL` / `START`
  - `FWD / BWD`：执行 gyro 自动搜索
  - `SL / SR`：执行半自动搜索，只给初值
- `TEST` / `RUN`
  - 用当前参数跑一轮，不自动搜索
- `STOP`
- `SAVE`
- `LOAD`
- `DEFAULTS`
- `PRINT`
- `PRINT ALL`
- `PRINT LAST`
- `SET A|B|C|D <PWM>`
- `BUMP A|B|C|D <DELTA>`
- `GYRO`

### 按钮交互

- `START_BUTTON_PIN = 33`
- 空闲时按下：开始当前 `MODE + TIER` 的 `AUTO`
- 运行时按下：停止当前测试

## 7. 建议操作流程

### 前进 / 后退

1. 上电后保持小车静止，等待 gyro bias 初始化完成。
2. `MODE FWD` 或 `MODE BWD`
3. `TIER LOW|MED|HIGH`
4. 输入 `AUTO`
5. 程序会围绕当前四轮 PWM 做多轮候选搜索
6. 查看 `PRINT LAST`
7. 再用 `TEST` 做人工复测
8. 满意后执行 `SAVE`

### 横移

1. `MODE SL` 或 `MODE SR`
2. `TIER LOW|MED|HIGH`
3. 输入 `AUTO`
4. 程序只会根据偏航大小搜索一个“更稳”的初值
5. 再执行 `TEST`
6. 人工观察是否仍然存在明显前后串动
7. 必要时用 `BUMP / SET` 微调
8. 满意后执行 `SAVE`

## 8. 哪些结果可信，哪些只能当初值

### 相对可信

- `forward + low/medium/high`
- `backward + low/medium/high`

这些结果适合作为：

- 直行默认 PWM
- proposal 主状态机里的基础运动参数
- gyro heading hold 的起始参数

### 只能作为初始值

- `strafe left + low/medium/high`
- `strafe right + low/medium/high`

这些结果只能用于：

- 给横移一个更稳定的起点
- 缩短人工试错时间
- 作为后续 encoder 闭环前的保守参数

## 9. 当前限制与后续改进

当前限制：

- 无 encoder
- 无侧向位移传感器
- 无法做真正的速度闭环
- 无法自动验证横移距离与串动

后续优先建议：

1. 增加 wheel encoder
2. 做单轮速度闭环
3. 做定长前进 / 后退标定
4. 做可量化的横移标定
5. 把这套参数真正接入 proposal 主状态机
