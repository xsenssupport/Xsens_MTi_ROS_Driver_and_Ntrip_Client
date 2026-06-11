# TriggerIndication 使用指南

> 更新日期：2026-06-11

> **说明**
>
> 本指南提供指导如何配置和使用 TriggerIndication 以便获得高精度触发时刻的时间戳。
>
> 参考文章：<https://base.xsens.com/s/article/article/Synchronization-with-the-MTi>
>
> Xsens Sirius/Avior/MTi-600 系列支持 TriggerIndication 功能。

本指南介绍如何通过 MT Manager 配置 Xsens MTi 传感器输出 TriggerIndication（硬件 SyncIn 触发指示），并在 ROS1 中通过 `xsens_ros_mti_driver` 发布并查看触发时间戳。

---

## 一、使用 MT Manager 配置传感器

### 1. 进入配置模式

```text
TX: GotoConfig
FA FF 30 00 D1

RX: GotoConfigAck
FA FF 31 00 D0
```

### 2. 配置传感器波特率为 921600

```text
TX: SetPortConfig - append 4 bytes per protocol
FA FF 8C 0C 00 01 00 80 00 01 00 80 00 00 00 00 67

RX: SetPortConfigAck
FA FF 8D 00 74
```

### 3. 配置输出（包含 TriggerIndication 1）

```text
TX: SetOutputConfiguration
FA FF C0 18 10 60 01 90 40 20 01 90 80 20 01 90 20 30 01 90 E0 20 01 90 48 10 FF FF 3E

RX: SetOutputConfigurationAck
FA FF C1 18 10 60 FF FF 40 20 01 90 80 20 01 90 20 30 01 90 E0 20 FF FF 48 10 FF FF 63
```

![MT Manager Device Data View 输出配置截图](./images/mt_manager_device_data_view_output_config.png)

*图：Device Data View 输出配置截图*

### 4. 进入测量模式

```text
TX: GotoMeasurement
FA FF 10 00 F1

RX: GotoMeasurementAck
FA FF 11 00 F0
```

---

## 二、触发测试

在 SyncIn 线上施加硬件触发后，MT Manager 中可以看到对应的 TriggerIndication 数据包。

![MT Manager Device Data View 中的 TriggerIndication 数据包](./images/mt_manager_device_data_view_trigger_indication.png)

*图：MT Manager 显示 30158、30157 等数据包均发出了 TriggerIndication 数据包。*

---

## 三、在 ROS1（Ubuntu 20.04）中使用

### 1. 配置 yaml 文件

编辑 `src/xsens_ros_mti_driver/param/xsens_mti_node.yaml`：

```yaml
pub_triggerin_time: true  # Publish imu/triggerin_time (sensor_msgs/TimeReference) on a hardware SyncIn trigger. Requires SyncIn configured on the device.
triggerin_lines: [1]      # Which SyncIn lines to enable trigger indication for: 1=TriggerIn1, 2=TriggerIn2
```

![xsens_mti_node.yaml 中的 TriggerIndication 配置](./images/xsens_mti_node_yaml_trigger_indication_config.png)

*图：`xsens_mti_node.yaml` 中的 TriggerIndication 配置。*

### 2. 查看触发时间话题

```bash
rostopic echo /imu/triggerin_time
```

![触发时 /imu/triggerin_time 话题输出](./images/triggerin_time_rostopic_at_trigger.png)

*图：触发时 `/imu/triggerin_time` 话题输出。*

### 3. 查看 status 中的 SyncIn 标志位

```bash
rostopic echo --filter "m.syncin_marker" /status
```

![触发时 /status 中的 syncin_marker](./images/status_at_syncin.png)

*图：触发时 `/status` 中的 `syncin_marker` 标志位。*

---

# Xsens MTi-630 触发指示（TriggerIndication）与时间同步说明

## 1. MTData2 数据包概述

Xsens MTi-630 通过 **MTData2** 数据包发送数据，数据包内容包括：

- 加速度（Acceleration）
- 角速度（Angular Velocity / RateOfTurn）
- 姿态（Orientation，欧拉角或四元数形式）
- 状态字（StatusWord）等

若额外配置 **TriggerIndication（触发指示）** 输出，则设备会单独输出一类触发数据包。

## 2. TriggerIndication 数据包

TriggerIndication 的含义是：当 **SyncIn Line1** 或 **SyncIn Line2** 上出现硬件触发信号时，设备将输出一个对应的数据包。示例数据如下：

```text
TriggerIn2
Line:        9
Polarity:    1
Timestamp:   290637743
FrameNumber: 0
```

## 3. 数据流示例

| 序号 | 时间 | 报文 | 负载（Payload） |
|------|--------------|--------------------|---------|
| 4122 | 10:02:21.017 | FA FF MTData2 `A1` | {(PacketCounter, 2 字节, 44417), (SampleTimeFine, 4 字节, 2906373), (UtcTime, 12 字节, (ns: 999292120, Year: 2022, Month: 2, Day: 11, Hour: … |
| 4121 | 10:02:21.017 | FA FF MTData2 `0B` | {(**TriggerIn2**, 8 字节, (Line: 9, Polarity: 1, Timestamp: 290637743, FrameNumber: 0)} |
| 4120 | 10:02:21.017 | FA FF MTData2 `A1` | {(PacketCounter, 2 字节, 44416), (SampleTimeFine, 4 字节, 2906348), (UtcTime, 12 字节, (ns: 996792120, Year: 2022, Month: 2, Day: 11, Hour: … |
| 4119 | 10:02:21.017 | FA FF MTData2 `A1` | {(PacketCounter, 2 字节, 44415), (SampleTimeFine, 4 字节, 2906323), (UtcTime, 12 字节, (ns: 994292120, Year: 2022, Month: 2, Day: 11, Hour: … |

由上表可知：

- **数据包 4119、4120、4122** 为常规的传感器 MTData2 数据包；
- **数据包 4121** 为专门的 TriggerIndication 数据包，表示在时间戳 **290637743** 这一时刻接收到了一次触发信号。

## 4. 时间戳解析

### 4.1 SampleTimeFine

常规 MTData2 数据包中包含 **SampleTimeFine** 字段，例如下面两个数据包：

- 数据包 1：SampleTimeFine = 2906348，即 290634.8 ms = 290.6348 s
- 数据包 2：SampleTimeFine = 2906373，即 290637.3 ms = 290.6373 s

两者差值为 25，对应 0.0025 s（即 2.5 ms），与 400 Hz 的数据发送频率相吻合。

### 4.2 触发时间戳

触发时间戳 **290637743** 的单位为微秒（μs），即：

> 290637743 μs = 290637.743 ms = 290.637743 s

## 5. 应用场景：Jetson Orin 与 MTi-630 的时间同步

假设当前系统配置如下：

- 一台 **NVIDIA Jetson Orin**，通过 **UART 串口**与 Xsens MTi-630 相连；
- Jetson 的一个 **GPIO 引脚**与 MTi-630 的 **SyncIn2** 引脚相连。

同步流程分为以下三步：

### 步骤 1：产生触发信号

Jetson 通过 GPIO 产生触发信号，并精确记录该触发信号对应的 UTC 时间（记为 `UtcTime_Trigger`）：

```text
UtcTime_Trigger = 1781158574.123456    （单位：秒，小数部分精确到微秒，即 6 位小数）
```

### 步骤 2：接收触发指示

运行 ROS 驱动的 Jetson 接收到 TriggerIndication 数据包，该数据包给出触发时间戳：**290637743 μs**。

### 步骤 3：计算数据包的 UTC 时间

由此可计算数据包 4122 对应的 UTC 时间：

```text
UtcTime_packet_4122 = UtcTime_Trigger + (2906373 / 10000 − 290637743 / 1000000)
```

## 6. 注意事项

**计数器回绕（Counter Rollover）：**

- 4 字节的触发时间戳每经过 2³² μs（约 4295 s）即发生一次回绕（溢出归零）；
- SampleTimeFine 同样为 4 字节计数器，也会在 2³² 处发生回绕。

在跨回绕边界进行时间差计算时，必须对回绕情况进行处理，否则将导致时间换算错误。
