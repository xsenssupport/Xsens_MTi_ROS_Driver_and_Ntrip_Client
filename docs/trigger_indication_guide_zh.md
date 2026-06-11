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
