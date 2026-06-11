# TriggerIndication User Guide

> Last updated: 2026-06-11

> **Note**
>
> This guide explains how to configure and use TriggerIndication to obtain high-precision timestamps of trigger events.
>
> Reference article: <https://base.xsens.com/s/article/article/Synchronization-with-the-MTi>
>
> TriggerIndication is supported on the Xsens Sirius/Avior/MTi-600 series.

This guide describes how to configure an Xsens MTi sensor to output TriggerIndication (hardware SyncIn trigger indication) via MT Manager, and how to publish and view the trigger timestamp in ROS1 through `xsens_ros_mti_driver`.

---

## 1. Configure the Sensor with MT Manager

### 1.1 Enter configuration mode

```text
TX: GotoConfig
FA FF 30 00 D1

RX: GotoConfigAck
FA FF 31 00 D0
```

### 1.2 Set the sensor baudrate to 921600

```text
TX: SetPortConfig - append 4 bytes per protocol
FA FF 8C 0C 00 01 00 80 00 01 00 80 00 00 00 00 67

RX: SetPortConfigAck
FA FF 8D 00 74
```

### 1.3 Configure the output (including TriggerIndication 1)

```text
TX: SetOutputConfiguration
FA FF C0 18 10 60 01 90 40 20 01 90 80 20 01 90 20 30 01 90 E0 20 01 90 48 10 FF FF 3E

RX: SetOutputConfigurationAck
FA FF C1 18 10 60 FF FF 40 20 01 90 80 20 01 90 20 30 01 90 E0 20 FF FF 48 10 FF FF 63
```

![MT Manager Device Data View output configuration](./images/mt_manager_device_data_view_output_config.png)

*Figure: Device Data View output configuration.*

### 1.4 Enter measurement mode

```text
TX: GotoMeasurement
FA FF 10 00 F1

RX: GotoMeasurementAck
FA FF 11 00 F0
```

---

## 2. Trigger Test

After applying a hardware trigger on the SyncIn line, the corresponding TriggerIndication packets appear in MT Manager.

![TriggerIndication packets in MT Manager Device Data View](./images/mt_manager_device_data_view_trigger_indication.png)

*Figure: MT Manager shows that packets 30158, 30157, etc. each emitted a TriggerIndication packet.*

---

## 3. Using It in ROS1 (Ubuntu 20.04)

### 3.1 Configure the yaml file

Edit `src/xsens_ros_mti_driver/param/xsens_mti_node.yaml`:

```yaml
pub_triggerin_time: true  # Publish imu/triggerin_time (sensor_msgs/TimeReference) on a hardware SyncIn trigger. Requires SyncIn configured on the device.
triggerin_lines: [1]      # Which SyncIn lines to enable trigger indication for: 1=TriggerIn1, 2=TriggerIn2
```

![TriggerIndication configuration in xsens_mti_node.yaml](./images/xsens_mti_node_yaml_trigger_indication_config.png)

*Figure: TriggerIndication configuration in `xsens_mti_node.yaml`.*

### 3.2 View the trigger time topic

```bash
rostopic echo /imu/triggerin_time
```

![/imu/triggerin_time topic output on trigger](./images/triggerin_time_rostopic_at_trigger.png)

*Figure: `/imu/triggerin_time` topic output on trigger.*

### 3.3 View the SyncIn flag in status

```bash
rostopic echo --filter "m.syncin_marker" /status
```

![syncin_marker in /status on trigger](./images/status_at_syncin.png)

*Figure: the `syncin_marker` flag in `/status` on trigger.*
