# Xsens MTi ROS 驱动和 Ntrip 客户端

[English Version (英文版)](README.md)

此代码基于官方的 ``xsens_ros_mti_driver``，并在 MTi-3/7/8/MTi-630R/MTi-670G/MTi-680G 上，运行于 Ubuntu 20.04 LTS 和 ROS Noetic 下进行了测试。

## ROS 和 ROS2 版本

请注意，此分支包含 ROS1 实现。如果您需要 ROS2 版本，请访问 [`ros2`](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2) 分支。

## 设备设置 - 输出配置
### 注意:
- 对于 MTi-680(G)或者MTi-8，需要启用 UTC 时间和 PvtData，以便从 ``/nmea`` 话题获取 GPGGA 数据，该数据将用于 Ntrip 客户端：
    - 在 MT Manager 的设备设置中，配置输出，选择 “UTC Time, Sample TimeFine, Status Word, Latitude and Longitude” 以及其他必要数据，并点击 “Apply”；
    - 或者将 [xsens_mti_node.yaml](./src/xsens_ros_mti_driver/param/xsens_mti_node.yaml) 中的 ``enable_deviceConfig`` 设置为 `true`，并将 ``pub_utctime`` 和 ``pub_gnss`` 设置为 `true`，然后根据需要调整其他输出参数，以完成传感器配置。

以下是推荐的输出配置和设备设置：

![配置截图](MTi-680_Output_Configuration.png)

![设置截图](MTi-680_Device_Settings.png)

- 对于 MTi-1/2/3/7/610/620/630/670/100/200/300/G-710/320/等IMU/VRU/AHRS, GNSS/INS非RTK设备，可以删除或者忽略`ntrip`包，仅使用 [xsens_ros_mti_driver](./src/xsens_ros_mti_driver)包，建议使用MT Manager，配置勾选SampleTimeFine, UtcTime, Orientation(Quaternion), Acceleration, Rate Of Turn, Magnetic Field, Status Word等输出选项，如果选择400Hz数据，波特率配置为921600

## 对 MTi ROS 驱动的修改

- 修复了 ``/nmea`` GPGGA 话题的 fix_type 以符合 NMEA 标准。
- 添加了以下功能：
    - 传感器输出配置；
    - 传感器滤波设置；
    - 设置波特率；
    - 设置 MTi-8/MTi-680(G) 的 GNSS 杆臂；
    - 设置 u-Blox GNSS 平台；
    - 配置选项（如 AHS、In-Run Compass、北斗、OrientationSmoother、PositionVelocitySmoother、ContinuousZRU）；
    - 周期性地估计陀螺仪偏差(Manual Gyro Bias Estimation)；
    - 添加 ``filter/euler`` 和HR高速率话题，如 ``imu/acceleration_hr``、``imu/angular_velocity_hr``；
    - 添加报错信息。

- 修改了以下代码：
    - ``lib/xspublic/xscontroller/iointerface.h``，第 138 行，将 `PO_XsensDefaults` 改为 ``PO_OneStopBit``；
    - ``lib/xspublic/xscommon/threading.cpp``，更新以兼容 glibc 2.35。

## Ntrip 客户端

Ntrip 客户端订阅 ``xsens_ros_mti_driver`` 的 ``/nmea`` 话题，并等待获取 GPGGA 数据（最长 300 秒）。随后，每 1 秒（由 [ntrip.launch](./src/ntrip/launch/ntrip.launch) 定义）向 Ntrip Caster (服务器) 发送 GPGGA 数据。

用户需要在 ``ntrip.launch`` 中修改自己的凭据/服务器/挂载点。

## 安装方法

安装依赖项：
```
sudo apt install ros-[ROSDISTRIBUTION]-nmea-msgs
sudo apt install ros-[ROSDISTRIBUTION]-mavros-msgs
```

以 ROS Noetic 为例（使用 ``rosversion -d`` 获取您的版本）：
```
sudo apt install ros-noetic-nmea-msgs
sudo apt install ros-noetic-mavros-msgs
```

克隆源文件到您的 ``catkin_ws``，并运行以下代码：
```
cd ~/catkin_ws
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin_make
```

在 catkin 工作区中加载 ``/devel/setup.bash`` 文件：
```
source ./devel/setup.bash
```
或将其添加到规则中：
```
sudo nano ~/.bashrc
```

在文件末尾添加以下行：
```
source /[PATH_TO_Your_catkin_ws]/devel/setup.bash
```

保存并退出。  
注意：如果未将加载命令添加到 ``~/.bashrc``，每次打开新终端时都需要手动运行 `source ./devel/setup.bash`，否则无法读取 `/status` 话题数据。

## 使用方法

修改 ``src/ntrip/launch/ntrip.launch`` 中的凭据/服务器/挂载点为您自己的信息。

打开两个终端运行以下命令：
```
roslaunch xsens_mti_driver xsens_mti_node.launch
```
或者使用 3D 显示 rviz：
```
roslaunch xsens_mti_driver display.launch
```

对于MTi-8/MTi-680(G)， 然后运行Ntrip：
```
roslaunch ntrip ntrip.launch
```

## 如何确认 RTK 状态

可以通过以下方法检查 RTK 状态：
- 执行 ``rostopic echo /rtcm``，应显示 HEX RTCM 数据；
- 或执行 ``rostopic echo /status``，检查 RTK Fix 类型，应为 `1(RTK Floating)` 或 `2(RTK Fix)`。

## ROS 话题

| 话题                      | 消息类型                        | 消息内容                                                                                                                                      | 数据输出频率<br>（取决于 MT Manager 中的模型和输出配置）                         |
| ------------------------ | ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| filter/free_acceleration | geometry_msgs/Vector3Stamped    | free acceleration from filter, which is the acceleration in the local earth coordinate system (L) from which<br>the local gravity is deducted | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| filter/positionlla       | geometry_msgs/Vector3Stamped    | filtered position output in latitude (x), longitude (y) and altitude (z) as Vector3, in WGS84 datum                                           | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| filter/quaternion        | geometry_msgs/QuaternionStamped | quaternion from filter                                                                                                                        | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| filter/euler        | geometry_msgs/Vector3Stamped | euler(roll,pitch,yaw) from filter                                                                                                                        | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| filter/twist             | geometry_msgs/TwistStamped      | filtered velocity and calibrated angular velocity                                                                                                                 | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| filter/velocity          | geometry_msgs/Vector3Stamped    | filtered velocity output as Vector3                                                                                                           | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| gnss                     | sensor_msgs/NavSatFix           | raw 4 Hz latitude, longitude, altitude and status data from GNSS receiver                                                                     | 4Hz                                                                             |
| gnss_pose                | geometry_msgs/PoseStamped       | filtered position output in latitude (x), longitude (y) and altitude (z) as Vector3 in WGS84 datum, and quaternion from filter                | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/acceleration         | geometry_msgs/Vector3Stamped    | calibrated acceleration                                                                                                                       | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/angular_velocity     | geometry_msgs/Vector3Stamped    | calibrated angular velocity                                                                                                                   | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/data                 | sensor_msgs/Imu                 | quaternion, calibrated angular velocity and acceleration                                                                                      | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/dq                   | geometry_msgs/QuaternionStamped | integrated angular velocity from sensor (in quaternion representation)                                                                        | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/dv                   | geometry_msgs/Vector3Stamped    | integrated acceleration from sensor                                                                                                           | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/mag                  | sensor_msgs/MagneticField    | calibrated magnetic field                                                                                                                     | 1-100Hz                                                                         |
| imu/time_ref             | sensor_msgs/TimeReference       | SampleTimeFine timestamp from device                                                                                                          | depending on packet                                                             |
| imu/utctime              | sensor_msgs/TimeReference       | UTC Time from the device                                                                                                                      | depending on packet                                                             |
| nmea                     | nmea_msgs/Sentence              | 4Hz GPGGA data from GNSS receiver PVTData(if available) and StatusWord                             | 4Hz                                                                             |
| pressure                 | sensor_msgs/FluidPressure       | barometric pressure from device                                                                                                               | 1-100Hz                                                                         |
| status                   | xsens_mti_driver/XsStatusWord | statusWord, 32bit                                                                                                                             | depending on packet                                                             |
| temperature              | sensor_msgs/Temperature         | temperature from device                                                                                                                       | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| tf                       | geometry_msgs/TransformStamped  | transformed orientation                                                                                                                       | 1-400Hz(MTi-600 and MTi-100 series), 1-100Hz(MTi-1 series)                      |
| imu/acceleration_hr         | geometry_msgs/Vector3Stamped    | high rate acceleration                                                                                                                       | see xsens_mti_node.yaml                      |
| imu/angular_velocity_hr     | geometry_msgs/Vector3Stamped    | high rate angular velocity                                                                                                                   | see xsens_mti_node.yaml                      |

请参考 [MTi 系列参考手册](https://mtidocs.movella.com/mti-system-overview) 获取详细数据定义。

## 故障排查

- 设备连不上问题， 请参考 [README.txt](./src/xsens_ros_mti_driver/README.txt)。
- 如果使用英伟达Jetson设备，请参考 [与 NVIDIA Jetson 的接口](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176)。
- 文档与代码链接：[所有 MTi 相关文档链接](https://base.movella.com/s/article/All-MTi-Related-Documentation-Links)。
- 如果问题与代码无关，请发送技术支持请求至 support@movella.com。
