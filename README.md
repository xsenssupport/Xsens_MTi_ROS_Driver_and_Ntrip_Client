
# Xsens MTi ROS Driver and Ntrip Client

This code was based on the official ``xsens_ros_mti_driver`` and tested on MTi-3/7/8/MTi-630R/MTi-670G/MTi-680G with ubuntu 20.04 LTS, ROS noetic.

## ROS vs ROS2 Versions

Note that this branch contains the ROS1 implementation for the packages. If you are looking for the ROS2 version, you should go to the [`ros2`](https://github.com/jiminghe/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2) branch

## Device Settings - Output Configurations
### Note: 
- For MTi-680(G), the UTC Time, PvtData needs to be enabled, in order to get GPGGA data for topic ``/nmea``, which will be used for the Ntrip Client: 
    - MT Manager - Device Settings - Output Configuration , select "UTC Time, Sample TimeFine, Status Word, Latitude and Longitude" and other required data, click "Apply", 
- or your could change the ``enable_deviceConfig`` in [xsens_mti_node.yaml](./src/xsens_ros_mti_driver/param/xsens_mti_node.yaml) to true and change the ``pub_utctime``, ``pub_gnss`` to true, then change the other desired output parameters as listed in the [xsens_mti_node.yaml](./src/xsens_ros_mti_driver/param/xsens_mti_node.yaml) for the complete sensor configurations.

Here are the recommended Output Configurations and Device Settings:

![Alt text](MTi-680_Output_Configuration.png)

![Alt text](MTi-680_Device_Settings.png)

## Changes made to the MTi ROS Driver:

 - Fix the fix_type of the ``/nmea`` GPGGA topic to align with NMEA standards.
 - Add: 
    - +Sensor ouput configurations; 
    - +Sensor Filter Settings;
    - +Setting baudrate; 
    - +Setting GNSS Lever Arm for MTi-8/MTi-680(G)
    - +Setting u-Blox GNSS Platform
    - +Option Flags Settings(AHS,In-Run Compass, Beidou, OrientationSmoother, PositionVelocitySmoother, ContinousZRU); 
    - +Manual Gyro Bias Estimation Periodically
    - +Add ``filter/euler`` and high rate topics for ``imu/acceleration_hr``, ``imu/angular_velocity_hr``
    - +Add error messages.

- change:
    - ``lib/xspublic/xscontroller/iointerface.h``, line 138, change to ``PO_OneStopBIt`` for PO_XsensDefaults.
    - ``lib/xspublic/xscommon/threading.cpp``, updated to work with glibc 2.35.

## Ntrip_Client
The Ntrip_client subscribes to the ``/nmea`` rostopic from ``xsens_ros_mti_driver``, and wait until it gets GPGGA data from that rostopic for maximum 300 sec, it will send GPGGA to the Ntrip Caster(Server) every 1 seconds(defined by [ntrip.launch](./src/ntrip/launch/ntrip.launch)).

User needs to change the ``ntrip.launch`` for their own credentials/servers/mountpoint. 

## How to Install:
install dependency:
```
sudo apt install ros-[ROSDISTRIBUTION]-nmea-msgs
sudo apt install ros-[ROSDISTRIBUTION]-mavros-msgs
```
for example for ROS Noetic(use ``rosversion -d`` to get your version):
```
sudo apt install ros-noetic-nmea-msgs
sudo apt install ros-noetic-mavros-msgs
```

clone the source file to your ``catkin_ws``, and run the code below:
```
cd ~/catkin_ws
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin_make
```
Source the ``/devel/setup.bash`` file inside your catkin workspace
```
source ./devel/setup.bash
```
or 

add it into rules:
```
sudo nano ~/.bashrc
```
At the end of the file, add the following line:
```
source /[PATH_TO_Your_catkin_ws]/devel/setup.bash
```
save the file, exit.

## How to Use:
change the credentials/servers/mountpoint in ``src/ntrip/launch/ntrip.launch`` to your own one.


open two terminals:
```
roslaunch xsens_mti_driver xsens_mti_node.launch
```
or with the 3D display rviz:
```
roslaunch xsens_mti_driver display.launch
```
and then
```
roslaunch ntrip ntrip.launch
```

## How to confirm your RTK Status

you could check ``rostopic echo /rtcm``, there should be HEX RTCM data coming,

or ``rostopic echo /status`` to check the RTK Fix type, it should be 1(RTK Floating) or 2(RTK Fix).


## ROS Topics

| topic                    | Message Type                    | Message Contents                                                                                                                              | Data Output Rate<br>(Depending on Model and OutputConfigurations at MT Manager) |
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

Please refer to [MTi Family Reference Manual](https://mtidocs.movella.com/mti-system-overview) for detailed definition of data. 


## Troubleshooting

- Refer to the [README.txt](./src/xsens_ros_mti_driver/README.txt)
- nVidia Jetson devices, ref to [Interfacing MTi devices with the NVIDIA Jetson](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176) 
- Docs, ref code: [All MTi Related Documentation Links](https://base.movella.com/s/article/All-MTi-Related-Documentation-Links)
