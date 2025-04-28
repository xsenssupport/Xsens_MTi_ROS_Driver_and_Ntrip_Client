
# Xsens MTi ROS2 Driver and Ntrip Client

[中文 (Chinese)](README.zh.md)

This code was based on the official Xsens 2025.0 [Open source Xsens Device API](https://base.movella.com/s/article/Introduction-to-the-MT-SDK-programming-examples-for-MTi-devices) tested on MTi-680G/MTi-8/MTi-630/MTi-300 with ROS2 Humble at Ubuntu 22.04.3 LTS .

## ROS vs ROS2 Versions

Note that this branch contains the `ROS2` implementation that is compatible for `Foxy`, `Humble` and `Jazzy`. 

If you are looking for the `ROS1` version, you should go to the [`main`](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/main) branch.

## How to clone this ROS2 branch

```
git clone --branch ros2 https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git
```

## Device Settings - Output Configurations
#### Note: the UTC Time, SampleTimeFine, Status Word, Latitude and Longitude needs to be enabled, in order to get GPGGA data for topic ``/nmea``: MT Manager - Device Settings - Output Configuration , select "UTC Time, Sample TimeFine, Status Word, Latitude and Longitude" and other required data, click "Apply"

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
The Ntrip_client subscribes to the ``/nmea`` rostopic from ``xsens_mti_ros2_driver``, and wait until it gets data for maximum 300 sec, it will send GPGGA to the Ntrip Caster(Server) every 1 second.

User needs to change the ``ntrip_launch.py`` for their own credentials/servers/mountpoint. 

## How to Install:
install dependency:
```
sudo apt install ros-[ROSDISTRIBUTION]-nmea-msgs
sudo apt install ros-[ROSDISTRIBUTION]-mavros-msgs
```
for example for ROS2 Humble:
```
sudo apt install ros-humble-nmea-msgs
sudo apt install ros-humble-mavros-msgs
```

change the NTRIP credentials/servers/mountpoint in ``src/ntrip/launch/ntrip_launch.py`` to your own one.


copy the `src` folder to your ``ros2_ws`` folder, and run the code below:
```
cd ~/ros2_ws
colcon build
```

Source the ``install/setup.bash`` file inside your ROS2 workspace
```
source install/setup.bash
```
or 

add it into rules:
```
sudo nano ~/.bashrc
```
At the end of the file, add the following line:
```
source /home/[USER_NAME]/ros2_ws/install/setup.bash
```
save the file, exit.

Note: If you don't add this source line to your `~/.bashrc`, then every time you open a new terminal, you will have to firstly do `source install/setup.bash`, otherwise you couldn't read the `/status` topic data.

## How to Use:
open first terminal:
```
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```
or with the 3D display rviz:
```
ros2 launch xsens_mti_ros2_driver display.launch.py
```
and then open another terminal
```
ros2 launch ntrip ntrip_launch.py
```

## How to confirm your RTK Status

you could check ``ros2 topic echo /rtcm``, there should be HEX RTCM data coming,

or ``ros2 topic echo /status`` to check the RTK Fix type, it should be 1(RTK Floating) or 2(RTK Fix).


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

- Refer to the [README.txt](./src/xsens_mti_ros2_driver/README.txt)
- nVidia Jetson devices, ref to [Interfacing MTi devices with the NVIDIA Jetson](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176) 
- Docs, ref code: [All MTi Related Documentation Links](https://base.movella.com/s/article/All-MTi-Related-Documentation-Links)
- Regarding Parameters: If you have previously run `colcon build` and then changed the credentials in ntrip or YAML parameters, you will need to run `colcon build` again after making those changes. Otherwise, it won't work for ROS 2.
- For technical support, and if it is no relevant to the code here, please send your questions to support@movella.com


If the program displays the message `No MTi device found`:

- For the MTi-1/600/Sirius product series, where the FTDI chip was used, try the following steps:
  ```bash
  sudo /sbin/modprobe ftdi_sio
  echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id
  ```
  Then, ensure you are in the `dialout` group:
  ```bash
  ls -l /dev/ttyUSB0
  groups
  ```
  If not, add yourself to the `dialout` group:
  ```bash
  sudo usermod -G dialout -a $USER
  ```
  Finally, reboot your computer:
  ```bash
  sudo reboot
  ```

- For MTi-100/300/G-710 devices:
  ```bash
  git clone https://github.com/xsens/xsens_mt.git
  cd xsens_mt
  sudo make HAVE_LIBUSB=1
  sudo modprobe usbserial
  sudo insmod ./xsens_mt.ko
  ```

- You can specify your own port and baud rate in the [`xsens_mti_node.yaml`](./src/xsens_mti_ros2_driver/param/xsens_mti_node.yaml) file:
  ```cpp
  // change the scan_for_devices to `false` and uncomment/change the port name and baud rate to your own values (by default it is 115200, unless you have changed the value with MT Manager).
  scan_for_devices: false
  port: '/dev/ttyUSB0'
  baudrate: 115200
  ```

- If either of the above methods works, try using `cutecom` to see if you can receive `FA FF 36` hex messages, if not, then you could contact [support@movella.com](mailto:support@movella.com):
  ```bash
  sudo apt install cutecom
  cutecom
  ```


