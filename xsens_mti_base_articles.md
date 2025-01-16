# Xsens MTi Documentations

All Xsens MTi products manuals can be found here:
https://mtidocs.movella.com/home

The corresponding manual catalogs and links are listed below according to different models:

## Manuals for MTi-1 series (including MTi-1, MTi-2, MTi-3, MTi-7, MTi-8)

- [MTi Family Reference Manual](https://mtidocs.movella.com/mti-family-reference-manual)
- [MT Manager User Manual](https://mtidocs.movella.com/mt-manager)
- [Firmware Updater User Manual](https://mtidocs.movella.com/firmware-updater)
- [Magnetic Calibration Manual](https://mtidocs.movella.com/magnetic-calibration-manual)
- [MTi 1-series Datasheet](https://mtidocs.movella.com/datasheet)
- [MTi 1-series DK User Manual](https://mtidocs.movella.com/use-manual)
- [MTi 1-series Hardware Integration Manual](https://mtidocs.movella.com/integration-manual)
- [MT Low Level Communication Protocol Documentation](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## MTi-320 Manual
[MTi-320 Manual](https://mtidocs.movella.com/mti-320)

## Manuals for MTi-600 series 
(including MTi-610, MTi-620, MTi-630, MTi-670, MTi-680; MTi-610R, MTi-620R, MTi-630R; MTi-670G, MTi-680G)

- [MTi Family Reference Manual](https://mtidocs.movella.com/mti-family-reference-manual)
- [MT Manager User Manual](https://mtidocs.movella.com/mt-manager)
- [Firmware Updater User Manual](https://mtidocs.movella.com/firmware-updater)
- [Magnetic Calibration Manual](https://mtidocs.movella.com/magnetic-calibration-manual)
- [MTi 600-series Datasheet](https://mtidocs.movella.com/mti-600-series-datasheet)
- [MTi 600-series DK User Manual](https://mtidocs.movella.com/dk-user-manual-2)
- [MTi 600-series Hardware Integration Manual](https://mtidocs.movella.com/hardware-integration-manual-2)
- [MT CAN Protocol Documentation](https://mtidocs.movella.com/mt-can-documentation)
- [MT Low Level Communication Protocol Documentation](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## Manuals for MTi-100 series 
(including MTi-100, MTi-200, MTi-300, MTi-G-710; MTi-10, MTi-20, MTi-30)

- [MTi Family Reference Manual](https://mtidocs.movella.com/mti-family-reference-manual)
- [MT Manager User Manual](https://mtidocs.movella.com/mt-manager)
- [Firmware Updater User Manual](https://mtidocs.movella.com/firmware-updater)
- [Magnetic Calibration Manual](https://mtidocs.movella.com/magnetic-calibration-manual)
- [MTi 10/100-series](https://mtidocs.movella.com/mti-10-100-series-user-manual)
- [MT Low Level Communication Protocol Documentation](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## Manuals for Xsens Sirius series 
(including Sirius IMU, VRU, AHRS models)

- [Family Reference Manual](https://mtidocs.movella.com/mti-family-reference-manual)
- [MT Manager User Manual](https://mtidocs.movella.com/mt-manager)
- [Firmware Updater User Manual](https://mtidocs.movella.com/firmware-updater)
- [Magnetic Calibration Manual](https://mtidocs.movella.com/magnetic-calibration-manual)
- [Xsens Sirius series Datasheet](https://mtidocs.movella.com/xsens-sirius-series-datasheet)
- [Xsens Sirius series DK User Manual](https://mtidocs.movella.com/xsens-sirius-dk-user-manual)
- [Xsens Sirius series Hardware Integration Manual](https://mtidocs.movella.com/xsens-sirius-series-hardware-integration-manual)
- [MT Low Level Communication Protocol Documentation](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## Manuals for Xsens Vision Navigator

- [Quick Start Guide](https://mtidocs.movella.com/vn-quick-start-guide)
- [Datasheet](https://mtidocs.movella.com/vn-datasheet)
- [Hardware Integration Manual](https://mtidocs.movella.com/vn-integration-manual)

## Software Installation

### 1) MT Software Suite
(including two softwares: MT Manager and Magnetic Field Mapper)
Please download under the MTi Products section: https://www.movella.com/support/software-documentation

### Linux MT Manager
*Not working for Nvidia Jetson or Rasperry-Pi*

```bash
sudo tar -xf MT_Software_Suite_linux-x64_2022.0_b7085_r119802.tar.gz
sudo chmod -R o+rw MT_Software_Suite_linux-x64_2022.0/
cd MT_Software_Suite_linux-x64_2022.0
sudo tar -xf mtmanager_linux-x64_2022.0.tar.gz
sudo chmod -R o+rw mtmanager/
```

Then refer to the readme in the doc(located at mtmanager/linux-64/doc/MTM.README) to install dependancies for MT Manager.

Ref: [MT Manager Installation Guide for ubuntu 20.04 and 22.04](https://base.movella.com/s/article/MT-Manager-Installation-Guide-for-ubuntu-20-04-and-22-04)

### 2) Firmware Updater
This software is only for firmware update, check the Firmware Release Notes to see if you need to update your firmware.
- Software Download: https://www.movella.com/hubfs/FirmwareUpdater.zip
- For MTi-680G, please make sure you have updated to the latest firmware 1.12.0

## Example Code

### Windows Example Code:
```
C:\Program Files\Xsens\MT Software Suite 2022.0\MT SDK\Examples
```

### MT SDK Documentation:
```
C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK
```

- Xsensdeviceapi:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xsensdeviceapi\doc\html\index.html`
- Xscontroller:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xscontroller\doc\html\index.html`
- Xstypes:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xstypes\doc\html\index.html`

### Linux Example Code
After downloading MT Software Suite, extract to your own choice of installation directory, default is `/usr/local/xsens`

```bash
sudo ./mtsdk_linux-x64_2022.0.sh
```

Example code by default is located in: `/usr/local/xsens/examples/mtsdk`
SDK Documentation by default is located in: `/usr/local/xsens/doc`, same as Windows, there will be xscontroller, xsensdeviceapi, xstypes.

For ARM architecture(NVidia Jetson, RasperryPi):
- MT Manager, xda_cpp cannot be used
- xda_public_cpp can be used

xda_public_cpp, located default at `/usr/local/xsens/examples/mtsdk/xda_public_cpp`, there are two cpp files:
1. `example_mti_receive_data.cpp` is the sample code, which helps to: scan ports, connect, configure output of MTi, start measurement, record .mtb document
2. `example_mti_parse_logfile.cpp`, which can be used to convert Xsens log file .mtb documents to .txt

## ROS Driver

### ROS1
https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/main

### ROS2 (Humble and Jazzy Jalisco)
https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2

Note: the NTRIP Client is for RTK GNSS/INS versions of products like MTi-8, MTi-680, MTi-680G. If you have an IMU/VRU/AHRS unit, you could ignore the NTRIP Client and change pub_positionLLA/pub_nmea/pub_gnss/pub_gnsspose to false in xsens_mti_node.yaml.

## Video Tutorials

1. [Choose your MTi product](https://tutorial.movella.com/video/choose-your-mti-product)
2. [Getting Started with MTi](https://tutorial.movella.com/video/getting-started-with-mti)
3. [MT Manager 2019](https://tutorial.movella.com/video/mt-manager-2019)
4. [MTi-600 Sensor Fusion Filters(VRU, AHRS)](https://tutorial.movella.com/video/xsens-mti-ahrs)
5. [Magnetic Field Mapper(MFM) Magnetic Calibration](https://tutorial.movella.com/video/magnetic-calibration)
6. [Magnetic calibration in the car (2D magnetic calibration)](https://www.youtube.com/watch?v=3YJMUPFdYH8)
7. [Active Heading Stabilization](https://tutorial.movella.com/video/active-heading-stabilization-ahs)
8. [Manual Gyro Bias Estimation](https://tutorial.movella.com/video/manual-gyro-bias-estimation)
9. [Handling Magnetic Distortion in Robotic and Industrial Applications](https://tutorial.movella.com/video/handling-magnetic-distortion-in-robotic-and-industrial-applications)
10. [Device Data View](https://tutorial.movella.com/video/device-data-view)
11. [XDA Processing](https://tutorial.movella.com/video/xda-processing)
12. [Xsens Vision Navigator Introduction](https://www.youtube.com/watch?v=_7AzFv3s0DI)

## Xsens Base Knowledge Articles
https://base.movella.com/s/ism-landing-page

### Detailed Links

#### MT Software Suite / SDK
1. [Links to the documentation, leaflets and changelogs for the MTi series](https://base.movella.com/s/article/Online-links-to-manuals-from-the-MT-Software-Suite)
2. [Installing the correct Software Suite for different (generation) products](https://base.movella.com/s/article/Installing-the-correct-Software-Suite-for-different-generation-products)
3. [Quick start for MTi Development Kit](https://base.movella.com/s/article/Quick-start-for-MTi-Development-Kit)
4. [Where to find the serial number on the MTi](https://base.movella.com/s/article/Where-to-find-the-serial-number-on-the-MTi)
5. [Introduction to the MT SDK programming examples for MTi devices](https://base.movella.com/s/article/Introduction-to-the-MT-SDK-programming-examples-for-MTi-devices)
6. [Open source XDA (Xsens Device API)](https://base.movella.com/s/article/Open-source-XDA-Xsens-Device-API)
7. [Recording a MTB file with XDA Processing](https://base.movella.com/s/article/Recording-a-MTB-file-with-XDA-Processing)
8. [Selecting Filter Profile](https://base.movella.com/s/article/article/Selecting-Filter-Profile)
9. [MTi Filter Profiles](https://base.movella.com/s/article/article/MTi-Filter-Profiles-1605869708823)
10. [The difference between XDA Processing and Onboard Processing](https://base.movella.com/s/article/article/The-difference-between-XDA-Processing-and-Onboard-Processing)

#### 3rd Party Integrations
1. [Third Party Drivers for use with the MTi](https://base.movella.com/s/article/article/Third-Party-Drivers-for-use-with-the-MTi)
2. [Interfacing an MTi GNSS/INS device with a Velodyne Lidar](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-Velodyne-Lidar)
3. [Interfacing an MTi GNSS/INS device with an Ouster Lidar](https://base.movella.com/s/article/article/Interfacing-with-Ouster-Lidar-for-Xsens)
4. [Interfacing an MTi GNSS/INS device with a HESAI Lidar](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-HESAI-Lidar)
5. [Best practices I2C and SPI for MTi 1-series](https://base.movella.com/s/article/article/Best-practices-I2C-and-SPI-for-MTi-1-series-1605869706124)

## Third Party Reference Code

### UART/RS232
#### Python:
- https://github.com/jiminghe/Xsens_MTi_Serial_Reader

#### C++:
- https://github.com/jiminghe/Xsens_MTi_Serial_Reader_Linux
- https://github.com/ealkhatib/XsensUart

#### C Library(MTi-300 connected to a STM32F4)
- https://github.com/Scottapotamas/xsens-mti

### SPI:
- https://github.com/Steven-GH/Xsens_MTi_SPI

### I2C:
- Arduino MEGA R3 2560: https://github.com/jiminghe/Xsens_MTi_I2C_Arduino
- Arduino UNO: https://github.com/Steven-GH/Xsens_MTi_I2C
- Raspberry Pi 5: https://github.com/jiminghe/Xsens-MTi-1-Series-I2C-with-Raspberry-Pi-5
- STM32F401RE Nucleo Board: https://github.com/jiminghe/Xsens_MTi_I2C_STM32F4/

### Other Resources:
- ROS 1 Driver (CANBus): https://github.com/jiminghe/Xsens_MTi_CAN_ROS_Driver
- XVN Rosbag Parsing(python): https://github.com/jiminghe/XVN_Rosbag_Parser
- XDA PUBLIC CPP with CMakeList.txt: https://github.com/jiminghe/xda_public_cpp/tree/cmake

## Product Leaflets

### 1-series:
- [MTi-1](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-1.pdf)
- [MTi-2](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-2.pdf)
- [MTi-3](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-3.pdf)
- [MTi-7](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-7.pdf)
- [MTi-8](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-8.pdf)
- [MTi-320](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-320.pdf)

### 600-series:
- [MTi-610](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-610.pdf)
- [MTi-620](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-620.pdf)
- [MTi-630](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630.pdf)
- [MTi-630R](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630R.pdf)
- [MTi-670](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-670.pdf)
- [MTi-670G](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-670G.pdf)
- [MTi-680](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-680.pdf)
- [MTi-680G](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-680G.pdf)

### 100-series:
- [MTi-100](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-100.pdf)
- [MTi-200](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-200.pdf)
- [MTi-300](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-300.pdf)
- [MTi-G-710](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-G-710.pdf)

### Xsens Sirius series:
- [Sirius IMU](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20IMU.pdf)
- [Sirius VRU](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20VRU.pdf)
- [Sirius AHRS](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20AHRS.pdf)

### Xsens Vision Navigator
[Xsens Vision Navigator](https://www.movella.com/hubfs/Downloads/Leaflets/Xsens%20Vision%20Navigator.pdf)

## MTi White Paper:
[MTi White Paper](https://www.movella.com/hubfs/Downloads/Whitepapers/MTi_whitepaper.pdf)