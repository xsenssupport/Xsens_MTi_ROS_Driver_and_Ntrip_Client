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
3. [Quick start for MTi Development Kit:](https://base.movella.com/s/article/Quick-start-for-MTi-Development-Kit)
4. [Where to find the serial number on the MTi](https://base.movella.com/s/article/Where-to-find-the-serial-number-on-the-MTi)
5. [Introduction to the MT SDK programming examples for MTi devices](https://base.movella.com/s/article/Introduction-to-the-MT-SDK-programming-examples-for-MTi-devices)
6. [Open source XDA (Xsens Device API)](https://base.movella.com/s/article/Open-source-XDA-Xsens-Device-API)
7. [Recording a MTB file with XDA Processing:](https://base.movella.com/s/article/Recording-a-MTB-file-with-XDA-Processing)
8. [Selecting Filter Profile:](https://base.movella.com/s/article/article/Selecting-Filter-Profile)
9. [MTi Filter Profiles:](https://base.movella.com/s/article/article/MTi-Filter-Profiles-1605869708823)
10. [The difference between XDA Processing and Onboard Processing:](https://base.movella.com/s/article/article/The-difference-between-XDA-Processing-and-Onboard-Processing)
11. [Using the MT Manager Device Data View:](https://base.movella.com/s/article/article/Using-the-MT-Manager-Device-Data-View)
12. [Restoring communication with your MTi:](https://base.movella.com/s/article/article/Restoring-communication-with-your-MTi)
13. [MT Initialization time:](https://base.movella.com/s/article/article/MT-Initialization-time)
14. [MT Software Suite Log Files:](https://base.movella.com/s/article/article/MT-Software-Suite-Log-Files)
15. [Editing MTB Files:](https://base.movella.com/s/article/article/Editing-MTB-Files)
16. [What is the message with MID 0x98 in my MTB file?](https://base.movella.com/s/article/article/What-is-the-message-with-MID-0x98-in-my-MTB-file)
17. [Make a log file compatible with MT Manager:](https://base.movella.com/s/article/article/Make-a-log-file-compatible-with-MT-Manager)
18. [Troubleshooter - When MT Manager and MFM are not working due to runtime-API missing from your computer (Windows 7):](https://base.movella.com/s/article/article/Troubleshooter-When-MT-Manager-and-MFM-are-not-working-due-to-runtime-API-missing-from-your-computer-Windows-7)
19. [Reset not working with USB cable in MT Manager:](https://base.movella.com/s/article/article/Reset-not-working-with-USB-cable-in-MT-Manager)
20. [My MTi-6XX is not detected / Installing the MTi USB dongle driver for Linux:](https://base.movella.com/s/article/article/My-MTi-6XX-is-not-detected-Installing-the-MTi-USB-dongle-driver-for-Linux)
21. [USB Communication on embedded Linux:](https://base.movella.com/s/article/article/USB-Communication-on-embedded-Linux)
22. [Legacy Mode and 3rd generation MTi no longer supported](https://base.movella.com/s/article/article/Legacy-Mode-and-3rd-generation-MTi-no-longer-supported-1605869708049)
23. [Where can I download a USB or COM-port driver? :](https://base.movella.com/s/article/article/Where-can-I-download-a-USB-or-COM-port-driver)
24. [How to use Device Data View to learn MT Low Level Communications](https://base.movella.com/s/article/article/How-to-use-Device-Data-View-to-learn-MT-Low-Level-Communications)
25. [MT SDK: Changing the amount of stop bits used for serial communication](https://base.movella.com/s/article/article/MT-SDK-Changing-the-amount-of-stop-bits-used-for-serial-communication)
26. [How to do Representative Motion(RepMo) Programatically with Low Level Xbus Messages](https://base.movella.com/s/article/How-to-do-Representative-Motion-RepMo-Programatically-with-Low-Level-Xbus-Messages)
27. [How to install MT Manager in ubuntu 20.04 and ubuntu 22.04](https://base.movella.com/s/question/0D509000016hfSQCAY/update-mtm-to-support-modern-ubuntu)
28. [How to Post Processing MTB File to Magnetic Field Calibration Result](https://base.movella.com/s/article/How-to-Post-Processing-MTB-File-to-Magnetic-Field-Calibration-Result)
29. [What to do when you are not able to connect to an MTi device in MT Manager or using Xsens Device API](https://base.movella.com/s/article/What-to-do-when-you-are-not-able-to-connect-to-an-MTi-device-in-MT-Manager-or-using-Xsens-Device-API)
30. [Downloading beta firmware versions using the Firmware Updater](https://base.movella.com/s/article/Downloading-beta-firmware-versions-using-the-Firmware-Updater)


#### 3rd Party Integrations
1. [Third Party Drivers for use with the MTi:](https://base.movella.com/s/article/article/Third-Party-Drivers-for-use-with-the-MTi)
2. [Interfacing an MTi GNSS/INS device with a Velodyne Lidar:](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-Velodyne-Lidar)
3. [Interfacing an MTi GNSS/INS device with an Ouster Lidar:](https://base.movella.com/s/article/article/Interfacing-with-Ouster-Lidar-for-Xsens)
4. [Interfacing an MTi GNSS/INS device with a HESAI Lidar:](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-HESAI-Lidar)
5. [Best practices I2C and SPI for MTi 1-series:](https://base.movella.com/s/article/article/Best-practices-I2C-and-SPI-for-MTi-1-series-1605869706124)
6. [Interfacing MTi devices with the NVIDIA Jetson:](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176)
7. [Interfacing the MTi 1-series DK with an Arduino:](https://base.movella.com/s/article/article/Interfacing-the-MTi-1-series-DK-with-an-Arduino)
8. [Interfacing MTi devices with the Raspberry Pi:](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-Raspberry-Pi)
9. [Using an NTRIP client with the Xsens ROS driver:](https://base.movella.com/s/article/article/Using-an-NTRIP-client-with-the-Xsens-ROS-driver)
10. [Interfacing the MTi-680G with a Racelogic VBOX NTRIP modem:](https://base.movella.com/s/article/article/Interfacing-the-MTi-680G-with-a-Racelogic-VBOX-NTRIP-modem)
11. [Using a mobile hotspot (NTRIP) to receive RTK corrections on the MTi-680-DK](https://base.movella.com/s/article/article/Using-a-mobile-hotspot-NTRIP-to-receive-RTK-corrections-on-the-MTi-680-DK)
12. [Using the MTi series with a data logger:](https://base.movella.com/s/article/article/Using-the-MTi-series-with-a-data-logger)
13. [Interfacing a GNSS/INS device with the SinoGNSS K803 GNSS receiver:](https://base.movella.com/s/article/article/Interfacing-MTi-670-with-SinoGNSS-K803-GNSS-receiver)
14. [Interfacing a GNSS/INS device with the Unicorecomm UB482 GNSS receiver:](https://base.movella.com/s/article/article/Interfacing-a-GNSS-INS-device-with-the-Unicorecomm-UB482-GNSS-receiver)
15. [Interfacing a GNSS/INS device with the Septentrio Mosaic-X5 GNSS receiver:](https://base.movella.com/s/article/article/Interfacing-a-GNSS-INS-device-with-the-Septentrio-Mosaic-X5-GNSS-receiver)
16. [Interfacing an MTi GNSS/INS device with an ArduSimple SimpleRTK2B-Pro](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-an-ArduSimple-SimpleRTK2B-Pro)
17. [mikroBUS (transceiver) compatibility of the MTi 1-series Development Board](https://base.movella.com/s/article/article/mikroBUS-transceiver-compatibility-of-the-MTi-1-series-Development-Board)
18. [mikroBUS (GNSS) compatibility of the MTi 1/600-series Development Board](https://base.movella.com/s/article/article/Mikrobus-compatibility-of-the-MTi-1-600-series-Development-Board)
19. [Using the MTi series with a serial-to-USB converter](https://base.movella.com/s/article/Using-the-MTi-series-with-a-serial-to-USB-converter)
20. [How to configure and use U-blox PointPerfect L-band RTK-SSR corrections with MTi-680/MTi-8](https://base.movella.com/s/article/How-to-configure-and-use-U-blox-PointPerfect-L-band-RTK-SSR-corrections-with-MTi-680-MTi-8)
21. [How to use u-blox PointPerfect NTRIP service with MTi-680(G)/MTi-8 for RTK](https://base.movella.com/s/article/How-to-use-u-blox-PointPerfect-NTRIP-service-with-MTi-680-G-MTi-8-for-RTK)
22. [Using the NTRIP-X Base Station with Xsens MTi and XVN products](https://base.movella.com/s/article/Using-the-NTRIP-X-Base-Station-with-Xsens-MTi-and-XVN-products)
23. [Receiving SBAS signals on the MTi series from a u-blox GNSS Receiver](https://base.movella.com/s/article/Receiving-SBAS-signals-on-the-MTi-series-from-a-u-blox-GNSS-Receiver)

#### Knowledge (Algorithms, data and theory)
1. [Filter profiles for MTi 1-series:](https://base.movella.com/s/article/article/Filter-profiles-for-MTi-1-series-1605870241087)
2. [Active Heading Stabilization (AHS):](https://base.movella.com/s/article/article/Active-Heading-Stabilization-AHS)
3. [In-run Compass Calibration (ICC) and Representative Motion](https://base.movella.com/s/article/article/In-run-Compass-Calibration-ICC-and-Representative-Motion)
4. [Synchronization with the MTi:](https://base.movella.com/s/article/article/Synchronization-with-the-MTi)
5. [ClockSync and StartSampling:](https://base.movella.com/s/article/article/ClockSync-and-StartSampling)
6. [Heading estimation using GNSS acceleration:](https://base.movella.com/s/article/article/Heading-estimation-using-GNSS-acceleration)
7. [Converting raw 16-bit ADC values to physical values:](https://base.movella.com/s/article/article/Converting-raw-16-bit-ADC-values-to-physical-values-1605869706647)
8. [Estimating Yaw in magnetically disturbed environments:](https://base.movella.com/s/article/article/Estimating-Yaw-in-magnetically-disturbed-environments)
9. [Best Practices for Automotive Applications:](https://base.movella.com/s/article/article/Best-Practices-for-Automotive-Applications)
10. [An explanation of the MTi fixed point output formats:](https://base.movella.com/s/article/article/An-explanation-of-the-MTi-fixed-point-output-formats-1605869706093)
11. [Example: How to determine the correct alignment matrices (RotSensor/RotLocal) for your application](https://base.movella.com/s/article/article/Example-How-to-determine-the-correct-alignment-matrices-RotSensor-RotLocal-for-your-application)
12. [Using different output formats for position and velocity:](https://base.movella.com/s/article/article/Using-different-output-formats-for-position-and-velocity)
13. [Reasons why the heading/yaw is not stable or incorrect:](https://base.movella.com/s/article/article/Reasons-why-the-heading-yaw-is-not-stable-or-incorrect)
14. [What is the relationship between gyroscopes and accelerometers in the orientation algorithm?:](https://base.movella.com/s/article/article/What-is-the-relationship-between-gyroscopes-and-accelerometers-in-the-orientation-algorithm)
15. [Using an MTi to estimate ship motion (Heave, Surge, Sway):](https://base.movella.com/s/article/article/Using-an-MTi-to-estimate-ship-motion-Heave-Surge-Sway)
16. [Using the magnetic norm to identify magnetic distortions:](https://base.movella.com/s/article/article/Using-the-magnetic-norm-to-identify-magnetic-distortions)
17. [What is the minimal velocity for the GNSS/INS devices to estimate its yaw?:](https://base.movella.com/s/article/article/What-is-the-minimal-velocity-for-the-GNSS-INS-devices-to-estimate-its-yaw)
18. [Which GNSS antenna to use with your GNSS/INS device:](https://base.movella.com/s/article/article/Which-GNSS-antenna-to-use-with-your-GNSS-INS-device)
19. [RMS noise of accelerometers and gyroscopes:](https://base.movella.com/s/article/article/RMS-noise-of-accelerometers-and-gyroscopes)
20. [Orientation output specifications:](https://base.movella.com/s/article/article/Orientation-output-specifications)
21. [Signal processing pipeline - fidelity of 400 Hz signal:](https://base.movella.com/s/article/article/Signal-processing-pipeline-fidelity-of-400-Hz-signal)
22. [The GNSS lever arm (antenna offset) and its role in the GNSS/INS sensor fusion algorithm](https://base.movella.com/s/article/article/The-GNSS-lever-arm-antenna-offset-and-its-role-in-the-GNSS-INS-sensor-fusion-algorithm)
23. [Transforming MTi data from local frame (L) to sensor frame (S)](https://base.movella.com/s/article/article/Transforming-MTi-data-from-local-frame-L-to-sensor-frame-S)
24. [Understanding Sensor Bias (offset):](https://base.movella.com/s/article/article/Understanding-Sensor-Bias-offset)
25. [Understanding Strapdown Integration:](https://base.movella.com/s/article/article/Understanding-Strapdown-Integration)
26. [Magnetic distortions caused by strong electrical currents:](https://base.movella.com/s/article/article/Magnetic-distortions-caused-by-strong-electrical-currents)
27. [Manual Gyro Bias Estimation (MGBE):](https://base.movella.com/s/article/article/Manual-Gyro-Bias-Estimation)
28. [Calculating the Magnetic Norm (Mag Norm):](https://base.movella.com/s/article/article/Calculating-the-Magnetic-Norm-Mag-Norm)
29. [Understanding Gimbal Lock and how to prevent it:](https://base.movella.com/s/article/Understanding-Gimbal-Lock-and-how-to-prevent-it)
30. [How do I calculate position and/or velocity from acceleration and how about integration drift?:](https://base.movella.com/s/article/article/How-do-I-calculate-position-and-or-velocity-from-acceleration-and-how-about-integration-drift)
31. [Interpreting magnetic field data represented as an arbitrary unit (a.u.):](https://base.movella.com/s/article/article/Interpreting-magnetic-field-data-represented-as-an-arbitrary-unit-a-u)
32. [MTi 10/100-series Origin of Coordinate system:](https://base.movella.com/s/article/article/MTi-10-100-series-Origin-of-Coordinate-system)
33. [MTi Test and Calibration values explained:](https://base.movella.com/s/article/article/MTi-Test-and-Calibration-values-explained)
34. [MTi 1-series Origin of Coordinate system:](https://base.movella.com/s/article/article/MTi-1-series-Origin-of-Coordinate-system-1605870241700)
35. [Mounting the MTi-OEM:](https://base.movella.com/s/article/article/Mounting-the-MTi-OEM-1605869708435)
36. [Magnetic immunity of the MTi AHRS:](https://base.movella.com/s/article/article/Magnetic-immunity-of-the-MTi-AHRS-1605869708053)
37. [Changing or Resetting the MTi reference co-ordinate systems:](https://base.movella.com/s/article/article/Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643)
38. [Delays introduced by serial communication interfaces for the MTi series:](https://base.movella.com/s/article/article/Delays-introduced-by-serial-communication-interfaces-for-the-MTi-series)
39. [Why does an accelerometer measure gravity with positive sign](https://base.movella.com/s/article/Why-does-an-accelerometer-measure-gravity-with-positive-sign)
40. [How to obtain the correct RotSensor matrix](https://base.movella.com/s/article/How-to-obtain-the-correct-RotSensor-matrix)
41. [How to connect MTi devices using Ethernet protocol](https://base.movella.com/s/article/How-to-connect-MTi-devices-using-Ethernet-protocol)
42. [Understanding the relationship between the noise characteristics](https://base.movella.com/s/article/Understanding-the-relationship-between-the-noise-characteristics)

#### Firmware
1. [How to determine the Firmware version of your MTi:](https://base.movella.com/s/article/article/How-to-determine-the-Firmware-version-of-your-MTi-1605869707397)
2. [Using the Firmware Updater with a beta firmware:](https://base.movella.com/s/article/article/Using-the-Firmware-Updater-with-a-beta-firmware)
3. [Device was not in bootloader when expected error](https://base.movella.com/s/article/article/Device-was-not-in-bootloader-when-expected-error)
4. [Firmware Updater freezes while scanning for devices](https://base.movella.com/s/article/article/Firmware-Updater-freezes-while-scanning-for-devices)
5. [Firmware Updater Log](https://base.movella.com/s/article/article/Firmware-Updater-Log)
6. [How to fix Firmware Updater error "Could not write eMTS"](https://base.movella.com/s/article/article/How-to-fix-Firmware-Updater-error-Could-not-write-eMTS)
7. [How to fix Magnetic Field Mapper Error 286 "Failed to start logging"](https://base.movella.com/s/article/article/How-to-fix-Magnetic-Field-Mapper-Error-286-Failed-to-start-logging)
8. [MTi in bootloader mode](https://base.movella.com/s/article/article/MTi-in-bootloader-mode)
9. [MTi Release Notes and Changelogs](https://base.movella.com/s/article/article/MTi-Release-Notes-and-Changelogs)
10. [Product Change Notifications (PCNs) and End of Life (EoL) notices](https://base.movella.com/s/article/article/Product-Change-Notifications-PCNs-and-End-of-Life-EoL-notices)


#### Xsens Sirius series
1. [Migration: Xsens MTi 100-series to Xsens Sirius-series](https://base.movella.com/s/article/Migration-Xsens-MTi-100-series-to-Xsens-Sirius-series)
2. [STEP-file for the Xsens Sirius series](https://base.movella.com/s/article/STEP-file-for-the-Xsens-Sirius-series)

#### MTi 10/100 – series
1. [AD resolution of the MTi 10-series and MTi 100-series:](https://base.movella.com/s/article/article/AD-resolution-of-the-MTi-10-series-and-MTi-100-series-1605869706076)
2. [Migration: MTi 10/100-series to MTi 600-series:](https://base.movella.com/s/article/article/Migration-MTi-10-100-series-to-MTi-600-series-1605870241697)
3. [Migration to 5th generation MTi 10/100-series (2017):](https://base.movella.com/s/article/article/Migration-to-5th-generation-MTi-10-100-series-1605869708410)
4. [Mounting the MTi-OEM:](https://base.movella.com/s/article/article/Mounting-the-MTi-OEM-1605869708435)
5. [MTi 10/100-series Origin of Coordinate system:](https://base.movella.com/s/article/article/MTi-10-100-series-Origin-of-Coordinate-system)
6. [Strength of MTi 10/100-series casing:](https://base.movella.com/s/article/article/Strength-of-MTi-10-100-series-casing)
7. [STEP-files for MTi 10/100-series:](https://base.movella.com/s/article/article/STEP-files-for-MTi-10-100-series)
8. [Use of MTi with longer cables:](https://base.movella.com/s/article/article/Use-of-MTi-with-longer-cables)
9. [MTi 10/100-series connectors and cables:](https://base.movella.com/s/article/article/MTi-10-100-series-connectors-and-cables)
10. [CA-MP2-MTi multi-purpose cable Molex connector part numbers:](https://base.movella.com/s/article/article/CA-MP2-MTi-multi-purpose-cable-Molex-connector-part-numbers-1605869706131)




#### MTi 600 – series
1. [Use of MTi with longer cables:](https://base.movella.com/s/article/article/Use-of-MTi-with-longer-cables)
2. [My MTi-6XX is not detected / Installing the MTi USB dongle driver for Linux:](https://base.movella.com/s/article/article/My-MTi-6XX-is-not-detected-Installing-the-MTi-USB-dongle-driver-for-Linux)
3. [STEP-files for the MTi 600-series](https://base.movella.com/s/article/article/STEP-files-for-the-MTi-600-series)
4. [How to receive RTCM correction messages from MTi-680(G)](https://base.movella.com/s/article/How-to-receive-RTCM-correction-messages-for-MTi-680-G)
5. [Introduction to RTK](https://base.movella.com/s/article/article/Introduction-to-RTK)
6. [Solving USB communication issues for the RTK GNSS daughter card](https://base.movella.com/s/article/article/Solving-USB-communication-issues-for-the-RTK-GNSS-daughter-card)
7. [.dbc file for MTi 600-series](https://base.movella.com/s/article/article/dbc-file-for-MTi-600-series-1605870241065)



#### MTi 1 – series
1. [Altium library and STEP-file for MTi 1-series:](https://base.movella.com/s/article/article/Altium-library-and-STEP-file-for-MTi-1-series-1605870241073)
2. [Interfacing the MTi 1-series DK with an Arduino](https://base.movella.com/s/article/article/Interfacing-the-MTi-1-series-DK-with-an-Arduino)
3. [Using the MTi 1-series module without reflow process](https://base.movella.com/s/article/article/Using-the-MTi-1-series-module-without-reflow-process)
4. [MTi 1-series Hardware Version 1.x Firmware Updates](https://base.movella.com/s/article/article/MTi-1-series-Hardware-Version-1-x-Firmware-Updates)
5. [MTi 1-series Origin of Coordinate system](https://base.movella.com/s/article/article/MTi-1-series-Origin-of-Coordinate-system-1605870241700)
6. [mikroBUS (transceiver) compatibility of the MTi 1-series Development Board](https://base.movella.com/s/article/article/mikroBUS-transceiver-compatibility-of-the-MTi-1-series-Development-Board)
7. [Migration to Hardware Version 2.0 of the MTi 1-series (June 2018)](https://base.movella.com/s/article/article/Migration-to-Hardware-Version-2-0-of-the-MTi-1-series-1605870241149)
8. [Migration to Hardware Version 3 of the MTi 1-series (August 2021)](https://base.movella.com/s/article/article/Migration-to-Hardware-Version-3-of-the-MTi-1-series)
9. [Migration to Hardware Version 5 of the MTi 1-series (May 2023)](https://base.movella.com/s/article/Migration-to-Hardware-Version-5-of-the-MTi-1-series)
10. [Filter profiles for MTi 1-series](https://base.movella.com/s/article/article/Filter-profiles-for-MTi-1-series-1605870241087)
11. [Best practices I2C and SPI for MTi 1-series](https://base.movella.com/s/article/article/Best-practices-I2C-and-SPI-for-MTi-1-series-1605869706124)
12. [Embedded Firmware Updater (FWU) for MTi 1-series](https://base.movella.com/s/article/article/Embedded-Firmware-Updater-FWU-for-MTi-1-series)
13. [Migration to Hardware Version 3 of the MTi-320 (June 2023)](https://base.movella.com/s/article/Migration-to-Hardware-Version-3-of-the-MTi-320)
14. [STEP files for the MTi-320 sensor](https://base.movella.com/s/article/STEP-files-for-the-MTi-320-sensor)



#### Xsens Vision Navigator
1. [Xsens Vision Navigator: Frequently Asked Questions (FAQ)](https://base.movella.com/s/article/Xsens-Vision-Navigator-Frequently-Asked-Questions-FAQ)
2. [STEP files for the Xsens Vision Navigator (XVN)](https://base.movella.com/s/article/STEP-files-for-the-Xsens-Vision-Navigator-XVN)
3. [Starting Fusion Indoors for Xsens Vision Navigator (XVN)](https://base.movella.com/s/article/Starting-Fusion-Indoors-for-Xsens-Vision-Navigator-XVN)
4. [Updating the Firmware of the Xsens Vision Navigator (XVN)](https://base.movella.com/s/article/Updating-the-Firmware-of-the-Xsens-Vision-Navigator-XVN)
5. [Camera Image Streaming with Xsens Vision Navigator](https://base.movella.com/s/article/Camera-Image-Streaming-with-Xsens-Vision-Navigator)
6. [Generating a log file of the Xsens Vision Navigator (XVN)](https://base.movella.com/s/article/Generating-a-log-file-of-the-Xsens-Vision-Navigator-XVN)
7. [Troubleshooting Xsens Vision Navigator (XVN)](https://base.movella.com/s/article/Troubleshooting-Xsens-Vision-Navigator-XVN)




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