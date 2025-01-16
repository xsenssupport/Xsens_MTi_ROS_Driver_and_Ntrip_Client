## Xsens MTi 文档

所有 Xsens MTi 产品手册均可在此处找到： [https://mtidocs.movella.com/home](https://mtidocs.movella.com/home)

下面根据不同型号列出相应的手册目录及链接：

## MTi-1 系列手册（包括 MTi-1、MTi-2、MTi-3、MTi-7、MTi-8）

-   [MTi 系列参考手册](https://mtidocs.movella.com/mti-family-reference-manual)
-   [MT Manager用户手册](https://mtidocs.movella.com/mt-manager)
-   [固件更新程序用户手册](https://mtidocs.movella.com/firmware-updater)
-   [磁校准手册](https://mtidocs.movella.com/magnetic-calibration-manual)
-   [MTi 1 系列数据表](https://mtidocs.movella.com/datasheet)
-   [MTi 1 系列 DK 用户手册](https://mtidocs.movella.com/use-manual)
-   [MTi 1 系列硬件集成手册](https://mtidocs.movella.com/integration-manual)
-   [MT 低级通信协议文档](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## MTi-320 手册

[MTi-320 手册](https://mtidocs.movella.com/mti-320)

## MTi-600 系列手册

（包括MTi-610、MTi-620、MTi-630、MTi-670、MTi-680；MTi-610R、MTi-620R、MTi-630R；MTi-670G、MTi-680G）

-   [MTi 系列参考手册](https://mtidocs.movella.com/mti-family-reference-manual)
-   [MT Manager用户手册](https://mtidocs.movella.com/mt-manager)
-   [固件更新程序用户手册](https://mtidocs.movella.com/firmware-updater)
-   [磁校准手册](https://mtidocs.movella.com/magnetic-calibration-manual)
-   [MTi 600 系列数据表](https://mtidocs.movella.com/mti-600-series-datasheet)
-   [MTi 600 系列 DK 用户手册](https://mtidocs.movella.com/dk-user-manual-2)
-   [MTi 600 系列硬件集成手册](https://mtidocs.movella.com/hardware-integration-manual-2)
-   [MT CAN 协议文档](https://mtidocs.movella.com/mt-can-documentation)
-   [MT 低级通信协议文档](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## MTi-100 系列手册

（包括MTi-100、MTi-200、MTi-300、MTi-G-710；MTi-10、MTi-20、MTi-30）

-   [MTi 系列参考手册](https://mtidocs.movella.com/mti-family-reference-manual)
-   [MT Manager用户手册](https://mtidocs.movella.com/mt-manager)
-   [固件更新程序用户手册](https://mtidocs.movella.com/firmware-updater)
-   [磁校准手册](https://mtidocs.movella.com/magnetic-calibration-manual)
-   [MTi 10/100 系列](https://mtidocs.movella.com/mti-10-100-series-user-manual)
-   [MT 低级通信协议文档](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## Xsens Sirius 系列手册

（包括 Sirius IMU、VRU、AHRS 型号）

-   [系列参考手册](https://mtidocs.movella.com/mti-family-reference-manual)
-   [MT Manager用户手册](https://mtidocs.movella.com/mt-manager)
-   [固件更新程序用户手册](https://mtidocs.movella.com/firmware-updater)
-   [磁校准手册](https://mtidocs.movella.com/magnetic-calibration-manual)
-   [Xsens Sirius 系列数据表](https://mtidocs.movella.com/xsens-sirius-series-datasheet)
-   [Xsens Sirius 系列 DK 用户手册](https://mtidocs.movella.com/xsens-sirius-dk-user-manual)
-   [Xsens Sirius 系列硬件集成手册](https://mtidocs.movella.com/xsens-sirius-series-hardware-integration-manual)
-   [MT 低级通信协议文档](https://mtidocs.movella.com/mt-low-level-communication-protocol-documentation)

## Xsens Vision Navigator 使用手册

-   [快速入门指南](https://mtidocs.movella.com/vn-quick-start-guide)
-   [产品规格书](https://mtidocs.movella.com/vn-datasheet)
-   [硬件集成手册](https://mtidocs.movella.com/vn-integration-manual)

## 软件安装

### MT软件套件

（包括两个软件：MT Manager 和 Magnetic Field Mapper）请在 MTi 产品部分下载：[https://www.movella.com/support/software-documentation](https://www.movella.com/support/software-documentation)

### Linux MT Manager

*不适用于 Nvidia Jetson 或 Rasperry-Pi*

```bash
sudo tar -xf MT_Software_Suite_linux-x64_2022.0_b7085_r119802.tar.gz
sudo chmod -R o+rw MT_Software_Suite_linux-x64_2022.0/
cd MT_Software_Suite_linux-x64_2022.0
sudo tar -xf mtmanager_linux-x64_2022.0.tar.gz
sudo chmod -R o+rw mtmanager/
```

然后参考文档中的自述文件（位于 mtmanager/linux-64/doc/MTM.README）来安装 MT Manager 的依赖项。

参考：[Ubuntu 20.04 和 22.04 版 MT Manager 安装指南](https://base.movella.com/s/article/MT-Manager-Installation-Guide-for-ubuntu-20-04-and-22-04)

### 2）固件更新程序

该软件仅用于固件更新，请查看固件发行说明，了解是否需要更新固件。

-   软件下载：[https://www.movella.com/hubfs/FirmwareUpdater.zip](https://www.movella.com/hubfs/FirmwareUpdater.zip)
-   对于 MTi-680G，请确保已更新至最新固件 1.12.0

## 示例代码

### Windows 示例代码：

```
C:\Program Files\Xsens\MT Software Suite 2022.0\MT SDK\Examples
```

### MT SDK 文档：

```
C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK
```

- Xsensdeviceapi:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xsensdeviceapi\doc\html\index.html`
- Xscontroller:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xscontroller\doc\html\index.html`
- Xstypes:
`C:\Program Files\Xsens\MT Software Suite 2022.0\Documentation\MT SDK\xstypes\doc\html\index.html`

### Linux 示例代码

下载MT软件套件后，解压到自己选择的安装目录，默认为`/usr/local/xsens`

```bash
sudo ./mtsdk_linux-x64_2022.0.sh
```

示例代码默认位于：`/usr/local/xsens/examples/mtsdk` SDK 文档默认位于：`/usr/local/xsens/doc`，与 Windows 相同，会有 xscontroller、xsensdeviceapi、xstypes。

对于 ARM 架构（NVidia Jetson、RasperryPi）：

-   MT Manager，xda_cpp 无法使用
-   xda_public_cpp 可以使用

xda\_public\_cpp，默认位于`/usr/local/xsens/examples/mtsdk/xda_public_cpp`，有两个cpp文件：

1.  `example_mti_receive_data.cpp`是示例代码，可帮助：扫描端口、连接、配置 MTi 的输出、启动测量、记录 .mtb 文档
2.  `example_mti_parse_logfile.cpp`，可用于将 Xsens 日志文件 .mtb 文档转换为 .txt

## ROS 驱动程序

### ROS1

[https://github.com/xsenssupport/Xsens\_MTi\_ROS\_Driver\_and\_Ntrip\_Client/tree/main](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/main)

### ROS2（Humble and Jazzy Jalisco）

[https://github.com/xsenssupport/Xsens\_MTi\_ROS\_Driver\_and\_Ntrip\_Client/tree/ros2](https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2)

注意：NTRIP 客户端适用于 MTi-8、MTi-680、MTi-680G 等产品的 RTK GNSS/INS 版本。如果您有 IMU/VRU/AHRS 单元，则可以忽略 NTRIP 客户端，并在 xsens_mti_node.yaml 中将 pub_positionLLA/pub_nmea/pub_gnss/pub_gnsspose 更改为 false。

## 视频教程

1.  [选择您的 MTi 产品](https://tutorial.movella.com/video/choose-your-mti-product)
2.  [开始使用 MTi](https://tutorial.movella.com/video/getting-started-with-mti)
3.  [MT Manager 2019](https://tutorial.movella.com/video/mt-manager-2019)
4.  [MTi-600 传感器融合滤波器 (VRU、AHRS)](https://tutorial.movella.com/video/xsens-mti-ahrs)
5.  [磁场测绘仪（MFM）磁校准](https://tutorial.movella.com/video/magnetic-calibration)
6.  [车内磁校准（2D磁校准）](https://www.youtube.com/watch?v=3YJMUPFdYH8)
7.  [主动航向稳定AHS](https://tutorial.movella.com/video/active-heading-stabilization-ahs)
8.  [手动陀螺仪零偏估计](https://tutorial.movella.com/video/manual-gyro-bias-estimation)
9.  [处理机器人和工业应用中的磁畸变](https://tutorial.movella.com/video/handling-magnetic-distortion-in-robotic-and-industrial-applications)
10.  [设备数据视图](https://tutorial.movella.com/video/device-data-view)
11.  [XDA 处理](https://tutorial.movella.com/video/xda-processing)
12.  [Xsens Vision Navigator 简介](https://www.youtube.com/watch?v=_7AzFv3s0DI)

## Xsens BASE 知识文章

[https://base.movella.com/s/ism-landing-page](https://base.movella.com/s/ism-landing-page)

### 详细链接

#### MT 软件套件/SDK

1.  [MTi 系列的文档、宣传单和更新日志链接](https://base.movella.com/s/article/Online-links-to-manuals-from-the-MT-Software-Suite)
2.  [为不同（代）产品安装正确的软件套件](https://base.movella.com/s/article/Installing-the-correct-Software-Suite-for-different-generation-products)
3.  [MTi 开发套件快速入门](https://base.movella.com/s/article/Quick-start-for-MTi-Development-Kit)
4.  [在哪里可以找到 MTi 上的序列号](https://base.movella.com/s/article/Where-to-find-the-serial-number-on-the-MTi)
5.  [MTi 设备的 MT SDK 编程示例介绍](https://base.movella.com/s/article/Introduction-to-the-MT-SDK-programming-examples-for-MTi-devices)
6.  [开源 XDA (Xsens 设备 API)](https://base.movella.com/s/article/Open-source-XDA-Xsens-Device-API)
7.  [使用 XDA Processing 录制 MTB 文件](https://base.movella.com/s/article/Recording-a-MTB-file-with-XDA-Processing)
8.  [选择滤波器配置文件](https://base.movella.com/s/article/article/Selecting-Filter-Profile)
9.  [MTi 滤波器配置文件](https://base.movella.com/s/article/article/MTi-Filter-Profiles-1605869708823)
10.  [XDA Processing 和 Onboard Processing 的区别](https://base.movella.com/s/article/article/The-difference-between-XDA-Processing-and-Onboard-Processing)
11.  [使用 MT Manager设备数据视图](https://base.movella.com/s/article/article/Using-the-MT-Manager-Device-Data-View)
12.  [恢复与 MTi 的通信](https://base.movella.com/s/article/article/Restoring-communication-with-your-MTi)
13.  [MT初始化时间](https://base.movella.com/s/article/article/MT-Initialization-time)
14.  [MT 软件套件日志文件](https://base.movella.com/s/article/article/MT-Software-Suite-Log-Files)
15.  [编辑 MTB 文件](https://base.movella.com/s/article/article/Editing-MTB-Files)
16.  [我的 MTB 文件中 MID 0x98 的消息是什么？](https://base.movella.com/s/article/article/What-is-the-message-with-MID-0x98-in-my-MTB-file)
17.  [制作与 MT Manager兼容的日志文件](https://base.movella.com/s/article/article/Make-a-log-file-compatible-with-MT-Manager)
18.  [疑难解答 - 当 MT Manager和 MFM 由于计算机缺少运行时 API 而无法运行时（Windows 7）](https://base.movella.com/s/article/article/Troubleshooter-When-MT-Manager-and-MFM-are-not-working-due-to-runtime-API-missing-from-your-computer-Windows-7)
19.  [MT Manager中无法使用 USB 电缆重置](https://base.movella.com/s/article/article/Reset-not-working-with-USB-cable-in-MT-Manager)
20.  [我的 MTi-6XX 未被检测到/为 Linux 安装 MTi USB 加密狗驱动程序](https://base.movella.com/s/article/article/My-MTi-6XX-is-not-detected-Installing-the-MTi-USB-dongle-driver-for-Linux)
21.  [嵌入式Linux上的USB通信](https://base.movella.com/s/article/article/USB-Communication-on-embedded-Linux)
22.  [不再支持传统模式和第三代 MTi](https://base.movella.com/s/article/article/Legacy-Mode-and-3rd-generation-MTi-no-longer-supported-1605869708049)
23.  [我可以在哪里下载 USB 或 COM 端口驱动程序？](https://base.movella.com/s/article/article/Where-can-I-download-a-USB-or-COM-port-driver)
24.  [如何使用设备数据视图了解 MT 低级通信](https://base.movella.com/s/article/article/How-to-use-Device-Data-View-to-learn-MT-Low-Level-Communications)
25.  [MT SDK：更改串行通信使用的停止位数](https://base.movella.com/s/article/article/MT-SDK-Changing-the-amount-of-stop-bits-used-for-serial-communication)
26.  [如何使用低级 Xbus 消息以编程方式执行代表性运动 (RepMo)](https://base.movella.com/s/article/How-to-do-Representative-Motion-RepMo-Programatically-with-Low-Level-Xbus-Messages)
27.  [如何在 ubuntu 20.04 和 ubuntu 22.04 中安装 MT Manager](https://base.movella.com/s/question/0D509000016hfSQCAY/update-mtm-to-support-modern-ubuntu)
28.  [如何将 MTB 文件后处理为磁场校准结果](https://base.movella.com/s/article/How-to-Post-Processing-MTB-File-to-Magnetic-Field-Calibration-Result)
29.  [当您无法在 MT Manager 中或使用 Xsens Device API 连接到 MTi 设备时该怎么办](https://base.movella.com/s/article/What-to-do-when-you-are-not-able-to-connect-to-an-MTi-device-in-MT-Manager-or-using-Xsens-Device-API)
30.  [使用固件更新程序下载测试版固件](https://base.movella.com/s/article/Downloading-beta-firmware-versions-using-the-Firmware-Updater)

#### 第三方集成

1.  [与 MTi 一起使用的第三方驱动程序](https://base.movella.com/s/article/article/Third-Party-Drivers-for-use-with-the-MTi)
2.  [将 MTi GNSS/INS 设备与 Velodyne Lidar 连接起](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-Velodyne-Lidar)
3.  [将 MTi GNSS/INS 设备与 Ouster Lidar 连接起来](https://base.movella.com/s/article/article/Interfacing-with-Ouster-Lidar-for-Xsens)
4.  [将 MTi GNSS/INS 设备与 HESAI 激光雷达连接起来](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-a-HESAI-Lidar)
5.  [MTi 1 系列的 I2C 和 SPI 最佳实践](https://base.movella.com/s/article/article/Best-practices-I2C-and-SPI-for-MTi-1-series-1605869706124)
6.  [将 MTi 设备与 NVIDIA Jetson 连接起来](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176)
7.  [将 MTi 1 系列 DK 与 Arduino 连接](https://base.movella.com/s/article/article/Interfacing-the-MTi-1-series-DK-with-an-Arduino)
8.  [将 MTi 设备与 Raspberry Pi 连接起来](https://base.movella.com/s/article/article/Interfacing-MTi-devices-with-the-Raspberry-Pi)
9.  [将 NTRIP 客户端与 Xsens ROS 驱动程序结合使用](https://base.movella.com/s/article/article/Using-an-NTRIP-client-with-the-Xsens-ROS-driver)
10.  [将 MTi-680G 与 Racelogic VBOX NTRIP 调制解调器连接](https://base.movella.com/s/article/article/Interfacing-the-MTi-680G-with-a-Racelogic-VBOX-NTRIP-modem)
11.  [使用移动热点 (NTRIP) 在 MTi-680-DK 上接收 RTK 校正](https://base.movella.com/s/article/article/Using-a-mobile-hotspot-NTRIP-to-receive-RTK-corrections-on-the-MTi-680-DK)
12.  [将 MTi 系列与数据记录器一起使用](https://base.movella.com/s/article/article/Using-the-MTi-series-with-a-data-logger)
13.  [将 GNSS/INS 设备与 SinoGNSS K803 GNSS 接收器连接起来](https://base.movella.com/s/article/article/Interfacing-MTi-670-with-SinoGNSS-K803-GNSS-receiver)
14.  [将 GNSS/INS 设备与 Unicorecomm UB482 GNSS 接收器连接起来](https://base.movella.com/s/article/article/Interfacing-a-GNSS-INS-device-with-the-Unicorecomm-UB482-GNSS-receiver)
15.  [将 GNSS/INS 设备与 Septentrio Mosaic-X5 GNSS 接收器连接起来](https://base.movella.com/s/article/article/Interfacing-a-GNSS-INS-device-with-the-Septentrio-Mosaic-X5-GNSS-receiver)
16.  [将 MTi GNSS/INS 设备与 ArduSimple SimpleRTK2B-Pro 连接起来](https://base.movella.com/s/article/article/Interfacing-an-MTi-GNSS-INS-device-with-an-ArduSimple-SimpleRTK2B-Pro)
17.  [MTi 1 系列开发板的 mikroBUS（收发器）兼容性](https://base.movella.com/s/article/article/mikroBUS-transceiver-compatibility-of-the-MTi-1-series-Development-Board)
18.  [MTi 1/600 系列开发板的 mikroBUS (GNSS) 兼容性](https://base.movella.com/s/article/article/Mikrobus-compatibility-of-the-MTi-1-600-series-Development-Board)
19.  [使用 MTi 系列和串行转 USB 转换器](https://base.movella.com/s/article/Using-the-MTi-series-with-a-serial-to-USB-converter)
20.  [如何配置和使用 U-blox PointPerfect L 波段 RTK-SSR 校正与 MTi-680/MTi-8](https://base.movella.com/s/article/How-to-configure-and-use-U-blox-PointPerfect-L-band-RTK-SSR-corrections-with-MTi-680-MTi-8)
21.  [如何将 u-blox PointPerfect NTRIP 服务与 MTi-680(G)/MTi-8 结合使用以实现 RTK](https://base.movella.com/s/article/How-to-use-u-blox-PointPerfect-NTRIP-service-with-MTi-680-G-MTi-8-for-RTK)
22.  [将 NTRIP-X 基站与 Xsens MTi 和 XVN 产品配合使用](https://base.movella.com/s/article/Using-the-NTRIP-X-Base-Station-with-Xsens-MTi-and-XVN-products)
23.  [通过 u-blox GNSS 接收器接收 MTi 系列上的 SBAS 信号](https://base.movella.com/s/article/Receiving-SBAS-signals-on-the-MTi-series-from-a-u-blox-GNSS-Receiver)

#### 知识（算法、数据和理论）

1.  [MTi 1 系列的过滤配置文件](https://base.movella.com/s/article/article/Filter-profiles-for-MTi-1-series-1605870241087)
2.  [主动航向稳定系统（AHS）](https://base.movella.com/s/article/article/Active-Heading-Stabilization-AHS)
3.  [运动中指南针校准 (ICC) 和代表性运动](https://base.movella.com/s/article/article/In-run-Compass-Calibration-ICC-and-Representative-Motion)
4.  [与 MTi 同步](https://base.movella.com/s/article/article/Synchronization-with-the-MTi)
5.  [ClockSync 和 StartSampling](https://base.movella.com/s/article/article/ClockSync-and-StartSampling)
6.  [使用 GNSS 加速度进行航向估计](https://base.movella.com/s/article/article/Heading-estimation-using-GNSS-acceleration)
7.  [将原始 16 位 ADC 值转换为物理值](https://base.movella.com/s/article/article/Converting-raw-16-bit-ADC-values-to-physical-values-1605869706647)
8.  [估算磁干扰环境中的偏航角](https://base.movella.com/s/article/article/Estimating-Yaw-in-magnetically-disturbed-environments)
9.  [汽车应用的最佳实践](https://base.movella.com/s/article/article/Best-Practices-for-Automotive-Applications)
10.  [MTi 定点输出格式的解释](https://base.movella.com/s/article/article/An-explanation-of-the-MTi-fixed-point-output-formats-1605869706093)
11.  [示例：如何为您的应用确定正确的对准矩阵 (RotSensor/RotLocal)](https://base.movella.com/s/article/article/Example-How-to-determine-the-correct-alignment-matrices-RotSensor-RotLocal-for-your-application)
12.  [对位置和速度使用不同的输出格式](https://base.movella.com/s/article/article/Using-different-output-formats-for-position-and-velocity)
13.  [航向/偏航不稳定或者不正确的原因](https://base.movella.com/s/article/article/Reasons-why-the-heading-yaw-is-not-stable-or-incorrect)
14.  [陀螺仪和加速度计在定向算法中有什么关系？](https://base.movella.com/s/article/article/What-is-the-relationship-between-gyroscopes-and-accelerometers-in-the-orientation-algorithm)
15.  [使用 MTi 估计船舶运动（升沉、纵荡、横摇）](https://base.movella.com/s/article/article/Using-an-MTi-to-estimate-ship-motion-Heave-Surge-Sway)
16.  [使用磁标准来识别磁畸变](https://base.movella.com/s/article/article/Using-the-magnetic-norm-to-identify-magnetic-distortions)
17.  [GNSS/INS 设备估计其偏航的最小速度是多少？](https://base.movella.com/s/article/article/What-is-the-minimal-velocity-for-the-GNSS-INS-devices-to-estimate-its-yaw)
18.  [您的 GNSS/INS 设备应使用哪种 GNSS 天线](https://base.movella.com/s/article/article/Which-GNSS-antenna-to-use-with-your-GNSS-INS-device)
19.  [加速度计和陀螺仪的 RMS 噪声](https://base.movella.com/s/article/article/RMS-noise-of-accelerometers-and-gyroscopes)
20.  [方向输出规格](https://base.movella.com/s/article/article/Orientation-output-specifications)
21.  [信号处理管道-400 Hz 信号的保真度](https://base.movella.com/s/article/article/Signal-processing-pipeline-fidelity-of-400-Hz-signal)
22.  [GNSS 杠杆臂（天线偏移）及其在 GNSS/INS 传感器融合算法中的作用](https://base.movella.com/s/article/article/The-GNSS-lever-arm-antenna-offset-and-its-role-in-the-GNSS-INS-sensor-fusion-algorithm)
23.  [将 MTi 数据从本地框架 (L) 转换为传感器框架 (S)](https://base.movella.com/s/article/article/Transforming-MTi-data-from-local-frame-L-to-sensor-frame-S)
24.  [了解传感器偏差（偏移）](https://base.movella.com/s/article/article/Understanding-Sensor-Bias-offset)
25.  [了解捷联积分](https://base.movella.com/s/article/article/Understanding-Strapdown-Integration)
26.  [强电流引起的磁畸变](https://base.movella.com/s/article/article/Magnetic-distortions-caused-by-strong-electrical-currents)
27.  [手动陀螺仪偏差估计 (MGBE)](https://base.movella.com/s/article/article/Manual-Gyro-Bias-Estimation)
28.  [计算磁范数（Mag Norm）](https://base.movella.com/s/article/article/Calculating-the-Magnetic-Norm-Mag-Norm)
29.  [了解万向节锁及其预防方法](https://base.movella.com/s/article/Understanding-Gimbal-Lock-and-how-to-prevent-it)
30.  [如何根据加速度计算位置和/或速度以及积分漂移如何？](https://base.movella.com/s/article/article/How-do-I-calculate-position-and-or-velocity-from-acceleration-and-how-about-integration-drift)
31.  [解释以任意单位（au）表示的磁场数据](https://base.movella.com/s/article/article/Interpreting-magnetic-field-data-represented-as-an-arbitrary-unit-a-u)
32.  [MTi 10/100 系列坐标系原点](https://base.movella.com/s/article/article/MTi-10-100-series-Origin-of-Coordinate-system)
33.  [MTi 测试和校准值解释](https://base.movella.com/s/article/article/MTi-Test-and-Calibration-values-explained)
34.  [MTi 1系列坐标系原点](https://base.movella.com/s/article/article/MTi-1-series-Origin-of-Coordinate-system-1605870241700)
35.  [安装 MTi-OEM](https://base.movella.com/s/article/article/Mounting-the-MTi-OEM-1605869708435)
36.  [MTi AHRS 的磁抗扰度](https://base.movella.com/s/article/article/Magnetic-immunity-of-the-MTi-AHRS-1605869708053)
37.  [更改或重置 MTi 参考坐标系统](https://base.movella.com/s/article/article/Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643)
38.  [MTi 系列串行通信接口引入的延迟](https://base.movella.com/s/article/article/Delays-introduced-by-serial-communication-interfaces-for-the-MTi-series)
39.  [为什么加速度计测量重力时显示正号](https://base.movella.com/s/article/Why-does-an-accelerometer-measure-gravity-with-positive-sign)
40.  [如何获取正确的 RotSensor 矩阵](https://base.movella.com/s/article/How-to-obtain-the-correct-RotSensor-matrix)
41.  [如何使用以太网协议连接 MTi 设备](https://base.movella.com/s/article/How-to-connect-MTi-devices-using-Ethernet-protocol)
42.  [了解噪声特性之间的关系](https://base.movella.com/s/article/Understanding-the-relationship-between-the-noise-characteristics)

#### 固件

1.  [如何确定 MTi 的固件版本](https://base.movella.com/s/article/article/How-to-determine-the-Firmware-version-of-your-MTi-1605869707397)
2.  [使用带有测试版固件的固件更新程序](https://base.movella.com/s/article/article/Using-the-Firmware-Updater-with-a-beta-firmware)
3.  [出现预期错误时设备未处于引导加载程序中](https://base.movella.com/s/article/article/Device-was-not-in-bootloader-when-expected-error)
4.  [固件更新程序在扫描设备时冻结](https://base.movella.com/s/article/article/Firmware-Updater-freezes-while-scanning-for-devices)
5.  [固件更新程序日志](https://base.movella.com/s/article/article/Firmware-Updater-Log)
6.  [如何修复固件更新程序错误“无法写入 eMTS”](https://base.movella.com/s/article/article/How-to-fix-Firmware-Updater-error-Could-not-write-eMTS)
7.  [如何修复磁场映射器错误 286“无法开始记录”](https://base.movella.com/s/article/article/How-to-fix-Magnetic-Field-Mapper-Error-286-Failed-to-start-logging)
8.  [MTi 处于引导加载程序模式](https://base.movella.com/s/article/article/MTi-in-bootloader-mode)
9.  [MTi 发行说明和变更日志](https://base.movella.com/s/article/article/MTi-Release-Notes-and-Changelogs)
10.  [产品变更通知 (PCN) 和产品停产 (EoL) 通知](https://base.movella.com/s/article/article/Product-Change-Notifications-PCNs-and-End-of-Life-EoL-notices)

#### Xsens Sirius 系列

1.  [迁移：Xsens MTi 100 系列至 Xsens Sirius 系列](https://base.movella.com/s/article/Migration-Xsens-MTi-100-series-to-Xsens-Sirius-series)
2.  [Xsens Sirius 系列的 STEP 文件](https://base.movella.com/s/article/STEP-file-for-the-Xsens-Sirius-series)

#### MTi 10/100 – 系列

1.  [MTi 10 系列和 MTi 100 系列的 AD 分辨率](https://base.movella.com/s/article/article/AD-resolution-of-the-MTi-10-series-and-MTi-100-series-1605869706076)
2.  [迁移：MTi 10/100 系列至 MTi 600 系列](https://base.movella.com/s/article/article/Migration-MTi-10-100-series-to-MTi-600-series-1605870241697)
3.  [迁移至第五代 MTi 10/100 系列 (2017)](https://base.movella.com/s/article/article/Migration-to-5th-generation-MTi-10-100-series-1605869708410)
4.  [安装 MTi-OEM](https://base.movella.com/s/article/article/Mounting-the-MTi-OEM-1605869708435)
5.  [MTi 10/100 系列坐标系原点](https://base.movella.com/s/article/article/MTi-10-100-series-Origin-of-Coordinate-system)
6.  [MTi 10/100 系列套管的强度](https://base.movella.com/s/article/article/Strength-of-MTi-10-100-series-casing)
7.  [MTi 10/100 系列的 STEP 文件](https://base.movella.com/s/article/article/STEP-files-for-MTi-10-100-series)
8.  [使用带有更长电缆的 MTi](https://base.movella.com/s/article/article/Use-of-MTi-with-longer-cables)
9.  [MTi 10/100 系列连接器和电缆](https://base.movella.com/s/article/article/MTi-10-100-series-connectors-and-cables)
10.  [CA-MP2-MTi 多用途电缆 Molex 连接器零件编号](https://base.movella.com/s/article/article/CA-MP2-MTi-multi-purpose-cable-Molex-connector-part-numbers-1605869706131)

#### MTi 600 – 系列

1.  [使用带有更长电缆的 MTi](https://base.movella.com/s/article/article/Use-of-MTi-with-longer-cables)
2.  [我的 MTi-6XX 未被检测到/为 Linux 安装 MTi USB 加密狗驱动程序](https://base.movella.com/s/article/article/My-MTi-6XX-is-not-detected-Installing-the-MTi-USB-dongle-driver-for-Linux)
3.  [MTi 600 系列的 STEP 文件](https://base.movella.com/s/article/article/STEP-files-for-the-MTi-600-series)
4.  [如何从 MTi-680(G) 接收 RTCM 校正信息](https://base.movella.com/s/article/How-to-receive-RTCM-correction-messages-for-MTi-680-G)
5.  [RTK 简介](https://base.movella.com/s/article/article/Introduction-to-RTK)
6.  [解决 RTK GNSS 子卡的 USB 通信问题](https://base.movella.com/s/article/article/Solving-USB-communication-issues-for-the-RTK-GNSS-daughter-card)
7.  [MTi 600 系列的 .dbc 文件](https://base.movella.com/s/article/article/dbc-file-for-MTi-600-series-1605870241065)

#### MTi 1 – 系列

1.  [MTi 1 系列的 Altium 库和 STEP 文件](https://base.movella.com/s/article/article/Altium-library-and-STEP-file-for-MTi-1-series-1605870241073)
2.  [将 MTi 1 系列 DK 与 Arduino 连接](https://base.movella.com/s/article/article/Interfacing-the-MTi-1-series-DK-with-an-Arduino)
3.  [使用 MTi 1 系列模块，无需回流焊工艺](https://base.movella.com/s/article/article/Using-the-MTi-1-series-module-without-reflow-process)
4.  [MTi 1 系列硬件版本 1.x 固件更新](https://base.movella.com/s/article/article/MTi-1-series-Hardware-Version-1-x-Firmware-Updates)
5.  [MTi 1 系列坐标系原点](https://base.movella.com/s/article/article/MTi-1-series-Origin-of-Coordinate-system-1605870241700)
6.  [MTi 1 系列开发板的 mikroBUS（收发器）兼容性](https://base.movella.com/s/article/article/mikroBUS-transceiver-compatibility-of-the-MTi-1-series-Development-Board)
7.  [迁移至 MTi 1 系列硬件版本 2.0（2018 年 6 月）](https://base.movella.com/s/article/article/Migration-to-Hardware-Version-2-0-of-the-MTi-1-series-1605870241149)
8.  [迁移至 MTi 1 系列硬件版本 3（2021 年 8 月）](https://base.movella.com/s/article/article/Migration-to-Hardware-Version-3-of-the-MTi-1-series)
9.  [迁移至 MTi 1 系列硬件版本 5（2023 年 5 月）](https://base.movella.com/s/article/Migration-to-Hardware-Version-5-of-the-MTi-1-series)
10.  [MTi 1 系列的过滤配置文件](https://base.movella.com/s/article/article/Filter-profiles-for-MTi-1-series-1605870241087)
11.  [MTi 1 系列的 I2C 和 SPI 最佳实践](https://base.movella.com/s/article/article/Best-practices-I2C-and-SPI-for-MTi-1-series-1605869706124)
12.  [适用于 MTi 1 系列的嵌入式固件更新程序 (FWU)](https://base.movella.com/s/article/article/Embedded-Firmware-Updater-FWU-for-MTi-1-series)
13.  [迁移至 MTi-320 硬件版本 3（2023 年 6 月）](https://base.movella.com/s/article/Migration-to-Hardware-Version-3-of-the-MTi-320)
14.  [MTi-320 传感器的 STEP 文件](https://base.movella.com/s/article/STEP-files-for-the-MTi-320-sensor)

#### Xsens 视觉导航

1.  [Xsens Vision Navigator：常见问题 (FAQ)](https://base.movella.com/s/article/Xsens-Vision-Navigator-Frequently-Asked-Questions-FAQ)
2.  [Xsens Vision Navigator (XVN) 的 STEP 文件](https://base.movella.com/s/article/STEP-files-for-the-Xsens-Vision-Navigator-XVN)
3.  [在室内启动 Xsens Vision Navigator (XVN) 的 Fusion](https://base.movella.com/s/article/Starting-Fusion-Indoors-for-Xsens-Vision-Navigator-XVN)
4.  [更新 Xsens Vision Navigator (XVN) 的固件](https://base.movella.com/s/article/Updating-the-Firmware-of-the-Xsens-Vision-Navigator-XVN)
5.  [使用 Xsens Vision Navigator 进行相机图像流传输](https://base.movella.com/s/article/Camera-Image-Streaming-with-Xsens-Vision-Navigator)
6.  [生成 Xsens Vision Navigator (XVN) 的日志文件](https://base.movella.com/s/article/Generating-a-log-file-of-the-Xsens-Vision-Navigator-XVN)
7.  [Xsens Vision Navigator (XVN) 故障排除](https://base.movella.com/s/article/Troubleshooting-Xsens-Vision-Navigator-XVN)

## 第三方参考代码

### UART/RS232

#### Python：

-   [https://github.com/jiminghe/Xsens\_MTi\_Serial\_Reader](https://github.com/jiminghe/Xsens_MTi_Serial_Reader)

#### C++：

-   [https://github.com/jiminghe/Xsens\_MTi\_Serial\_Reader\_Linux](https://github.com/jiminghe/Xsens_MTi_Serial_Reader_Linux)
-   [https://github.com/ealkhatib/XsensUart](https://github.com/ealkhatib/XsensUart)

#### C 库（MTi-300 连接到 STM32F4）

-   [https://github.com/Scottapotamas/xsens-mti](https://github.com/Scottapotamas/xsens-mti)

### SPI：

-   [https://github.com/Steven-GH/Xsens\_MTi\_SPI](https://github.com/Steven-GH/Xsens_MTi_SPI)

### I2C：

-   Arduino MEGA R3 2560： https: [//github.com/jiminghe/Xsens\_MTi\_I2C\_Arduino](https://github.com/jiminghe/Xsens_MTi_I2C_Arduino)
-   Arduino UNO：[https://github.com/Steven-GH/Xsens\_MTi\_I2C](https://github.com/Steven-GH/Xsens_MTi_I2C)
-   树莓派 5： https: [//github.com/jiminghe/Xsens-MTi-1-Series-I2C-with-Raspberry-Pi-5](https://github.com/jiminghe/Xsens-MTi-1-Series-I2C-with-Raspberry-Pi-5)
-   STM32F401RE Nucleo 开发板：[https://github.com/jiminghe/Xsens\_MTi\_I2C\_STM32F4/](https://github.com/jiminghe/Xsens_MTi_I2C_STM32F4/)

### 其他资源：

-   ROS 1 驱动程序（CANBus）：[https://github.com/jiminghe/Xsens\_MTi\_CAN\_ROS\_Driver](https://github.com/jiminghe/Xsens_MTi_CAN_ROS_Driver)
-   XVN Rosbag 解析（python）：[https://github.com/jiminghe/XVN\_Rosbag\_Parser](https://github.com/jiminghe/XVN_Rosbag_Parser)
-   XDA PUBLIC CPP 和 CMakeList.txt：[https://github.com/jiminghe/xda\_public\_cpp/tree/cmake](https://github.com/jiminghe/xda_public_cpp/tree/cmake)

## 产品宣传单

### 1系列：

-   [MTi-1](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-1.pdf)
-   [MTi-2](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-2.pdf)
-   [MTi-3](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-3.pdf)
-   [MTi-7](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-7.pdf)
-   [MTi-8](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-8.pdf)
-   [MTi-320](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-320.pdf)

### 600 系列：

-   [MTi-610](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-610.pdf)
-   [MTi-620](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-620.pdf)
-   [MTi-630](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630.pdf)
-   [MTi-630R](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-630R.pdf)
-   [MTi-670](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-670.pdf)
-   [MTi-670G](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-670G.pdf)
-   [MTi-680](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-680.pdf)
-   [MTi-680G](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-680G.pdf)

### 100 系列：

-   [MTi-100](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-100.pdf)
-   [MTi-200](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-200.pdf)
-   [MTi-300](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-300.pdf)
-   [MTi-G-710](https://www.movella.com/hubfs/Downloads/Leaflets/MTi-G-710.pdf)

### Xsens Sirius系列：

- [Sirius IMU](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20IMU.pdf)
- [Sirius VRU](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20VRU.pdf)
- [Sirius AHRS](https://www.movella.com/hubfs/Downloads/Manuals/MTi%20product%20leaflet_Sirius%20AHRS.pdf)

### Xsens Vision Navigator
[Xsens Vision Navigator](https://www.movella.com/hubfs/Downloads/Leaflets/Xsens%20Vision%20Navigator.pdf)

## MTi 白皮书：

[MTi 白皮书](https://www.movella.com/hubfs/Downloads/Whitepapers/MTi_whitepaper.pdf)