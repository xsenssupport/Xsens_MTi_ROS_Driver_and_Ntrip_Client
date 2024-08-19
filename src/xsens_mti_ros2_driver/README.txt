=[ Xsens MTi driver for ROS2 ]============================================================

Documentation:
    You can find the full documentation in "<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html".

Prerequisites:
    - ROS Humble or Jazzy
    - C/C++ Compiler: GCC 5.4.0
    - C++11

Building:
    - Copy xsens_ros2_mti_driver folder and ntrip folder(for MTi-680(G)/MTi-8 only) into your colcon workspace 'src' folder, if you have the RTK GNSS/INS sensor model like MTi-8 or MTi-680/680G, please also copy the ntrip folder.
        Make sure the permissions are set to o+rw on your files and directories.
        $ sudo chmod -R o+rw xsens_ros2_mti_driver/

    - Build xspublic from your catkin workspace:
        $ pushd src/xsens_mti_ros2_driver/lib/xspublic && make && popd

    - Build Xsens MTi ROS2 Driver package:
        $ colcon build

    - Source workspace:
        $ source install/setup.bash

Running:
    - Configure your MTi device to output desired data (e.g. for display example - orientation output)

    - Launch the Xsens MTi driver from your colcon workspace:
            $ ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

        After the device has been detected, you can communicate with it from another process / terminal window.
        For example:
            $ ros2 topic echo /filter/quaternion
        This will result in a continuous stream of data output:
            ---
            header: 
              seq: 1386351
              stamp: 
                secs: 1545223809
                nsecs: 197252179
              frame_id: "imu_link"
            quaternion: 
              x: 0.00276306713931
              y: 0.00036825647112
              z: -0.89693570137
              w: -0.442152231932
            ---

    - There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device):
            $ ros2 launch xsens_mti_ros2_driver display.launch.py


-[ Troubleshooting ]------------------------------------------------------------

    - The Mti1 (Motion Tracker Development Board) is not recognized.

        Support for the Development Board is present in recent kernels. (Since June 12, 2015).
        If your kernel does not support the Board, you can add this manually

        $ sudo /sbin/modprobe ftdi_sio
        $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id

    - The MTi-10/20/30 or MTi-100/200/300 or MTi-G-710 is not recognized.
        $ git clone https://github.com/xsens/xsens_mt.git
        $ cd xsens_mt
        
        You may need to install the kernel headers for 
        your running kernel version to ensure that you have all the necessary files to 
        compile the module:
	   $sudo apt-get install linux-headers-$(uname -r)
   
	 Then:
	   $sudo make HAVE_LIBUSB=1
	   $sudo modprobe usbserial
	   $sudo insmod ./xsens_mt.ko

    - The device is recognized, but I cannot ever access the device -

        Make sure you are in the correct group (often dialout or uucp) in order to
        access the device. You can test this with

            $ ls -l /dev/ttyUSB0
            crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
            $ groups
            dialout audio video usb users plugdev

        If you aren't in the correct group, you can fix this in two ways.

        1. Add yourself to the correct group
            You can add yourself to it by using your distributions user management
            tool, or call

                $ sudo usermod -G dialout -a $USER

            Be sure to replace dialout with the actual group name if it is
            different. After adding yourself to the group, either relogin to your
            user, or call

                $ newgrp dialout

            to add the current terminal session to the group.

        2. Use udev rules
            Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

                SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

            Change $GROUP into your desired group (e.g. adm, plugdev, or usb).
            
        3. Change the port parameters in the xsens_mti_node.yaml
            Change the parameters to below:
                $ scan_for_devices: false
                $ port: 'dev/ttyUSB0' #Uncomment this line and change the value to the exact port name of the MTi on your device.
                $ baudrate: 115200 #Uncomment this line and change the exact value that you had previously configured with MT Manager, 115200 is factory default value.
                

    - The device is inaccessible for a while after plugging it in -

        When having problems with the device being busy the first 20 seconds after
        plugin, purge the modemmanager application.

    - RViz doesn't show an MTi model.

        It is a known issue with urdfdom in ROS Melodic. A workaround is to unset/modify the LC_NUMERIC environment variable:

        $ LC_NUMERIC="en_US.UTF-8"
