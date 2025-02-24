# ROS NTRIP Client

A ROS package that implements a NTRIP (Networked Transport of RTCM via Internet Protocol) client for receiving RTCM correction data from NTRIP casters. This client can be used with GNSS receivers to obtain RTK corrections.

## Features

- Connects to NTRIP casters using NTRIP protocol version 2.0
- Supports authentication
- Handles RTCM3 messages
- Publishes RTCM messages for use with ROS-compatible GNSS receivers
- Automatic reconnection on connection loss
- Periodic GGA position updates to caster
- Diagnostic information publishing
- Detailed debug output options

## Dependencies

### ROS Packages
```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros-msgs
sudo apt-get install ros-${ROS_DISTRO}-nmea-msgs
sudo apt-get install ros-${ROS_DISTRO}-diagnostic-msgs
```

### System Dependencies
```bash
sudo apt-get install libboost-all-dev
```

## Installation

1. Create a catkin workspace (if you don't have one):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source the workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

Launch the NTRIP client using the provided launch file:

```bash
roslaunch ntrip ntrip.launch
```

### Launch File Parameters

- **Server Configuration**
  - `host`: NTRIP caster hostname or IP address
  - `port`: NTRIP caster port number
  - `mountpoint`: NTRIP mountpoint name
  - `username`: Authentication username
  - `password`: Authentication password

- **Connection Parameters**
  - `update_rate`: Rate at which to check connection status (Hz) and Interval for sending GGA messages to caster (seconds)
  - `reconnect_delay`: Delay between reconnection attempts (seconds)
  - `max_reconnect_attempts`: Maximum number of reconnection attempts (0 for infinite)

- **GGA Configuration**
  - `send_default_gga`: Whether to send default GGA when no NMEA input available

- **Debug Options**
  - `debug`: Enable detailed debug output
  - `output_rtcm_details`: Enable RTCM message details output

### Topics

#### Subscribed Topics
- `nmea` ([nmea_msgs/Sentence](http://docs.ros.org/api/nmea_msgs/html/msg/Sentence.html)): NMEA GGA messages to send to the NTRIP caster

#### Published Topics
- `rtcm` ([mavros_msgs/RTCM](http://docs.ros.org/api/mavros_msgs/html/msg/RTCM.html)): RTCM correction data received from the NTRIP caster
- `/ntrip/diagnostics` ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html)): Connection and message statistics

## Troubleshooting

- If you're having connection issues, make sure your network allows access to the NTRIP caster's host and port
- Check the ROS logs for detailed error messages when debug mode is enabled
- Verify your credentials and mountpoint information with your NTRIP service provider
