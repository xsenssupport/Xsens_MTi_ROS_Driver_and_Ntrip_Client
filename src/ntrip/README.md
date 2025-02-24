# ROS 2 NTRIP Client

A ROS 2 package that implements a NTRIP (Networked Transport of RTCM via Internet Protocol) client for receiving RTCM correction data from NTRIP casters. This client can be used with GNSS receivers to obtain RTK corrections.

## Features

- Connects to NTRIP casters using NTRIP protocol version 2.0
- Supports authentication
- Handles RTCM3 messages
- Publishes RTCM messages for use with ROS 2-compatible GNSS receivers
- Automatic reconnection on connection loss
- Periodic GGA position updates to caster
- Diagnostic information publishing
- Detailed debug output options

## Dependencies

### ROS 2 Packages
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

1. Create a ROS 2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ntrip
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

Launch the NTRIP client using the provided launch file:

```bash
ros2 launch ntrip ntrip_launch.py
```

To run the ntrip node with debug mode:
```bash
ros2 launch ntrip ntrip_launch.py log_level:=debug
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

- **Debug Options**
  - `debug`: Enable detailed debug output


### Topics

#### Subscribed Topics
- `nmea` ([nmea_msgs/msg/Sentence](https://github.com/ros2/common_interfaces/blob/master/nmea_msgs/msg/Sentence.msg)): NMEA GGA messages to send to the NTRIP caster

#### Published Topics
- `/rtcm` ([mavros_msgs/msg/RTCM](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/RTCM.msg)): RTCM correction data received from the NTRIP caster
- `/diagnostics` ([diagnostic_msgs/msg/DiagnosticArray](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticArray.msg)): Connection and message statistics


## Troubleshooting

- If you're having connection issues, make sure your network allows access to the NTRIP caster's host and port
- Check the ROS 2 logs for detailed error messages when debug mode is enabled
- Verify your credentials and mountpoint information with your NTRIP service provider
- Use `ros2 topic echo` commands to monitor the published topics
- Use `ros2 param list` and `ros2 param get` to verify parameter settings

## Additional ROS 2 Commands

View available topics:
```bash
ros2 topic list
```

Monitor RTCM messages:
```bash
ros2 topic echo /rtcm
```

Check node parameters:
```bash
ros2 param list /ntrip
```

View diagnostic messages:
```bash
ros2 topic echo /diagnostics
