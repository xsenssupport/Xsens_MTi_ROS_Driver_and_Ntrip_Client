# GNSS/INS odometry frame correction

## ros2_dev port status

Target baseline: 3b63ebb6643396fc184a8a0f8d08bd36fa333e56 (local ros2_dev tracking reference).
Ported from validated ros2 fix commit 90a1949. The publisher uses ros2_dev's
existing rclcpp::Node and Publisher types and compatible TF headers; existing
GNSS PVT/satellite parameters and test targets are preserved.
All hardware results below belong to the original ros2 implementation.
The user ran tests for this port in a separate ROS 2 Jazzy workspace on
2026-09-30: both CTest targets passed (9 existing GTest cases and the new
local_enu_regression executable). colcon test-result reported 11 tests, 0 errors,
0 failures and 0 skipped; the aggregate includes test wrapper accounting.
The supplied transcript contains test output, not the preceding driver build log.
No hardware rerun of this port is claimed. Remote freshness remains unconfirmed
until a successful fetch.


Baseline: 61dc6ad596002aa3ee398b508706933c77c2dc12 (ros2 after PR #57). The original odometry
publisher was added by 8eba7a83d080a15ebbabcbeae3a0386350ca09a5.

## Message contract (breaking change)

`pub_odometry: true` publishes `/odometry` as a sensor measurement:
- `header.frame_id`: `odometry_frame_id`, default `local_enu`.
- `child_frame_id`: `frame_id`, default `imu_link`.
- Position: WGS84 ECEF converted to fixed ENU at the first valid sample.
- Orientation: sensor attitude relative to that same fixed ENU frame.
- Twist: linear and angular velocity in sensor axes, as required by Odometry.

This replaces the former relative UTM coordinates and erroneous imu_link-to-base_link
relationship. The startup origin resets when the publisher is recreated. Height is
WGS84 ellipsoidal height. ENU conversion is explicit even for NED/NWU device output.
Do not combine different startup origins under the same frame name without alignment.
GNSS corrections can jump; this measurement does not guarantee continuous `odom`.
Covariances remain unspecified (zero-filled); configure measurement uncertainty in the
consumer. Finite/range checks do not replace application-specific GNSS quality gating.

## TF ownership

`pub_odometry_tf` defaults to false. Enabling odometry on a GNSS device suppresses
legacy `pub_transform`, including when that parameter remains true in an old YAML.
No `odom_init`, `base_link`, or static TF is published by the odometry publisher.
For standalone visualization only, `pub_odometry_tf: true` publishes exactly one
`local_enu -> imu_link` transform per valid sample. Do not enable it when imu_link
already has a parent in the robot's TF tree. With odometry disabled, legacy
orientation-only `world -> imu_link` visualization is unchanged.

For a robot, keep both driver TF options false and provide measured
`base_link -> imu_link` through URDF/robot_state_publisher or a static broadcaster.
The localization system must transform sensor measurements using those extrinsics,
including the translation/lever arm, and own `map -> odom`; the continuous odometry
source owns `odom -> base_link`. Align the startup local_enu frame with map in that
system. Do not simply rename local_enu to map unless their origins and axes agree.
Device-configured alignment and lever-arm compensation must be accounted for when
identifying the frame/reference point of the measurements; avoid compensating twice.
An optional `earth -> map` requires a real ECEF georeference, not UTM translations.
This driver does not manufacture earth/map/odom transforms or vehicle mounting data.

## Validation

Build and run in a supported ROS 2 environment:

```sh
colcon build --packages-select xsens_mti_ros2_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select xsens_mti_ros2_driver
colcon test-result --verbose
```

The C++ regression target checks first-sample origin, ENU axis/height direction,
90-degree and arbitrary attitude preservation, fixed-origin attitude consistency,
body-frame velocity, dateline/equator continuity, poles, and invalid coordinates.

## Final validation record (2026-09-30)

Baseline: ros2 61dc6ad596002aa3ee398b508706933c77c2dc12, including PR #57.
Test hardware: Sirius RTK S1R43A, firmware 1.6.0, Ubuntu VM / ROS 2 Jazzy.
User-run evidence:
- Driver build succeeded. CTest local_enu_regression: 1 passed, 0 failed.
- Final receiver configuration correction rebuilt and passed the same test;
  device output configuration and lifecycle activation completed without the
  previous GNSS platform error.
- TF enabled: one driver broadcaster, local_enu -> imu_link; no /tf_static.
- TF disabled with legacy pub_transform still true: no /tf or /tf_static,
  while /odometry continued publishing in local_enu.
- Approximately 90-degree clockwise physical rotation changed TF yaw from
  94.74 to 3.94 degrees (-90.80 degrees), without double rotation.
- Outdoor stationary bag stationary_rtk_fixed_03: all 1629 status messages
  reported RTK status 2, GNSS fix and filter valid; all 466 GGA messages had
  quality 4. Odometry: 1163 samples at approximately 10 Hz with no duplicate
  or backward timestamps. Peak-to-peak E/N/U position variation was
  1.49/2.28/4.79 cm; mean/max speed magnitude was 0.0157/0.0210 m/s.
- RTCM: 776 messages with a maximum recorded arrival gap of 1.31 seconds.
  RTCM headers are zero-stamped, so continuity used bag recording timestamps.
- Recorded LLA-derived relative displacement matches odometry to within
  0.000000337 m (first recorded point used for approximate ENU tangent axes).
  This measures numerical consistency, not GNSS accuracy. Three status timestamp
  intervals were zero; odometry timestamps were strictly increasing.

Sirius/Avior RTK IDs match isRtk() separately from isGnss(). Both are accepted
for GNSS/INS publishers and output configuration. Receiver-specific u-blox
platform/BeiDou settings retain the original isGnss() condition. No SDK device
classification behavior is changed.

## Configuration and test overrides

Both YAML profiles retain their existing defaults, including publication flags,
output rates and the commented frame_id. Only odometry_frame_id: local_enu and
pub_odometry_tf: false are added. Odometry remains opt-in via pub_odometry: true.
When GNSS/INS odometry is enabled, legacy pub_transform is suppressed even if
its original true value is retained. With odometry disabled, legacy behavior
is preserved.

The Sirius RTK bench tests used explicit overrides: pub_odometry true,
output_data_rate and output_data_rate_lower 10, pub_ship_motion false, and
frame_id imu_link. TF was enabled only for standalone rotation/TF checks and
disabled for integration-style checks. These test settings are not new defaults.
Set pub_transform false as well when integrating with an external robot TF tree.

Changing requested rates does not change device outputs while enable_deviceConfig
is false. For a fresh device, configure quaternion, rate of turn, latitude/longitude,
ellipsoidal altitude and velocity at matching rates, plus status/time and GNSS PVT
for NMEA. Device configuration also applies other YAML settings: review mounting,
lever arm and option flags before enabling it. Once configured, it can be disabled
for subsequent launches. Verify installed parameters after deployment.

## Limits and untested integration

Static variation above is repeatability over about two minutes, not surveyed
absolute accuracy. Indoor/window recordings showed meter-scale variation already
present in the sensor LLA output; those captures were not fixed-solution runs.
No vehicle-level earth/map/odom/base_link graph, nonzero mounting extrinsics,
dynamic accuracy, ENU/NED/NWU device-output variants or loss/reacquisition test
has been completed. The C++ mathematical test covers coordinate rotation, not
full ROS publisher automation. Legacy orientation-only behavior with odometry
disabled was preserved by inspection but not retested on hardware.

The first finite complete measurement defines the origin; initialization does
not wait for GNSS convergence. GNSS corrections may jump. Covariance is still
zero-filled/unspecified and must not be interpreted as zero uncertainty. Configure
consumer uncertainty and quality gating before localization use. Do not claim
that this sensor measurement publisher alone implements a complete REP 105
localization stack.
