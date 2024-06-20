# Define the custom XsStatusWord message
bool selftest
bool filter_valid
bool gnss_fix
uint8 no_rotation_update_status ##0: not running, 2: aborted, 3:running
bool representative_motion
bool clock_bias_estimation
#bool reserved1
bool clipflag_acc_x
bool clipflag_acc_y
bool clipflag_acc_z
bool clipflag_gyr_x
bool clipflag_gyr_y
bool clipflag_gyr_z
bool clipflag_mag_x
bool clipflag_mag_y
bool clipflag_mag_z
#uint8 reserved2
bool clipping_indication
#bool reserved3
bool syncin_marker
bool syncout_marker
uint8 filter_mode ##0: Without GNSS, 1: Coasting, 3: With GNSS
bool have_gnss_time_pulse
uint8 rtk_status ##0: No RTK, 1: RTK Floating, 2: RTK Fix
#uint8 reserved4

########################
####ref to MT Low Level Protocol Communication: https://mtidocs.xsens.com/messages
####1) Bit 0: Selftest, This flag indicates if the MT passed the self-test according to eMTS(electronic Motion Tracker Specification).
####2) Bit 1: Filter Valid, This flag indicates if input into the orientation filter is reliable and / or complete.
####3) Bit 2: GNSS fix, This flag indicates if the GNSS unit has a proper fix. 
####4) Bit 3:4: NoRotationUpdate Status, This flag indicates the status of the no rotation update procedure in the filter after the SetNoRotation message has been sent. 11: Running with no rotation assumption; 10: Rotation detected, no gyro bias estimation (sticky); 00: Estimation complete, no errors.
####5) Bit 5: Representative Motion (RepMo), Indicates if the MTi is in In-run Compass Calibration Representative Mode
####6) Bit 6: Clock Bias Estimation (ClockSync), Indicates that the Clock Bias Estimation synchronization feature is active
####7) Bit 7: Reserved, Reserved for future use
####8) Bit 8: Clipflag Acc X, If set, an out of range acceleration on the X axis is detected
####9) Bit 9: Clipflag Acc Y, If set, an out of range acceleration on the Y axis is detected
####10) Bit 10: Clipflag Acc Z, If set, an out of range acceleration on the Z axis is detected
####11) Bit 11: Clipflag Gyr X, If set, an out of range angular velocity on the X axis is detected
####12) Bit 12: Clipflag Gyr Y, If set, an out of range angular velocity on the Y axis is detected
####13) Bit 13: Clipflag Gyr Z, If set, an out of range angular velocity on the Z axis is detected
####14) Bit 14: Clipflag Mag X, If set, an out of range magnetic field on the X axis is detected
####15) Bit 15: Clipflag Mag Y, If set, an out of range magnetic field on the Y axis is detected
####16) Bit 16: Clipflag Mag Z, If set, an out of range magnetic field on the Z axis is detected
####17) Bit 17:18, Reserved, Reserved for future use
####18) Bit 19, Clipping Indication, This flag indicates going out of range of one of the sensors (it is set when one or more bits from 8:16 are set)
####19) Bit 20,  Reserved, Reserved for future use
####20) Bit 21, SyncIn Marker, When a SyncIn is detected, this bit will rise to 1. 
####21) Bit 22, SyncOut Marker, When SyncOut is active, this bit will rise to 1.
####22) Bit 23:25, Filter Mode, Indicates Filter Mode, currently only available for GNSS/INS devices:000: Without GNSS (filter profile is in VRU mode); 001: Coasting mode (GNSS has been lost <60 sec ago);011: With GNSS (default mode)
####23) Bit 26, HaveGnssTimePulse, Indicates that the 1PPS GNSS time pulse is present
####24) 27:28, RtkStatus, Indicates the availability and status of RTK: 00: No RTK; 01: RTK floating; 10: RTK fixed
####25) 29:31, Reserved, Reserved for future use
