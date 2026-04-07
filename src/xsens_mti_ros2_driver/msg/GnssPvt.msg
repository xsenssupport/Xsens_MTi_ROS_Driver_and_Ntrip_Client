# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message combines Position, velocity and time solution in LLH,
# including accuracy figures
#
# Field names mirror ublox_msgs/NavPVT for interoperability.
# Fixed output rate from Xsens GNSS/INS devices is 4 Hz.
# Supported models: MTi-7, MTi-8, MTi-670(G), MTi-680(G), MTi-G-710
#

std_msgs/Header header

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 i_tow            # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid             # Validity flags
uint8 VALID_DATE = 1            # Valid UTC Date
uint8 VALID_TIME = 2            # Valid
uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved
uint8 VALID_MAG = 8             # Valid Magnetic Declination

uint32 t_acc            # time accuracy estimate [ns] (UTC)
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)

uint8 fix_type          # GNSS fix Type, range 0..5
uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4
uint8 FIX_TYPE_TIME_ONLY = 5

uint8 flags             # Fix Status Flags
uint8 FLAGS_GNSS_FIX_OK = 1
uint8 FLAGS_DIFF_SOLN = 2
uint8 FLAGS_PSM_MASK = 28
uint8 PSM_OFF = 0
uint8 PSM_ENABLED = 4
uint8 PSM_ACQUIRED = 8
uint8 PSM_TRACKING = 12
uint8 PSM_POWER_OPTIMIZED_TRACKING = 16
uint8 PSM_INACTIVE = 20
uint8 FLAGS_HEAD_VEH_VALID = 32
uint8 FLAGS_CARRIER_PHASE_MASK = 192
uint8 CARRIER_PHASE_NO_SOLUTION = 0
uint8 CARRIER_PHASE_FLOAT = 64
uint8 CARRIER_PHASE_FIXED = 128

uint8 flags2            # Additional Flags
uint8 FLAGS2_CONFIRMED_AVAILABLE = 32
uint8 FLAGS2_CONFIRMED_DATE = 64
uint8 FLAGS2_CONFIRMED_TIME = 128

uint8 num_sv            # Number of SVs used in Nav Solution
int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 h_msl             # Height above mean sea level [mm]
uint32 h_acc            # Horizontal Accuracy Estimate [mm]
uint32 v_acc            # Vertical Accuracy Estimate [mm]

int32 vel_n             # NED north velocity [mm/s]
int32 vel_e             # NED east velocity [mm/s]
int32 vel_d             # NED down velocity [mm/s]
int32 g_speed           # Ground Speed (2-D) [mm/s]
int32 heading           # Heading of motion 2-D [deg / 1e-5]
uint32 s_acc            # Speed Accuracy Estimate [mm/s]
uint32 head_acc         # Heading Accuracy Estimate [deg / 1e-5]

uint16 p_dop            # Position DOP [1 / 0.01]
uint8[6] reserved1      # Reserved

int32 head_veh          # Heading of vehicle (2-D) [deg / 1e-5]
int16 mag_dec           # Magnetic declination [deg / 1e-2]
uint16 mag_acc          # Magnetic declination accuracy [deg / 1e-2]
