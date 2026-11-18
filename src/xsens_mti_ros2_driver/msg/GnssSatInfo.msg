# XsRawGnssSatInfo.msg
# Maps to XsRawGnssSatInfo / XsSatInfo structs from xsrawgnsssatinfo.h
# Corresponds to Xsens XDI_GnssSatInfo
#
# Wire layout:
#   [0..3]   itow    U4  GPS time of week [ms]
#   [4]      num_svs U1  number of satellites (max 60)
#   [5]      res1    U1  reserved
#   [6]      res2    U1  reserved
#   [7]      res3    U1  reserved
#   [8+4*N]  gnss_id U1  GNSS identifier for satellite N
#   [9+4*N]  sv_id   U1  satellite identifier for satellite N
#   [10+4*N] cno     U1  carrier to noise ratio [dBHz] for satellite N
#   [11+4*N] flags   U1  flags for satellite N

# --- Header ---

# GPS time of week [ms]
uint32 itow

# Number of satellites reported (max 60)
uint8 num_svs

# Reserved for future use
uint8 res1
uint8 res2
uint8 res3

# --- Per-satellite data (repeated num_svs times, max 60) ---
# Stored as flat parallel arrays of length 60, preserving the
# 4-byte-per-satellite wire layout of XsSatInfo[60].
# Only indices [0..num_svs-1] contain valid data.

# GNSS identifier (XsSatInfo.m_gnssId)
#   0 = GPS, 1 = SBAS, 2 = Galileo, 3 = BeiDou, 4 = IMES, 5 = QZSS, 6 = GLONASS
uint8[60] gnss_id

# Satellite identifier (XsSatInfo.m_svId)
uint8[60] sv_id

# Carrier to noise ratio / signal strength [dBHz] (XsSatInfo.m_cno)
uint8[60] cno

# Flags bitmask (XsSatInfo.m_flags / XsSatInfoFlags)
#
# bits[0..2] Signal quality indicator (mask 0x07):
#   0 = no signal
#   1 = searching
#   2 = acquired
#   3 = detected but unusable
#   4 = code locked and time synchronized
#   5,6,7 = code and carrier locked and time synchronized
# bit[3]     SV used for navigation         (mask 0x08)
# bits[4..5] SV health flag                 (mask 0x30):
#   0x00 = unknown, 0x10 = healthy, 0x20 = unhealthy
# bit[6]     differential correction available (mask 0x40)
# bit[7]     reserved

uint8 FLAGS_SIGNAL_QUALITY_MASK              = 7   # 0x07
uint8 FLAGS_SIGNAL_QUALITY_NO_SIGNAL         = 0   # 0x00
uint8 FLAGS_SIGNAL_QUALITY_SEARCHING         = 1   # 0x01
uint8 FLAGS_SIGNAL_QUALITY_ACQUIRED          = 2   # 0x02
uint8 FLAGS_SIGNAL_QUALITY_UNUSABLE          = 3   # 0x03
uint8 FLAGS_SIGNAL_QUALITY_CODE_TIME_OK      = 4   # 0x04
uint8 FLAGS_SIGNAL_QUALITY_CODE_CARRIER_OK1  = 5   # 0x05
uint8 FLAGS_SIGNAL_QUALITY_CODE_CARRIER_OK2  = 6   # 0x06
uint8 FLAGS_SIGNAL_QUALITY_CODE_CARRIER_OK3  = 7   # 0x07
uint8 FLAGS_USED_FOR_NAVIGATION_MASK         = 8   # 0x08
uint8 FLAGS_USED_FOR_NAVIGATION              = 8   # 0x08
uint8 FLAGS_HEALTH_MASK                      = 48  # 0x30
uint8 FLAGS_HEALTH_UNKNOWN                   = 0   # 0x00
uint8 FLAGS_HEALTH_HEALTHY                   = 16  # 0x10
uint8 FLAGS_HEALTH_UNHEALTHY                 = 32  # 0x20
uint8 FLAGS_DIFFERENTIAL_MASK                = 64  # 0x40
uint8 FLAGS_DIFFERENTIAL_AVAILABLE           = 64  # 0x40

uint8[60] flags