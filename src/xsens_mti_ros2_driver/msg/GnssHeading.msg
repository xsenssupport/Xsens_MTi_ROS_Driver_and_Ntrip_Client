# Dual-antenna GNSS heading (XDI_GnssHeading, 0x7040)
#
# Comparable in content to u-blox NAV-RELPOSNED, but in Xsens units and flag bits:
# see the MT Low Level Communication Protocol for the definition of this output.
#
# Relative position of the secondary antenna (ANT2) with respect to the primary
# antenna (ANT1) in the local North-East-Down frame, and the heading of that baseline.
# Values are converted from the raw device units (mm, 1e-5 deg) to meters and degrees.
#
# Only use rel_pos_* when rel_pos_valid is true, and rel_pos_heading when heading_valid is true.
# Supported models: Sirius/Avior Dual Antenna GNSS/INS
#

std_msgs/Header header

uint32 i_tow                # GNSS time of week [ms]

float64 rel_pos_n           # North component of the baseline ANT1 -> ANT2 [m]
float64 rel_pos_e           # East component of the baseline ANT1 -> ANT2 [m]
float64 rel_pos_d           # Down component of the baseline ANT1 -> ANT2 [m]
float64 rel_pos_length      # Length of the baseline ANT1 -> ANT2 [m]
float64 rel_pos_heading     # Angle between True North and the baseline projected onto the horizontal plane [deg]

float64 acc_n               # Accuracy of rel_pos_n [m]
float64 acc_e               # Accuracy of rel_pos_e [m]
float64 acc_d               # Accuracy of rel_pos_d [m]
float64 acc_length          # Accuracy of rel_pos_length [m]
float64 acc_heading         # Accuracy of rel_pos_heading [deg]

# Decoded status flags
bool gnss_fix_ok                # Valid GNSS fix
bool diff_corrections_applied   # Differential corrections (RTK) were applied
bool rel_pos_valid              # rel_pos_* and acc_* are valid
bool heading_valid              # rel_pos_heading and acc_heading are valid

uint8 carrier_solution          # Carrier phase range solution
uint8 CARRIER_SOLUTION_NONE = 0     # No carrier phase range solution
uint8 CARRIER_SOLUTION_FLOAT = 1    # Floating ambiguities
uint8 CARRIER_SOLUTION_FIXED = 2    # Fixed ambiguities

# Raw status flags as received from the device
uint32 flags
uint32 FLAGS_GNSS_FIX_OK = 1
uint32 FLAGS_DIFF_CORRECTIONS_APPLIED = 2
uint32 FLAGS_REL_POS_VALID = 4
uint32 FLAGS_CARRIER_SOLUTION_MASK = 24     # 2 bits, shift right by 3 for carrier_solution
uint32 FLAGS_HEADING_VALID = 64
